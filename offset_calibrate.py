#!/usr/bin/env python3

"""
Calibrates direction + offset for each motor in a step-by-step manner.

For each motor (turning or rolling):
1) Clear errors
2) Enter CLOSED_LOOP_CONTROL
3) Send a small positive velocity for a few seconds
   - Continuously request encoder data, printing "pos=xxx" so you see if it increases or decreases
   - Observe physically which way it's turning
4) Ask if that is the correct forward direction (y/n). If no => flip=-1, else flip=+1
5) For turning motors, prompt user to align motor at mechanical zero, read encoder once => offset
6) Return motor to IDLE
7) Save to JSON { node_id: { "flip": ±1.0, "offset": ... }, ... }

At the end, you have a config file you can load in your main script to apply flips + offsets.
"""

import can
import struct
import json
import time
import sys
import select

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CONFIG_FILE = "motor_config.json"

# CAN function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
SET_INPUT_VEL = 0x0D
GET_ENCODER_ESTIMATES = 0x09
HEARTBEAT = 0x01

# Axis states
CLOSED_LOOP = 8
IDLE = 1

# We'll apply a small velocity to test direction
TEST_VELOCITY = 0.1  # turns/s (adjust if too fast or too slow)
TEST_DURATION = 3.0  # seconds to run the direction test

bus = can.interface.Bus("can0", bustype="socketcan")

def to_arbitration_id(node_id, func_code):
    """Helper to build arbitration ID for ODrive CANSimple."""
    return (node_id << 5) | func_code

def send_set_axis_state(node_id, state):
    """Sends Set_Axis_State command."""
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False
    )
    bus.send(msg)

def send_clear_errors(node_id):
    """Sends Clear_Errors command."""
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, CLEAR_ERRORS),
        data=[],
        is_extended_id=False
    )
    bus.send(msg)

def send_input_vel(node_id, vel, torque_ff=0.0):
    """Sends Set_Input_Vel command."""
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_INPUT_VEL),
        data=struct.pack('<ff', vel, torque_ff),
        is_extended_id=False
    )
    bus.send(msg)

def request_encoder_once(node_id, timeout=0.3):
    """
    Sends one Get_Encoder_Estimates (0x09) request to 'node_id'.
    Waits up to 'timeout' seconds for the reply (node<<5 | (0x09|0x10)).
    Returns (pos, vel) or None if no response.
    """
    req_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES)
    resp_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES | 0x10)

    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))

    start = time.time()
    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            return (pos, vel)
    return None

def wait_for_closed_loop(node_id, timeout=2.0):
    """
    Wait until the motor reports AxisState=CLOSED_LOOP_CONTROL in heartbeat.
    Returns True if successful, False if timed out.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.2)
        if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
            n_id = msg.arbitration_id >> 5
            if n_id == node_id:
                axis_error, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if axis_state == CLOSED_LOOP:
                    return True
    return False

def direction_test(node_id):
    """
    1) Apply small positive velocity for TEST_DURATION seconds
    2) Repeatedly request encoder and print pos changes
    3) Ask user if it's correct forward direction => returns +1 or -1
    """
    print(f"   → Setting velocity={TEST_VELOCITY} for ~{TEST_DURATION} seconds...")
    send_input_vel(node_id, TEST_VELOCITY)

    t0 = time.time()
    prev_pos = None
    while (time.time() - t0) < TEST_DURATION:
        res = request_encoder_once(node_id)
        if res is not None:
            pos, vel = res
            if prev_pos is not None:
                diff = pos - prev_pos
                print(f"   Node {node_id} pos={pos:.3f} (delta={diff:.3f})")
            else:
                print(f"   Node {node_id} pos={pos:.3f}")
            prev_pos = pos
        else:
            print("   No encoder response (check if firmware supports requests in this state).")

        # Sleep a bit
        time.sleep(0.4)

    # Stop movement
    send_input_vel(node_id, 0.0)
    # Ask if direction is correct
    ans = input(f"\n   Did motor {node_id} rotate FORWARD physically? (y/n) [y]: ")
    if ans.strip().lower().startswith("n"):
        flip = -1.0
        print("   → We'll flip direction for this motor.")
    else:
        flip = 1.0
        print("   → No flip needed.")
    return flip

def calibrate_turning_offset(node_id, flip_sign):
    """
    Prompt user to physically align turning motor at zero,
    read encoder once => offset = that raw position.
    We'll store that raw offset in the config. Then in your control script you do:
       final_angle = offset + flip*(desired_angle)
    """
    input(f"\n   Align TURNING motor {node_id} to your mechanical ZERO, then press ENTER...")

    res = request_encoder_once(node_id)
    if res is None:
        print("   ⚠️ No encoder response => offset=0.0")
        return 0.0
    pos, _ = res
    print(f"   Node {node_id} raw offset = {pos:.3f}")
    return pos

def main():
    # Clear old messages
    while not (bus.recv(timeout=0) is None):
        pass

    config_data = {}

    for node_id in ALL_NODE_IDS:
        print(f"\n=== Calibrating motor {node_id} ===")
        # 1) Clear errors
        send_clear_errors(node_id)
        time.sleep(0.05)

        # 2) Enter CLOSED_LOOP_CONTROL
        send_set_axis_state(node_id, CLOSED_LOOP)
        print(f"   Waiting for node {node_id} to go CLOSED_LOOP...")
        if wait_for_closed_loop(node_id, 3.0):
            print(f"   ✅ Node {node_id} is CLOSED_LOOP.")
        else:
            print(f"   ⚠️ Node {node_id} did NOT enter CLOSED_LOOP. Continuing anyway...")

        # 3) Direction test: spin slowly, read position
        flip_sign = direction_test(node_id)

        # 4) If turning motor, prompt user to align zero => record offset
        if node_id in TURNING_NODE_IDS:
            offset_val = calibrate_turning_offset(node_id, flip_sign)
        else:
            offset_val = 0.0

        # 5) Return to IDLE
        send_set_axis_state(node_id, IDLE)
        print(f"   Node {node_id} set to IDLE.\n")

        config_data[node_id] = {
            "flip": flip_sign,
            "offset": offset_val
        }

    # Save to JSON
    with open(CONFIG_FILE, "w") as f:
        json.dump(config_data, f, indent=2)

    print(f"\n✅ Done calibrating all motors. Saved to '{CONFIG_FILE}'.\n")
    print("Use 'flip' and 'offset' in your main script as needed, e.g.:")
    print("  final_angle   = offset + flip*(desired_angle)    (for turning)")
    print("  final_vel     = flip*(desired_velocity)          (for rolling)\n")

if __name__ == "__main__":
    # For reading console input in the direction_test loop (non-blocking),
    # we need 'select' on some systems. We'll just ensure it’s imported:
    main()
