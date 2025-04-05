#!/usr/bin/env python3

"""
Calibration script for ALL motors (turning + rolling). 
- Sets them all to IDLE (no torque).
- Lets you spin each motor by hand to check its 'forward' direction.
- If the encoder position increases when rotating forward, flip=+1. If it decreases, flip=-1.
- For turning motors, also prompt user to align to zero and record offset.
- For rolling motors, offset=0 (unused), but we still store a flip if needed.
- Saves everything to 'motor_config.json'.
"""

import can
import struct
import time
import json
import sys

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CONFIG_FILE = "motor_config.json"

# CAN function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
GET_ENCODER_ESTIMATES = 0x09

IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_arbid(node_id, func_code):
    return (node_id << 5) | func_code

def request_encoder_once(node_id, timeout=0.3):
    """
    Sends a single Get_Encoder_Estimates (0x09) request to `node_id`.
    Waits up to `timeout` seconds for a reply (arbitration_id = node<<5 | 0x19).
    Returns (pos, vel) or None if no reply.
    """
    req_id = to_arbid(node_id, GET_ENCODER_ESTIMATES)
    resp_id = to_arbid(node_id, GET_ENCODER_ESTIMATES | 0x10)
    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))
    
    t0 = time.time()
    while time.time() - t0 < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            return (pos, vel)
    return None

def idle_all_nodes():
    """Clear errors and set all nodes to IDLE so they are free to spin."""
    print("🔧 Setting all motors to IDLE (no torque)...")
    for node in ALL_NODE_IDS:
        # Clear errors
        bus.send(can.Message(arbitration_id=to_arbid(node, CLEAR_ERRORS), data=[], is_extended_id=False))
        time.sleep(0.05)
        # Set IDLE
        bus.send(can.Message(
            arbitration_id=to_arbid(node, SET_AXIS_STATE),
            data=struct.pack('<I', IDLE),
            is_extended_id=False
        ))
    # Optionally wait a moment
    time.sleep(0.5)

def direction_test(node_id):
    """
    Continuously requests encoder data from a motor in IDLE, 
    printing changes in position so user can see if it is increasing or decreasing 
    when rotating forward. Press Enter to stop test.
    Returns user-chosen flip (+1 or -1).
    """
    print(f"\n--- DIRECTION TEST for node {node_id} ---")
    print("Rotate the motor forward by hand. Observe position is going up or down.")
    print("Press ENTER (in the terminal) to finish the test...\n")

    prev_pos = None
    flip = +1  # default

    while True:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            # If user pressed Enter
            _ = sys.stdin.readline()
            break

        result = request_encoder_once(node_id)
        if result is not None:
            pos, vel = result
            if prev_pos is not None:
                diff = pos - prev_pos
                print(f" Node {node_id}: pos={pos:.3f}  (delta={diff:.3f})")
            else:
                print(f" Node {node_id}: pos={pos:.3f}")
            prev_pos = pos
        else:
            print(" No encoder reply. Firmware might not respond in IDLE. Retrying...")
        time.sleep(0.4)

    # Ask user if we need to flip
    ans = input("Did position INCREASE when rotating forward? (y/n) [y]: ")
    if ans.lower().startswith("n"):
        flip = -1
        print("→ Direction will be flipped.")
    else:
        flip = +1
        print("→ Direction will NOT be flipped.")
    return flip

def calibrate_offset_for_turning(node_id, flip_sign):
    """
    Prompt user to align turning motor to zero,
    then read encoder once. 
    offset = raw_position * flip_sign 
    (So that if we later do final_position = offset + flip_sign*(desired_angle), 
     the motor will be at zero for the physically aligned position.)
    """
    input(f"\nAlign TURNING motor {node_id} to ZERO, press ENTER to record offset...")

    result = request_encoder_once(node_id)
    if result is None:
        print("⚠️ No reply for offset. Using 0.0")
        return 0.0
    pos, vel = result
    # We'll store the offset in "raw" space, but incorporate flip here if you prefer:
    # If you want the physically aligned position to be 0, 
    # offset should be:  offset = (raw_position * flip_sign * -1).
    #
    # Then final_position = offset + flip_sign*(desired_angle).
    #
    # Another approach: store raw_offset = pos, and handle flipping in control script. 
    # Here, let's do the simpler approach: store offset so that 
    # final_position = offset + flip_sign*(0) = offset is 0 => offset = -flip_sign * pos
    offset_value = -flip_sign * pos
    print(f" Node {node_id}: raw pos={pos:.3f}, flip={flip_sign:.1f}, => offset={offset_value:.3f}")
    return offset_value

# We need select to detect Enter press in direction test
import select

def main():
    # Ensure old messages flushed
    while not (bus.recv(timeout=0) is None):
        pass
    
    # 1) Put all motors in IDLE
    idle_all_nodes()

    # 2) For each motor, run direction test
    config_data = {}
    for node in ALL_NODE_IDS:
        flip = direction_test(node)  # +1 or -1
        # If turning motor, we also calibrate offset
        if node in TURNING_NODE_IDS:
            off = calibrate_offset_for_turning(node, flip)
        else:
            off = 0.0  # rolling motors don't need offset
        config_data[node] = {"flip": flip, "offset": off}

    # 3) Save results
    with open(CONFIG_FILE, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"\n✅ Calibration complete. Saved flips & offsets to '{CONFIG_FILE}'.\n")

if __name__ == "__main__":
    main()
