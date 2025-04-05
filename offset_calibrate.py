#!/usr/bin/env python3

"""
Calibrates flips + offsets for all motors (turning + rolling) while
ALL motors remain in CLOSED_LOOP_CONTROL.

Steps:
1) Clear errors on all nodes.
2) Put all nodes in CLOSED_LOOP_CONTROL.
3) For each motor in turn:
   a) Set velocity=TEST_VELOCITY on that motor, set velocity=0 on others.
   b) Repeatedly request encoder from that motor and print positions
      so you can confirm if physically it's rotating forward or backward.
   c) Prompt user: flip direction or not.
   d) If turning motor, prompt user to align to mechanical zero, read encoder once => offset.
4) After all motors are done, set them all to IDLE.
5) Save config (flip + offset for each node) to motor_config.json

Use 'flip' and 'offset' in your control script as:
  turning_final_angle = offset + flip * desired_angle
  rolling_final_velocity = flip * desired_velocity
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
SET_INPUT_VEL = 0x0D
GET_ENCODER_ESTIMATES = 0x09
HEARTBEAT = 0x01

# Axis states
CLOSED_LOOP = 8
IDLE = 1

# Velocity test params
TEST_VELOCITY = 0.2   # turns/s
TEST_DURATION = 5.0   # seconds to run test for each motor

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node_id, func_code):
    """Build ODrive CANSimple arbitration ID."""
    return (node_id << 5) | func_code

def clear_errors(node_id):
    """Sends Clear_Errors command."""
    bus.send(can.Message(
        arbitration_id=to_id(node_id, CLEAR_ERRORS),
        data=[],
        is_extended_id=False
    ))

def set_axis_state(node_id, state_int):
    """Sends Set_Axis_State command."""
    msg = can.Message(
        arbitration_id=to_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', state_int),
        is_extended_id=False
    )
    bus.send(msg)

def set_input_vel(node_id, velocity, torque_ff=0.0):
    """Sends Set_Input_Vel command."""
    msg = can.Message(
        arbitration_id=to_id(node_id, SET_INPUT_VEL),
        data=struct.pack('<ff', velocity, torque_ff),
        is_extended_id=False
    )
    bus.send(msg)

def request_encoder_once(node_id, timeout=0.3):
    """
    Sends one Get_Encoder_Estimates (0x09) request.
    Waits up to `timeout` for response (arbitration_id = node<<5 | 0x19).
    Returns (pos, vel) or None if no reply.
    """
    req_id = to_id(node_id, GET_ENCODER_ESTIMATES)
    resp_id = to_id(node_id, GET_ENCODER_ESTIMATES | 0x10)

    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))

    start = time.time()
    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            return (pos, vel)
    return None

def wait_for_closed_loop(node_id, timeout=2.0):
    """Wait until heartbeat shows AxisState=8 (CLOSED_LOOP)."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = bus.recv(timeout=0.2)
        if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
            n_id = msg.arbitration_id >> 5
            if n_id == node_id:
                error, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if state == CLOSED_LOOP:
                    return True
    return False

def main():
    print("=== BEGIN CALIBRATION (All nodes in same state) ===")

    # 1) Clear old messages from bus
    while not (bus.recv(timeout=0) is None):
        pass

    # 2) Clear errors for all nodes
    print("🧹 Clearing errors on all nodes...")
    for node in ALL_NODE_IDS:
        clear_errors(node)
        time.sleep(0.05)

    # 3) Put ALL nodes into CLOSED_LOOP_CONTROL
    print("🔄 Putting all nodes in CLOSED_LOOP_CONTROL...")
    for node in ALL_NODE_IDS:
        set_axis_state(node, CLOSED_LOOP)

    # 4) Wait for them all to confirm
    not_ready = set(ALL_NODE_IDS)
    deadline = time.time() + 3
    while time.time() < deadline and not_ready:
        msg = bus.recv(timeout=0.2)
        if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
            n_id = msg.arbitration_id >> 5
            if n_id in not_ready:
                _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if state == CLOSED_LOOP:
                    not_ready.remove(n_id)
                    print(f"✅ Node {n_id} in CLOSED_LOOP_CONTROL")
    if not_ready:
        print(f"⚠️ The following nodes never confirmed CLOSED_LOOP: {not_ready}")

    # 5) Zero velocities initially
    print("\n⚙️ Setting velocity=0 on all nodes so none move initially.")
    for node in ALL_NODE_IDS:
        set_input_vel(node, 0.0)

    # 6) For each node: direction test + offset
    config_data = {}
    for node in ALL_NODE_IDS:
        print(f"\n=== Node {node} direction test ===")

        # a) Set test velocity on this node, 0 on others
        print(f"   Setting node {node} velocity={TEST_VELOCITY}, others=0")
        for n2 in ALL_NODE_IDS:
            if n2 == node:
                set_input_vel(n2, TEST_VELOCITY)
            else:
                set_input_vel(n2, 0.0)

        # b) Keep requesting encoder from this node for ~TEST_DURATION
        t0 = time.time()
        prev_pos = None
        while time.time() - t0 < TEST_DURATION:
            res = request_encoder_once(node)
            if res is not None:
                pos, vel = res
                if prev_pos is not None:
                    dpos = pos - prev_pos
                    print(f"   Node {node}: pos={pos:.3f}, (delta={dpos:.3f})")
                else:
                    print(f"   Node {node}: pos={pos:.3f}")
                prev_pos = pos
            else:
                print("   No encoder response, continuing...")

            time.sleep(0.4)

        # Stop movement for this node
        set_input_vel(node, 0.0)

        # c) Ask user if direction is correct
        ans = input(f"\n   Did node {node} rotate forward physically? (y/n) [y]: ")
        if ans.strip().lower().startswith('n'):
            flip = -1.0
            print("   → We'll flip direction for this motor.")
        else:
            flip = 1.0
            print("   → No flip needed.")

        # d) If turning motor, prompt user to align zero => read offset
        if node in TURNING_NODE_IDS:
            input(f"\n   Align TURNING motor {node} to mechanical ZERO, press ENTER to record offset...")
            res = request_encoder_once(node)
            if res:
                off, _ = res
                print(f"   Node {node} raw offset = {off:.3f}")
            else:
                off = 0.0
                print("   ⚠️ No encoder response => offset=0.0 by default.")
        else:
            off = 0.0  # rolling motors need no offset

        config_data[node] = {
            "flip": flip,
            "offset": off
        }

    # 7) Finally, set all nodes to IDLE
    print("\n🛑 Setting all nodes to IDLE (unpowered).")
    for node in ALL_NODE_IDS:
        set_axis_state(node, IDLE)
    print("   Done.")

    # 8) Save config
    with open(CONFIG_FILE, "w") as f:
        json.dump(config_data, f, indent=2)
    print(f"\n💾 Saved calibration to '{CONFIG_FILE}'.\n")
    print("Use them in your control script, e.g.:")
    print("  turning_final_angle = offset + flip * desired_angle")
    print("  rolling_final_vel   = flip * desired_vel\n")


if __name__ == "__main__":
    main()
