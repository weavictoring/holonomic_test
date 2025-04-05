#!/usr/bin/env python3

"""
Calibration script for ODrive motors that cannot read encoders in IDLE.
Strategy:
1) Prompt user to manually align ALL motors at mechanical zero while unpowered.
2) Then we power them all on (CLOSED_LOOP_CONTROL) to read their encoder positions exactly once.
3) For turning motors, store that reading as the offset.
   For rolling motors, offset=0 (they don't need an angle zero).
4) Prompt the user if each motor's direction should be flipped or not.
5) Save it all to motor_config.json as {node_id: {"offset":..., "flip":...}, ...}.
"""

import can
import struct
import json
import time
import sys

# Define your node groups
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CONFIG_FILE = "motor_config.json"

# CAN protocol function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
GET_ENCODER_ESTIMATES = 0x09
HEARTBEAT = 0x01

# Axis states
CLOSED_LOOP = 8
IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_arbid(node_id, func):
    """Helper to build arbitration ID for ODrive CANSimple."""
    return (node_id << 5) | func

def send_set_axis_state(node_id, state):
    """Sends Set_Axis_State command to 'state' (int)."""
    msg = can.Message(
        arbitration_id=to_arbid(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False
    )
    bus.send(msg)

def request_encoder_once(node_id, timeout=0.3):
    """
    Sends one Get_Encoder_Estimates (0x09) request to 'node_id'.
    Waits up to 'timeout' seconds for the reply (arbitration_id = (node<<5)|(0x09|0x10) ).
    Returns (pos, vel) or None if no response.
    """
    req_id = to_arbid(node_id, GET_ENCODER_ESTIMATES)
    resp_id = to_arbid(node_id, GET_ENCODER_ESTIMATES | 0x10)

    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))

    start = time.time()
    while time.time() - start < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            return (pos, vel)
    return None

def main():
    # 1) Prompt user: physically align all motors to 0 while unpowered
    input(
        "\n➡️  Please power OFF or ensure no torque is applied, "
        "and manually align ALL motors (turning + rolling) to zero.\n"
        "Press ENTER once everything is aligned..."
    )

    # 2) Clear any old messages from the bus
    while not (bus.recv(timeout=0) is None):
        pass

    # 3) Clear errors, then set all nodes to CLOSED_LOOP_CONTROL
    print("🧹 Clearing errors on all nodes...")
    for node in ALL_NODE_IDS:
        bus.send(can.Message(arbitration_id=to_arbid(node, CLEAR_ERRORS), data=[], is_extended_id=False))
        time.sleep(0.05)

    print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL so we can read encoders...")
    for node in ALL_NODE_IDS:
        send_set_axis_state(node, CLOSED_LOOP)

    # Wait for each node's heartbeat => state=8
    ready_nodes = set()
    expected_nodes = set(ALL_NODE_IDS)
    timeout_t = time.time() + 3
    while time.time() < timeout_t and ready_nodes != expected_nodes:
        msg = bus.recv(timeout=0.2)
        if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
            node_id = msg.arbitration_id >> 5
            if node_id in expected_nodes:
                axis_error, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if axis_state == CLOSED_LOOP:
                    ready_nodes.add(node_id)
                    print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")

    # 4) For each node: read encoder once. 
    #    If turning, store offset = that position. If rolling, offset=0. 
    #    Ask user about flipping direction.
    config_data = {}
    for node_id in ALL_NODE_IDS:
        if node_id in TURNING_NODE_IDS:
            print(f"\n🎯 Reading ZERO offset for TURNING motor {node_id}...")
            resp = request_encoder_once(node_id)
            if resp:
                pos, vel = resp
                offset_val = pos
                print(f"   Node {node_id} raw offset = {offset_val:.3f}")
            else:
                # No response => default to 0 offset
                offset_val = 0.0
                print(f"   ⚠️ No encoder response, offset=0.0")
        else:
            # rolling motor => no offset
            offset_val = 0.0

        # Ask user if direction is reversed
        ans = input(f"   Flip direction for node {node_id}? (y/n) [n]: ")
        if ans.lower().startswith('y'):
            flip_val = -1.0
            print("   → Flipping direction.")
        else:
            flip_val = 1.0
            print("   → No flip.")

        config_data[node_id] = {
            "offset": offset_val,
            "flip": flip_val
        }

    # 5) Optional: set all nodes back to IDLE
    print("\n🛑 Setting all nodes to IDLE (unpowered)...")
    for node_id in ALL_NODE_IDS:
        send_set_axis_state(node_id, IDLE)

    # 6) Save to JSON
    with open(CONFIG_FILE, "w") as f:
        json.dump(config_data, f, indent=2)

    print(
        f"\n✅ Calibration complete. Offsets + flips saved to '{CONFIG_FILE}'.\n"
        "Use them in your control script as:\n"
        "    final_angle = offset + flip * desired_angle   (for turning)\n"
        "    final_vel   = flip * desired_vel             (for rolling)\n"
    )

if __name__ == "__main__":
    main()
