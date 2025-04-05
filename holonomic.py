#!/usr/bin/env python3

"""
Control script that:
1) Loads 'motor_config.json' for each node's flip + offset.
2) Clears errors on all nodes.
3) Sets all nodes to CLOSED_LOOP_CONTROL.
4) For turning motors, sends position commands = offset + flip*(TARGET_ANGLE).
5) For rolling motors, sends velocity commands = flip*(TARGET_VELOCITY).
6) Prints encoder feedback in a loop until Ctrl+C, then sets nodes to IDLE.

Requires:
- `motor_config.json` with structure like:
  {
    "0": { "flip": 1.0,  "offset":  2.34 },
    "1": { "flip": -1.0, "offset":  0.0 },
    "2": { "flip": 1.0,  "offset": -0.56 },
    ...
  }
"""

import can
import struct
import json
import time
import sys
import signal
import os

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

MOTOR_CONFIG_FILE = "motor_config.json"

# Desired motion
TARGET_ANGLE = 0.5       # turns for turning motors
TARGET_VELOCITY = 1.0    # turns/s for rolling motors

# CANSimple function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
ENCODER_ESTIMATES = 0x09

CLOSED_LOOP = 8
IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_arbid(node_id, func_code):
    return (node_id << 5) | func_code

def shutdown_all():
    """Set all motors to IDLE and exit."""
    print("\n🛑 Ctrl+C detected — setting all nodes to IDLE.")
    for node in ALL_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_arbid(node, SET_AXIS_STATE),
            data=struct.pack('<I', IDLE),
            is_extended_id=False
        ))
        print(f"   Node {node} set to IDLE.")
    sys.exit(0)

signal.signal(signal.SIGINT, lambda s, f: shutdown_all())

def main():
    # 1) Load motor flips + offsets from JSON
    if not os.path.exists(MOTOR_CONFIG_FILE):
        print(f"❌ '{MOTOR_CONFIG_FILE}' not found. Please run calibration first.")
        sys.exit(1)

    with open(MOTOR_CONFIG_FILE, "r") as f:
        config_data = json.load(f)
        # Example:
        # {
        #   "0": { "flip": 1.0, "offset": 2.31 },
        #   "1": { "flip": -1.0, "offset": 0.0 },
        #   ...
        # }

    # 2) Flush old CAN traffic
    while not (bus.recv(timeout=0) is None):
        pass

    # 3) Clear errors on all nodes
    print("🧹 Clearing errors on all nodes...")
    for node_id in ALL_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_arbid(node_id, CLEAR_ERRORS),
            data=[],
            is_extended_id=False
        ))
        time.sleep(0.05)

    # 4) Put all nodes in CLOSED_LOOP_CONTROL
    print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL...")
    for node_id in ALL_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_arbid(node_id, SET_AXIS_STATE),
            data=struct.pack('<I', CLOSED_LOOP),
            is_extended_id=False
        ))

    # Wait for each node to confirm
    ready_nodes = set()
    expected_nodes = set(ALL_NODE_IDS)
    timeout = time.time() + 3
    while time.time() < timeout and ready_nodes != expected_nodes:
        msg = bus.recv(timeout=0.2)
        if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
            n_id = msg.arbitration_id >> 5
            if n_id in expected_nodes:
                axis_error, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
                if axis_state == CLOSED_LOOP:
                    ready_nodes.add(n_id)
                    print(f"   Node {n_id} in CLOSED_LOOP_CONTROL")

    not_ready = expected_nodes - ready_nodes
    if not_ready:
        print(f"⚠️ Some nodes did not confirm CLOSED_LOOP: {not_ready}")

    # 5) Send commands (turning motors get position, rolling get velocity)
    print("\n🎯 Sending turning angles (with flip + offset)...")
    for node_id in TURNING_NODE_IDS:
        # load flip + offset, default if missing
        node_str = str(node_id)
        flip = 1.0
        offset = 0.0
        if node_str in config_data:
            flip = config_data[node_str].get("flip", 1.0)
            offset = config_data[node_str].get("offset", 0.0)
        final_pos = offset + flip * TARGET_ANGLE

        print(f"   Node {node_id}: flip={flip}, offset={offset:.3f}, target_angle={TARGET_ANGLE:.3f} => final_pos={final_pos:.3f}")
        bus.send(can.Message(
            arbitration_id=to_arbid(node_id, SET_INPUT_POS),
            data=struct.pack('<fhh', final_pos, 0, 0),
            is_extended_id=False
        ))

    print("\n🚀 Sending rolling velocities (with flip)...")
    for node_id in ROLLING_NODE_IDS:
        node_str = str(node_id)
        flip = 1.0
        offset = 0.0  # not used for rolling
        if node_str in config_data:
            flip = config_data[node_str].get("flip", 1.0)
        final_vel = flip * TARGET_VELOCITY
        print(f"   Node {node_id}: flip={flip}, velocity={TARGET_VELOCITY:.3f} => final_vel={final_vel:.3f}")
        bus.send(can.Message(
            arbitration_id=to_arbid(node_id, SET_INPUT_VEL),
            data=struct.pack('<ff', final_vel, 0.0),
            is_extended_id=False
        ))

    # 6) Listen for encoder broadcasts or respond to requests
    #    If your firmware automatically broadcasts at (node<<5|0x09),
    #    or if you want to request it, that's up to you.
    print("\n📡 Listening for encoder feedback (Ctrl+C to stop)...")
    while True:
        msg = bus.recv()
        if msg and (msg.arbitration_id & 0x1F) == ENCODER_ESTIMATES:
            n_id = msg.arbitration_id >> 5
            pos, vel = struct.unpack('<ff', msg.data)
            print(f"📍 Node {n_id} | Pos: {pos:.3f}, Vel: {vel:.3f}")

if __name__ == "__main__":
    main()
