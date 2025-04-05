#!/usr/bin/env python3

import can
import struct
import time
import signal
import sys
import json
import os

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

OFFSET_FILE = "turning_offsets.json"

TARGET_ANGLE = 0.5       # turns for turning motors
TARGET_VELOCITY = 1.0    # turns/s for rolling motors

# CAN function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
ENCODER_ESTIMATES = 0x09

CLOSED_LOOP = 8
IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd):
    return (node << 5) | cmd

def shutdown_all():
    print("\n🛑 Ctrl+C detected — setting all nodes to IDLE.")
    for node in ALL_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_id(node, SET_AXIS_STATE),
            data=struct.pack('<I', IDLE),
            is_extended_id=False
        ))
        print(f"🛑 Node {node} set to IDLE.")
    sys.exit(0)

signal.signal(signal.SIGINT, lambda s, f: shutdown_all())

# === 1) Load offset & flip data
if not os.path.exists(OFFSET_FILE):
    print(f"❌ Offset file '{OFFSET_FILE}' not found. Please run calibration first.")
    sys.exit(1)

with open(OFFSET_FILE, "r") as f:
    offset_data = json.load(f)
    # Example: offset_data = {
    #   "0": { "offset": -0.123, "sign": -1.0 },
    #   "2": { "offset":  0.456, "sign":  1.0 },
    #   "4": { "offset":  0.000, "sign":  1.0 }
    # }

# === 2) Flush old CAN traffic
while not (bus.recv(timeout=0) is None):
    pass

# === 3) Clear errors & set CLOSED_LOOP_CONTROL
print("🧹 Clearing errors...")
for node_id in ALL_NODE_IDS:
    bus.send(can.Message(arbitration_id=to_id(node_id, CLEAR_ERRORS), data=[], is_extended_id=False))
    time.sleep(0.05)

print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL...")
for node_id in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),
        is_extended_id=False
    ))

ready_nodes = set()
expected_nodes = set(ALL_NODE_IDS)
timeout = time.time() + 3
while time.time() < timeout and ready_nodes != expected_nodes:
    msg = bus.recv(timeout=0.2)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node_id = msg.arbitration_id >> 5
        if node_id in expected_nodes:
            _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if state == CLOSED_LOOP:
                ready_nodes.add(node_id)
                print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")

# === 4) Send turning motors to angle (offset + sign * TARGET_ANGLE)
print("\n🎯 Sending turning angles...")
for node_id in TURNING_NODE_IDS:
    node_str = str(node_id)  # JSON keys are strings
    if node_str in offset_data:
        off = offset_data[node_str].get("offset", 0.0)
        sgn = offset_data[node_str].get("sign", 1.0)
        final_pos = off + sgn * TARGET_ANGLE
        print(f"   Node {node_id}: offset={off:.3f}, sign={sgn:.0f}, target={TARGET_ANGLE:.3f} → final={final_pos:.3f}")
    else:
        # Default if not found
        final_pos = TARGET_ANGLE
        print(f"   Node {node_id} missing offset data. Using no offset/flip.")

    bus.send(can.Message(
        arbitration_id=to_id(node_id, SET_INPUT_POS),
        data=struct.pack('<fhh', final_pos, 0, 0),
        is_extended_id=False
    ))

# === 5) Send rolling motors to velocity
print("\n🚀 Sending rolling velocities...")
for node_id in ROLLING_NODE_IDS:
    print(f"   Node {node_id}: velocity={TARGET_VELOCITY:.3f}")
    bus.send(can.Message(
        arbitration_id=to_id(node_id, SET_INPUT_VEL),
        data=struct.pack('<ff', TARGET_VELOCITY, 0.0),
        is_extended_id=False
    ))

# === 6) Listen for encoder broadcasts (passive)
print("\n📡 Listening for encoder feedback (Ctrl+C to stop)...")
while True:
    msg = bus.recv()
    if msg and (msg.arbitration_id & 0x1F) == ENCODER_ESTIMATES:
        node_id = msg.arbitration_id >> 5
        pos, vel = struct.unpack('<ff', msg.data)
        print(f"📍 Node {node_id} | Pos: {pos:.3f} turns, Vel: {vel:.3f}")
