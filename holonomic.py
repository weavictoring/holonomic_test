import can
import struct
import time
import signal
import sys
import json
import os

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
OFFSET_FILE = "turning_offsets.json"

TARGET_ANGLE = 0.5       # turns
TARGET_VELOCITY = 1.0    # turns/s

# CANSimple command codes
SET_AXIS_STATE = 0x07
CLEAR_ERRORS = 0x08
HEARTBEAT = 0x01
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
ENCODER_ESTIMATES = 0x09
CLOSED_LOOP = 0x08
IDLE = 0x01

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd):
    return (node << 5) | cmd

def shutdown_all():
    print("\n🛑 Ctrl+C detected — setting all nodes to IDLE.")
    for node in TURNING_NODE_IDS + ROLLING_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_id(node, SET_AXIS_STATE),
            data=struct.pack('<I', IDLE),
            is_extended_id=False
        ))
        print(f"🛑 Node {node} set to IDLE.")
    sys.exit(0)

signal.signal(signal.SIGINT, lambda s, f: shutdown_all())

# === Load offsets ===
if not os.path.exists(OFFSET_FILE):
    print(f"❌ Offset file '{OFFSET_FILE}' not found. Please run calibration first.")
    sys.exit(1)

with open(OFFSET_FILE, 'r') as f:
    offsets = json.load(f)

# === Flush CAN bus
while not (bus.recv(timeout=0) is None): pass

# === Clear errors
print("🧹 Clearing errors...")
for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node_id, CLEAR_ERRORS),
        data=[],
        is_extended_id=False
    ))
    time.sleep(0.05)

# === Set to CLOSED_LOOP_CONTROL
print("🔄 Sending CLOSED_LOOP_CONTROL...")
for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),
        is_extended_id=False
    ))

# === Wait for each node to enter CLOSED_LOOP_CONTROL
ready_nodes = set()
expected_nodes = set(TURNING_NODE_IDS + ROLLING_NODE_IDS)
while ready_nodes != expected_nodes:
    msg = bus.recv(timeout=1.0)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node_id = msg.arbitration_id >> 5
        _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if state == CLOSED_LOOP and node_id in expected_nodes:
            if node_id not in ready_nodes:
                ready_nodes.add(node_id)
                print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")

# === Send turning angles (with offsets)
print("\n🎯 Sending turning angles...")
for node in TURNING_NODE_IDS:
    offset = float(offsets.get(str(node), 0.0))
    target_pos = TARGET_ANGLE + offset
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_INPUT_POS),
        data=struct.pack('<fhh', target_pos, 0, 0),
        is_extended_id=False
    ))
    print(f"🎯 Turning node {node} → raw: {TARGET_ANGLE:.3f}, offset: {offset:.3f}, final: {target_pos:.3f}")

# === Send rolling velocities
print("\n🚀 Sending rolling velocities...")
for node in ROLLING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_INPUT_VEL),
        data=struct.pack('<ff', TARGET_VELOCITY, 0.0),
        is_extended_id=False
    ))
    print(f"🚀 Rolling node {node} → {TARGET_VELOCITY:.3f} turns/s")

# === Listen for encoder feedback
print("\n📡 Listening for encoder feedback (Ctrl+C to stop)...")
while True:
    msg = bus.recv()
    if msg and (msg.arbitration_id & 0x1F) == ENCODER_ESTIMATES:
        node_id = msg.arbitration_id >> 5
        pos, vel = struct.unpack('<ff', msg.data)
        print(f"📍 Node {node_id} | Pos: {pos:.3f} turns, Vel: {vel:.3f} turns/s")
