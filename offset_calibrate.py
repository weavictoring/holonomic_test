import can
import struct
import json
import time

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

OFFSET_FILE = "turning_offsets.json"

# CANSimple codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
ENCODER_ESTIMATES = 0x09

CLOSED_LOOP = 8
IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd):
    """Helper to construct arbitration ID"""
    return (node << 5) | cmd

# 1) Flush old messages
while not (bus.recv(timeout=0) is None):
    pass

# 2) Clear errors on all nodes
print("🧹 Clearing errors on all nodes...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(arbitration_id=to_id(node, CLEAR_ERRORS), data=[], is_extended_id=False))
    time.sleep(0.05)

# 3) Put all nodes into CLOSED_LOOP_CONTROL
print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL (so they broadcast encoders)...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),
        is_extended_id=False
    ))

# Wait for each node's heartbeat with state=8
ready_nodes = set()
expected_nodes = set(ALL_NODE_IDS)
timeout = time.time() + 2
while time.time() < timeout and ready_nodes != expected_nodes:
    msg = bus.recv(timeout=0.2)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node_id = msg.arbitration_id >> 5
        if node_id in expected_nodes:
            axis_error, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if axis_state == CLOSED_LOOP:
                ready_nodes.add(node_id)
                print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")

# 4) Calibrate turning motors (passive read)
offsets = {}
for node in TURNING_NODE_IDS:
    input(f"\n➡️ Manually align TURNING motor {node} to its ZERO, then press ENTER...")

    print("   Waiting for next broadcast from this node...")
    while True:
        msg = bus.recv(timeout=1.0)
        if not msg:
            print(f"   ⚠️ Timeout waiting for node {node}. Retrying...")
            continue
        
        # Check if this is a broadcasted encoder message for *this* node
        if (msg.arbitration_id & 0x1F) == ENCODER_ESTIMATES:
            this_node = msg.arbitration_id >> 5
            if this_node == node:
                pos, vel = struct.unpack('<ff', msg.data)
                offsets[node] = pos
                print(f"   📍 Node {node} zero offset: {pos:.3f} turns")
                break

# 5) Put all nodes back to IDLE (so they don’t hold torque)
print("\n🛑 Setting all nodes to IDLE...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', IDLE),
        is_extended_id=False
    ))

# 6) Save offsets to JSON
with open(OFFSET_FILE, "w") as f:
    json.dump(offsets, f, indent=2)

print(f"\n💾 Calibration complete — offsets saved to '{OFFSET_FILE}'.")
print("Done!")
