import can
import struct
import json
import time

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

OFFSET_FILE = "turning_offsets.json"

# CANSimple command codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
CLOSED_LOOP = 0x08
IDLE = 0x01
ENCODER_ESTIMATES = 0x09
HEARTBEAT = 0x01

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd): return (node << 5) | cmd

# === Flush CAN RX buffer
while not (bus.recv(timeout=0) is None): pass

# === Step 1: Clear errors on all nodes
print("🧹 Clearing errors...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(arbitration_id=to_id(node, CLEAR_ERRORS), data=[], is_extended_id=False))
    time.sleep(0.05)

# === Step 2: Set all nodes to CLOSED_LOOP_CONTROL
print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),
        is_extended_id=False
    ))

# === Step 3: Wait for all to report state 8 (closed loop)
ready_nodes = set()
while ready_nodes != set(ALL_NODE_IDS):
    msg = bus.recv(timeout=1.0)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node = msg.arbitration_id >> 5
        _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if state == CLOSED_LOOP and node in ALL_NODE_IDS:
            if node not in ready_nodes:
                ready_nodes.add(node)
                print(f"✅ Node {node} entered CLOSED_LOOP_CONTROL")

# === Step 4: Prompt and capture zero offsets for turning motors
offsets = {}
for node in TURNING_NODE_IDS:
    input(f"\n➡️ Manually align TURNING motor {node} to ZERO position, then press ENTER...")

    # Request encoder estimate
    req_id = to_id(node, ENCODER_ESTIMATES)
    resp_id = to_id(node, ENCODER_ESTIMATES | 0x10)
    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))

    # Wait for response
    start = time.time()
    while time.time() - start < 1.0:
        msg = bus.recv(timeout=0.2)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            offsets[node] = pos
            print(f"📍 Node {node} zero offset: {pos:.3f} turns")
            break
    else:
        print(f"⚠️ No encoder response from node {node}")

# === Step 5: Set all nodes to IDLE
print("\n🛑 Setting all nodes to IDLE...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', IDLE),
        is_extended_id=False
    ))

# === Step 6: Save to JSON
with open(OFFSET_FILE, "w") as f:
    json.dump(offsets, f, indent=2)

print(f"\n💾 Calibration complete. Offsets saved to '{OFFSET_FILE}'")
