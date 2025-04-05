import can
import struct
import json
import time

TURNING_NODE_IDS = [0, 2, 4]
OFFSET_FILE = "turning_offsets.json"
ENCODER_ESTIMATES = 0x09
CLOSED_LOOP = 0x08
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd): return (node << 5) | cmd

# Flush old messages
while not (bus.recv(timeout=0) is None): pass

# Set motors to closed-loop so encoders are live
print("🔧 Sending all turning motors to CLOSED_LOOP_CONTROL...")
for node in TURNING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),
        is_extended_id=False
    ))

# Wait for them to confirm
ready_nodes = set()
while len(ready_nodes) < len(TURNING_NODE_IDS):
    msg = bus.recv(timeout=0.5)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node_id = msg.arbitration_id >> 5
        _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if state == CLOSED_LOOP and node_id in TURNING_NODE_IDS:
            if node_id not in ready_nodes:
                ready_nodes.add(node_id)
                print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")

# Prompt user to align each motor and read its encoder value
offsets = {}
for node in TURNING_NODE_IDS:
    input(f"\n➡️  Manually align turning motor {node} to its ZERO (forward) position and press ENTER...")

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
            print(f"📍 Node {node} calibrated at position: {pos:.3f} turns")
            break
    else:
        print(f"⚠️ No response from node {node}. Skipping...")

# Save to file
with open(OFFSET_FILE, 'w') as f:
    json.dump(offsets, f, indent=2)

print(f"\n💾 Calibration complete. Offsets saved to: {OFFSET_FILE}")
