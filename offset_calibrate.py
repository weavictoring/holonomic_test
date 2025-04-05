import can
import struct
import json
import time

TURNING_NODE_IDS = [0, 2, 4]
OFFSET_FILE = "turning_offsets.json"

# CANSimple command codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
IDLE = 0x01
ENCODER_ESTIMATES = 0x09

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd): return (node << 5) | cmd

# === Flush CAN RX buffer
while not (bus.recv(timeout=0) is None): pass

# === Clear errors and set IDLE
print("🔧 Setting motors to IDLE (free to move)...")
for node in TURNING_NODE_IDS:
    bus.send(can.Message(arbitration_id=to_id(node, CLEAR_ERRORS), data=[], is_extended_id=False))
    time.sleep(0.05)
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', IDLE),
        is_extended_id=False
    ))
    print(f"🛠️ Node {node} set to IDLE")

# === Prompt and record offsets
offsets = {}
for node in TURNING_NODE_IDS:
    input(f"\n➡️ Manually rotate turning motor {node} to its ZERO (forward) position, then press ENTER...")

    # Request encoder estimate
    req_id = to_id(node, ENCODER_ESTIMATES)
    resp_id = to_id(node, ENCODER_ESTIMATES | 0x10)

    bus.send(can.Message(arbitration_id=req_id, data=[], is_extended_id=False))

    start = time.time()
    while time.time() - start < 1.0:
        msg = bus.recv(timeout=0.2)
        if msg and msg.arbitration_id == resp_id:
            pos, vel = struct.unpack('<ff', msg.data)
            offsets[node] = pos
            print(f"📍 Node {node} zero offset recorded: {pos:.3f} turns")
            break
    else:
        print(f"⚠️ No encoder response from node {node}. Skipping...")

# === Save to JSON
with open(OFFSET_FILE, "w") as f:
    json.dump(offsets, f, indent=2)

print(f"\n💾 Offsets saved to {OFFSET_FILE}")
