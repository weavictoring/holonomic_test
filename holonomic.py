import can
import struct
import time
import signal
import sys

# Your motor configuration
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]

TARGET_ANGLE = 0.5       # turns for position control
TARGET_VELOCITY = 1.0    # turns/sec for velocity control

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd):
    return (node << 5) | cmd

# === Safe shutdown on Ctrl+C ===
def shutdown_all():
    print("\n🛑 Ctrl+C detected — stopping all motors.")
    for node in TURNING_NODE_IDS + ROLLING_NODE_IDS:
        bus.send(can.Message(
            arbitration_id=to_id(node, 0x07),  # Set_Axis_State
            data=struct.pack('<I', 1),  # 1 = IDLE
            is_extended_id=False
        ))
        print(f"🛑 Node {node} set to IDLE.")
    sys.exit(0)

signal.signal(signal.SIGINT, lambda s, f: shutdown_all())

# === Main sequence ===
# Flush old CAN traffic
while not (bus.recv(timeout=0) is None): pass

# Put all nodes into CLOSED_LOOP_CONTROL
print("🔄 Entering closed-loop control...")
for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node_id, 0x07),
        data=struct.pack('<I', 8),  # 8 = CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

# Wait for all nodes to report CLOSED_LOOP_CONTROL
ready_nodes = set()
while len(ready_nodes) < len(TURNING_NODE_IDS + ROLLING_NODE_IDS):
    msg = bus.recv()
    if msg and (msg.arbitration_id & 0x1F) == 0x01:  # Heartbeat
        node_id = msg.arbitration_id >> 5
        error, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if state == 8 and node_id not in ready_nodes:
            ready_nodes.add(node_id)
            print(f"✅ Node {node_id} is in CLOSED_LOOP_CONTROL")

# Send position to turning motors
print("\n🎯 Sending turning motor angles...")
for node in TURNING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, 0x0C),  # Set_Input_Pos
        data=struct.pack('<fhh', TARGET_ANGLE, 0, 0),
        is_extended_id=False
    ))
    print(f"🎯 Turning node {node} → {TARGET_ANGLE} turns")

# Send velocity to rolling motors
print("\n🚀 Sending rolling motor velocities...")
for node in ROLLING_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, 0x0D),  # Set_Input_Vel
        data=struct.pack('<ff', TARGET_VELOCITY, 0.0),
        is_extended_id=False
    ))
    print(f"🚀 Rolling node {node} → {TARGET_VELOCITY} turns/s")

# Listen for encoder estimates (printed if received)
print("\n📡 Listening for encoder feedback...")
while True:
    msg = bus.recv()
    if msg and (msg.arbitration_id & 0x1F) == 0x09:  # Encoder estimates
        node_id = msg.arbitration_id >> 5
        pos, vel = struct.unpack('<ff', msg.data)
        print(f"📍 Node {node_id} | Pos: {pos:.3f}, Vel: {vel:.3f}")
