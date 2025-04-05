#!/usr/bin/env python3

"""
Calibration script that:
1. Clears errors on all nodes,
2. Sets all nodes into CLOSED_LOOP_CONTROL so they broadcast encoder data,
3. Prompts user to manually align each *turning* motor to its 0° position,
4. Reads its broadcasted encoder position,
5. Applies a direction flip if configured for that node,
6. Stores offset and flip sign in JSON for use by the main control script.

This uses passive listening for 'ENCODER_ESTIMATES' (0x09) just like your working minimal example.
"""

import can
import struct
import json
import time

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

# If you want to flip any motor's direction, set True. If not, False.
# For instance, if node 0 is physically reversed, set True.
TURNING_FLIPS = {
    0: False,
    2: False,
    4: False
}

OFFSET_FILE = "turning_offsets.json"

# CAN function codes
CLEAR_ERRORS = 0x08
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
ENCODER_ESTIMATES = 0x09

CLOSED_LOOP = 8
IDLE = 1

bus = can.interface.Bus("can0", bustype="socketcan")

def to_id(node, cmd):
    """Helper to build arbitration ID."""
    return (node << 5) | cmd

# 1) Flush old messages
while not (bus.recv(timeout=0) is None):
    pass

# 2) Clear errors on all nodes
print("🧹 Clearing errors on all nodes...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, CLEAR_ERRORS),
        data=[],
        is_extended_id=False
    ))
    time.sleep(0.05)

# 3) Set all nodes to CLOSED_LOOP_CONTROL (so they broadcast encoder data)
print("🔄 Setting all nodes to CLOSED_LOOP_CONTROL, so we can read encoder broadcasts...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', CLOSED_LOOP),  # 8
        is_extended_id=False
    ))

# Wait for each node's heartbeat with state=8
ready_nodes = set(ALL_NODE_IDS)
timeout = time.time() + 3
already_ok = set()
while time.time() < timeout and ready_nodes:
    msg = bus.recv(timeout=0.2)
    if msg and (msg.arbitration_id & 0x1F) == HEARTBEAT:
        node_id = msg.arbitration_id >> 5
        if node_id in already_ok:
            continue
        axis_error, axis_state, _, _ = struct.unpack('<IBBB', msg.data[:7])
        if axis_state == CLOSED_LOOP and node_id in ready_nodes:
            print(f"✅ Node {node_id} in CLOSED_LOOP_CONTROL")
            ready_nodes.remove(node_id)
            already_ok.add(node_id)

if ready_nodes:
    print(f"⚠️ Some nodes did not confirm CLOSED_LOOP_CONTROL: {ready_nodes}")

# 4) For each turning motor: prompt user to align, then read broadcasted encoder
results = {}
for node in TURNING_NODE_IDS:
    input(f"\n➡️ Manually align TURNING motor {node} to ZERO, press ENTER...")

    print("   Waiting for next broadcast from node {node}... (Ctrl+C to cancel)")
    pos_captured = None

    while True:
        msg = bus.recv(timeout=1.0)
        if not msg:
            print("   ⚠️ Timeout waiting for broadcast. Retrying...")
            continue

        # Check if it's an encoder broadcast for *this* node
        if (msg.arbitration_id & 0x1F) == ENCODER_ESTIMATES:
            n_id = msg.arbitration_id >> 5
            if n_id == node:
                pos, vel = struct.unpack('<ff', msg.data)
                print(f"   Node {node} raw offset: {pos:.3f} turns")
                pos_captured = pos
                break

    # Flip direction if needed
    flip_bool = TURNING_FLIPS[node]
    flip_sign = -1.0 if flip_bool else 1.0
    adjusted_pos = flip_sign * pos_captured
    print(f"   Direction flip = {flip_bool}, final offset = {adjusted_pos:.3f}")

    # Store offset + sign in the results
    results[node] = {
        "offset": adjusted_pos,
        "sign": flip_sign
    }

# 5) Set all nodes back to IDLE
print("\n🛑 Setting all nodes to IDLE (no torque)...")
for node in ALL_NODE_IDS:
    bus.send(can.Message(
        arbitration_id=to_id(node, SET_AXIS_STATE),
        data=struct.pack('<I', IDLE),  # 1
        is_extended_id=False
    ))
print("   Done!")

# 6) Save offset & sign to JSON
with open(OFFSET_FILE, "w") as f:
    json.dump(results, f, indent=2)

print(f"\n💾 Calibration finished. Offsets + flips saved to '{OFFSET_FILE}'.")
