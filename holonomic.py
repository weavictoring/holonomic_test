import can
import struct
import time

NODE_IDS = list(range(6))  # Node IDs 0–5
VEL_SETPOINT = 1.0  # turns/s

# CAN function codes
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
SET_INPUT_VEL = 0x0D
GET_ENCODER_ESTIMATES = 0x09

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush any old CAN messages
while not (bus.recv(timeout=0) is None): pass

def to_arbitration_id(node_id, func_code):
    return (node_id << 5) | func_code

def enter_closed_loop(node_id):
    print(f"🔄 Setting node {node_id} to CLOSED_LOOP_CONTROL...")
    bus.send(can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', 8),  # 8 = CLOSED_LOOP_CONTROL
        is_extended_id=False
    ))

    # Wait for heartbeat with state 8
    timeout = time.time() + 2
    while time.time() < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == to_arbitration_id(node_id, HEARTBEAT):
            _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if state == 8:
                print(f"✅ Node {node_id} is now in CLOSED_LOOP_CONTROL.")
                return True
    print(f"❌ Timeout waiting for node {node_id} to enter closed-loop.")
    return False

def send_velocity(node_id, velocity):
    print(f"🚀 Sending velocity {velocity:.2f} turns/s to node {node_id}")
    bus.send(can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_INPUT_VEL),
        data=struct.pack('<ff', velocity, 0.0),  # velocity, torque_ff
        is_extended_id=False
    ))

def request_encoder_estimate(node_id):
    request_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES)
    response_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES | 0x10)

    bus.send(can.Message(
        arbitration_id=request_id,
        data=[],
        is_extended_id=False
    ))

    timeout = time.time() + 0.5
    while time.time() < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == response_id:
            pos, vel = struct.unpack('<ff', msg.data)
            print(f"📍 Node {node_id}: Pos = {pos:.3f} turns, Vel = {vel:.3f} turns/s")
            return
    print(f"⚠️ No encoder response from node {node_id}")

# === Main Control Loop ===
print("🚦 Starting multi-node ODrive CAN test...")

for node_id in NODE_IDS:
    if enter_closed_loop(node_id):
        send_velocity(node_id, VEL_SETPOINT)
        time.sleep(0.1)
        request_encoder_estimate(node_id)
    else:
        print(f"⚠️ Skipping velocity command to node {node_id} due to state failure.")
