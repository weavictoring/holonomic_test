import can
import struct
import time
import signal
import sys

# Customize your motor groups here
TURNING_NODE_IDS = [0, 1, 2]
ROLLING_NODE_IDS = [3, 4, 5]

TARGET_ANGLE = 0.5    # turns for turning motors
TARGET_VELOCITY = 1.0  # turns/s for rolling motors

# CANSimple function codes
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
SET_INPUT_POS = 0x0C
SET_INPUT_VEL = 0x0D
GET_ENCODER_ESTIMATES = 0x09

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush old messages
while not (bus.recv(timeout=0) is None): pass

def to_arbitration_id(node_id, func_code):
    return (node_id << 5) | func_code

def set_axis_state(node_id, state):
    bus.send(can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_AXIS_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False
    ))

def enter_closed_loop(node_id):
    set_axis_state(node_id, 8)  # CLOSED_LOOP_CONTROL
    timeout = time.time() + 2
    while time.time() < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == to_arbitration_id(node_id, HEARTBEAT):
            _, state, _, _ = struct.unpack('<IBBB', msg.data[:7])
            if state == 8:
                print(f"✅ Node {node_id} is in CLOSED_LOOP_CONTROL")
                return True
    print(f"❌ Node {node_id} failed to enter closed-loop")
    return False

def send_position(node_id, pos):
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_INPUT_POS),
        data=struct.pack('<fhh', pos, 0, 0),  # position, vel_ff, torque_ff
        is_extended_id=False
    )
    bus.send(msg)
    print(f"🎯 Turning Node {node_id}: sent position {pos:.3f}")

def send_velocity(node_id, velocity):
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_INPUT_VEL),
        data=struct.pack('<ff', velocity, 0.0),  # velocity, torque_ff
        is_extended_id=False
    )
    bus.send(msg)
    print(f"🚀 Rolling Node {node_id}: sent velocity {velocity:.3f}")

def request_encoder(node_id):
    request_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES)
    response_id = to_arbitration_id(node_id, GET_ENCODER_ESTIMATES | 0x10)
    bus.send(can.Message(arbitration_id=request_id, data=[], is_extended_id=False))
    timeout = time.time() + 0.5
    while time.time() < timeout:
        msg = bus.recv(timeout=0.1)
        if msg and msg.arbitration_id == response_id:
            pos, vel = struct.unpack('<ff', msg.data)
            print(f"📍 Node {node_id}: Pos = {pos:.3f}, Vel = {vel:.3f}")
            return
    print(f"⚠️ No encoder response from node {node_id}")

def shutdown_all_nodes():
    print("\n🛑 Ctrl+C detected! Stopping all nodes...")
    for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
        set_axis_state(node_id, 1)  # IDLE
        print(f"🛑 Node {node_id} set to IDLE.")
    sys.exit(0)

# === Ctrl+C handling ===
signal.signal(signal.SIGINT, lambda sig, frame: shutdown_all_nodes())

# === Main control loop ===
try:
    print("🔄 Entering closed-loop mode...")
    for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
        enter_closed_loop(node_id)

    print(f"\n🎯 Sending position {TARGET_ANGLE:.3f} to turning motors...")
    for node_id in TURNING_NODE_IDS:
        send_position(node_id, TARGET_ANGLE)

    print(f"\n🚀 Sending velocity {TARGET_VELOCITY:.3f} to rolling motors...")
    for node_id in ROLLING_NODE_IDS:
        send_velocity(node_id, TARGET_VELOCITY)

    # Live encoder feedback loop
    while True:
        for node_id in TURNING_NODE_IDS + ROLLING_NODE_IDS:
            request_encoder(node_id)
        time.sleep(0.5)

except Exception as e:
    print(f"\n❌ Exception occurred: {e}")
    shutdown_all_nodes()
