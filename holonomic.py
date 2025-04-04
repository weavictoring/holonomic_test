import can
import struct
import time
import signal
import sys

NODE_IDS = list(range(6))  # Node IDs 0–5
TARGET_POSITION = 0.5  # turns
VEL_FF = 0
TORQUE_FF = 0

# CANSimple function codes
SET_AXIS_STATE = 0x07
HEARTBEAT = 0x01
SET_INPUT_POS = 0x0C
GET_ENCODER_ESTIMATES = 0x09

bus = can.interface.Bus("can0", bustype="socketcan")

# Flush old CAN messages
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
                print(f"✅ Node {node_id} entered CLOSED_LOOP_CONTROL.")
                return True
    print(f"❌ Node {node_id} failed to enter closed-loop.")
    return False

def send_position(node_id, pos):
    msg = can.Message(
        arbitration_id=to_arbitration_id(node_id, SET_INPUT_POS),
        data=struct.pack('<fhh', pos, int(VEL_FF), int(TORQUE_FF)),
        is_extended_id=False
    )
    bus.send(msg)

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
    for node_id in NODE_IDS:
        set_axis_state(node_id, 1)  # IDLE
        print(f"🛑 Node {node_id} set to IDLE.")
    sys.exit(0)

# === Handle Ctrl+C cleanly ===
signal.signal(signal.SIGINT, lambda sig, frame: shutdown_all_nodes())

# === Main loop ===
try:
    print(f"\n🎯 Syncing all turning motors to {TARGET_POSITION:.3f} turns...\n")

    for node_id in NODE_IDS:
        if enter_closed_loop(node_id):
            send_position(node_id, TARGET_POSITION)
        else:
            print(f"⚠️ Skipping node {node_id} (not ready).")

    # Continuous monitoring
    while True:
        for node_id in NODE_IDS:
            request_encoder(node_id)
        time.sleep(0.5)

except Exception as e:
    print(f"\n⚠️ Exception occurred: {e}")
    shutdown_all_nodes()
