import can
import struct
import time

# IDs from 0 to 5
NODE_IDS = list(range(6))
BITRATE = 250000

# Function codes from ODrive CAN protocol
FUNC_ENCODER_ESTIMATES = 0x09
FUNC_IQ = 0x0A
FUNC_BUS_VOLTAGE_CURRENT = 0x17
FUNC_ERROR = 0x03

# Request/response ID helpers
def req_id(func, node_id): return (func << 5) | node_id
def resp_id(func, node_id): return ((func | 0x10) << 5) | node_id

# CAN bus init (make sure can0 is up separately before running this)
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Function to send a request and wait for response
def send_request(node_id, func_code):
    rid = req_id(func_code, node_id)
    resp = resp_id(func_code, node_id)

    msg = can.Message(arbitration_id=rid, data=[], is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanError as e:
        print(f"CAN send error to node {node_id}: {e}")
        return None

    start = time.time()
    while time.time() - start < 0.5:
        rx = bus.recv(timeout=0.1)
        if rx and rx.arbitration_id == resp:
            return rx.data
    return None

# Function to scan nodes and print encoder position/velocity
def scan_odrives():
    for node_id in NODE_IDS:
        print(f"\n🧭 Scanning node {node_id}...")

        data = send_request(node_id, FUNC_ENCODER_ESTIMATES)
        if data:
            pos, vel = struct.unpack('<ff', data)
            print(f"✅ Node {node_id} OK | Pos: {pos:.2f}, Vel: {vel:.2f}")
        else:
            print(f"⚠️ No response from node {node_id}")

if __name__ == "__main__":
    print("🚦 Scanning ODrive nodes on CAN bus (can0)...\n")
    scan_odrives()
