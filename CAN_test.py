import can
import struct
import time

# ODrive CAN protocol uses 11-bit IDs
# Sample node ID. Change if your ODrive uses a different one.
NODE_ID = 0x01

# Standard CAN IDs for reading ODrive data (requesting)
ODRIVE_CAN_IDS = {
    "heartbeat": 0x001,
    "get_encoder_estimates": 0x009,
    "get_iq": 0x00A,
    "get_bus_voltage_current": 0x017,
    "get_error": 0x003,
}

# Request IDs are formed by: 0x00 + function_id << 5 + node_id
def get_request_id(function_code, node_id=NODE_ID):
    return (function_code << 5) | node_id

# Setup CAN bus
bus = can.interface.Bus(channel='can0', bustype='socketcan')

# Function to send a CAN request and wait for the response
def send_request_and_receive(request_id, response_id, timeout=0.5):
    msg = can.Message(arbitration_id=request_id, data=[], is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Sent request with ID: 0x{request_id:03X}")
    except can.CanError as e:
        print(f"CAN send error: {e}")
        return None

    # Wait for response
    start = time.time()
    while time.time() - start < timeout:
        response = bus.recv(timeout)
        if response and response.arbitration_id == response_id:
            return response.data
    print(f"Timeout waiting for response to 0x{response_id:03X}")
    return None

# Decode and print various ODrive info
def read_encoder_estimates():
    request_id = get_request_id(0x09)
    response_id = get_request_id(0x09 | 0x10)  # response bit set

    data = send_request_and_receive(request_id, response_id)
    if data:
        pos, vel = struct.unpack('<ff', data)
        print(f"Encoder Position: {pos:.3f} turns, Velocity: {vel:.3f} turns/s")

def read_iq():
    request_id = get_request_id(0x0A)
    response_id = get_request_id(0x0A | 0x10)

    data = send_request_and_receive(request_id, response_id)
    if data:
        iq_setpoint, iq_measured = struct.unpack('<ff', data)
        print(f"Iq Setpoint: {iq_setpoint:.3f} A, Iq Measured: {iq_measured:.3f} A")

def read_bus_voltage_current():
    request_id = get_request_id(0x17)
    response_id = get_request_id(0x17 | 0x10)

    data = send_request_and_receive(request_id, response_id)
    if data:
        voltage, current = struct.unpack('<ff', data)
        print(f"Bus Voltage: {voltage:.2f} V, Bus Current: {current:.2f} A")

def read_error():
    request_id = get_request_id(0x03)
    response_id = get_request_id(0x03 | 0x10)

    data = send_request_and_receive(request_id, response_id)
    if data:
        axis_error = struct.unpack('<I', data[:4])[0]
        print(f"Axis Error: 0x{axis_error:08X}")

# === MAIN ===
if __name__ == "__main__":
    print("Reading ODrive data via CAN...\n")
    read_encoder_estimates()
    read_iq()
    read_bus_voltage_current()
    read_error()
