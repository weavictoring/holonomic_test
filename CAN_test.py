import can
import struct
import time

bus = can.interface.Bus(channel='can0', bustype='socketcan')

NODE_IDS = list(range(6))  # 0 to 5
FUNC_CODE = 0x17  # Get_Bus_Voltage_Current

def request_voltage_current(node_id):
    request_id = (FUNC_CODE << 5) | node_id
    response_id = ((FUNC_CODE | 0x10) << 5) | node_id

    msg = can.Message(arbitration_id=request_id, data=[], is_extended_id=False)
    try:
        bus.send(msg)
        print(f"Sent to node {node_id}: 0x{request_id:03X}")
    except can.CanError as e:
        print(f"Send error: {e}")
        return

    start = time.time()
    while time.time() - start < 1.0:
        msg = bus.recv(timeout=0.2)
        if msg and msg.arbitration_id == response_id:
            voltage, current = struct.unpack('<ff', msg.data)
            print(f"Node {node_id}: Bus Voltage = {voltage:.2f} V, Current = {current:.2f} A")
            return
    print(f"No response from node {node_id}")

for node_id in NODE_IDS:
    request_voltage_current(node_id)
