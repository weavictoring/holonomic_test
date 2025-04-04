import can

bus = can.interface.Bus(channel='can0', bustype='socketcan')

print("🔍 Listening for any CAN traffic...")
while True:
    msg = bus.recv()
    print(f"ID: 0x{msg.arbitration_id:X}, Data: {msg.data.hex()}")
