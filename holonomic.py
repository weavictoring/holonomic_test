# holonomic_odrive_base.py
# Control script for a holonomic base with 6 ODrive-controlled motors via CAN

import can
import struct
import time
import numpy as np

# Define constants
CONTROL_FREQ = 250
CONTROL_PERIOD = 1.0 / CONTROL_FREQ
TWO_PI = 2 * np.pi
NODE_IDS = {
    "steer": [0, 2, 4],  # steer motors (aligned)
    "drive": [1, 3, 5],  # drive motors
}

# CAN IDs (ODrive native protocol)
FUNC_SET_VELOCITY = 0x0d
FUNC_HEARTBEAT = 0x001

def make_id(function_code, node_id):
    return (function_code << 5) | node_id

class ODriveMotor:
    def __init__(self, bus, node_id):
        self.bus = bus
        self.node_id = node_id

    def set_velocity(self, velocity_rps):
        msg_id = make_id(FUNC_SET_VELOCITY, self.node_id)
        data = struct.pack('<f', velocity_rps) + b'\x00\x00\x00\x00'
        msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=False)
        self.bus.send(msg)

class HolonomicBase:
    def __init__(self, can_bus):
        self.bus = can_bus
        self.steer_motors = [ODriveMotor(self.bus, nid) for nid in NODE_IDS["steer"]]
        self.drive_motors = [ODriveMotor(self.bus, nid) for nid in NODE_IDS["drive"]]

    def align_steer_motors(self, angle_rad):
        velocity_rps = 0.0  # hold position (we're using velocity control for now)
        for motor in self.steer_motors:
            motor.set_velocity(velocity_rps)

    def drive(self, vx, vy, omega):
        # Simple placeholder logic - forward motion only for example
        # Later: use proper inverse kinematics
        for motor in self.drive_motors:
            motor.set_velocity(vx * 10)  # scale to RPS

    def stop(self):
        for motor in self.steer_motors + self.drive_motors:
            motor.set_velocity(0.0)

if __name__ == '__main__':
    bus = can.interface.Bus(channel='can0', bustype='socketcan')
    base = HolonomicBase(bus)

    try:
        print("Starting holonomic base control loop...")
        for _ in range(250):  # Run for 1 second
            base.align_steer_motors(0.0)
            base.drive(0.2, 0.0, 0.0)  # move forward
            time.sleep(CONTROL_PERIOD)
    finally:
        base.stop()
        print("Stopped holonomic base")
