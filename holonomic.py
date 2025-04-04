#!/usr/bin/env python3
"""
Minimal example of controlling a 3-caster (6-motor) swerve drive
via ODrive CAN from a Raspberry Pi + CAN Hat.

Each caster has:
 - a "turn" motor (steering) -> Node IDs: 0, 2, 4
 - a "drive" motor (rolling) -> Node IDs: 1, 3, 5

IMPORTANT: This is a simple demonstration of sending velocity
           setpoints. Adapt it to your actual gear ratios,
           ODrive velocity units, and swerve logic.

Author: [Your Name]
Date: [Month Year]
"""

import time
import math
import can

##############################################################################
#                           USER-DEFINED PARAMS                               #
##############################################################################

# --- Pi CAN interface name (often 'can0') ---
CAN_INTERFACE = 'can0'

# --- ODrive node IDs for 3 turning axes and 3 driving axes ---
TURN_NODE_IDS  = [0, 2, 4]  # Example
DRIVE_NODE_IDS = [1, 3, 5]  # Example

# --- ODrive "set velocity" command offset ---
# Depending on ODrive firmware, this might be 0x00 or 0x07, etc.
ODRIVE_VEL_CMD_OFFSET = 0x00

# --- Maximum velocity commands (in encoder counts/s) to limit outputs ---
TURN_SPEED_MAX  = 30000  # example limit
DRIVE_SPEED_MAX = 50000  # example limit

# --- Robot geometry for 3-caster swerve (example) ---
L = 0.25    # radius from center to each caster (m)
WHEEL_R = 0.05  # wheel radius (m)

##############################################################################
#                          INVERSE KINEMATICS                                 #
##############################################################################

def swerve_ik(vx, vy, omega):
    """
    Basic 3-wheel swerve inverse kinematics.

    Inputs:
      vx    : robot velocity in X (m/s)
      vy    : robot velocity in Y (m/s)
      omega : robot angular velocity about Z (rad/s)

    Returns:
      turn_cmds  : list of steering motor speeds (counts/s) (length=3)
      drive_cmds : list of drive motor speeds (counts/s) (length=3)
    """
    # Three modules spaced 120 deg apart
    angles = [0, 2*math.pi/3, 4*math.pi/3]

    turn_cmds  = []
    drive_cmds = []

    for ang in angles:
        # In real swerve, you typically do position control for the steering axis.
        # For demonstration, let's say we send a velocity command based on omega:
        steer_speed = omega * 3000.0  # TOTALLY arbitrary scale for demonstration
        # Limit to max safe value
        steer_speed = max(-TURN_SPEED_MAX, min(TURN_SPEED_MAX, steer_speed))

        # For the drive speed, let's do a naive approach:
        # - Project [vx, vy] onto direction of the wheel
        # - Add tangential speed from rotation about the center
        wheel_dir_x = math.cos(ang)
        wheel_dir_y = math.sin(ang)

        linear_part = vx * wheel_dir_x + vy * wheel_dir_y
        # For rotation: tangential speed = omega * radius (assuming the wheel is oriented tangentially)
        # This is simplified! Real swerve logic is more nuanced (you find the actual orientation).
        # We'll just add or subtract something for demonstration.
        # e.g. tangential = omega * L (some sign depending on angle)
        tangential = omega * L  # simplistic

        wheel_speed_m_s = linear_part + tangential

        # Convert m/s to ODrive motor speed in "counts/s"
        # (You must incorporate gear ratio and encoder CPR).
        # For demonstration, let's do a big scale factor:
        drive_cmd = wheel_speed_m_s * 10000.0
        drive_cmd = max(-DRIVE_SPEED_MAX, min(DRIVE_SPEED_MAX, drive_cmd))

        turn_cmds.append(steer_speed)
        drive_cmds.append(drive_cmd)

    return turn_cmds, drive_cmds

##############################################################################
#                           ODRIVE CAN HELPER                                 #
##############################################################################

class ODriveCAN:
    """
    Minimal helper class to send velocity commands to ODrive via CAN.
    """

    def __init__(self, interface):
        # python-can uses "socketcan" interface on Linux (Raspberry Pi).
        # Make sure 'can0' is up with the correct bitrate before running.
        self.bus = can.interface.Bus(channel=interface, bustype='socketcan')

    def send_velocity_command(self, node_id, vel_counts_s, torque_ff=0):
        """
        Send velocity setpoint to ODrive axis (Node ID = node_id).
        The ODrive expects data in certain formats; check your firmware doc.

        Typically, velocity is a 32-bit int (counts/s * 0.001?), feed-forward torque is 32-bit int.
        The code below is an example. Adjust scale as needed for your ODrive version.
        """
        # Example: many ODrive firmwares interpret the velocity as an int32 in 0.001 counts/s.
        vel_int = int(vel_counts_s * 0.001)  
        trq_int = int(torque_ff * 1000)

        arb_id = (node_id << 5) + ODRIVE_VEL_CMD_OFFSET
        data   = vel_int.to_bytes(4, byteorder='little', signed=True) + \
                 trq_int.to_bytes(4, byteorder='little', signed=True)

        msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
        try:
            self.bus.send(msg)
        except can.CanError:
            print(f"Failed to send velocity cmd to node {node_id}")

##############################################################################
#                              MAIN LOOP                                     #
##############################################################################

if __name__ == "__main__":
    odrive_can = ODriveCAN(CAN_INTERFACE)

    # Example desired motion: move forward at 0.2 m/s, no lateral, small rotation
    vx_des = 0.2
    vy_des = 0.0
    w_des  = 0.3  # rad/s

    try:
        while True:
            # Compute swerve velocities
            turn_cmds, drive_cmds = swerve_ik(vx_des, vy_des, w_des)

            # Send to each of the 3 caster modules
            for i in range(3):
                odrive_can.send_velocity_command(TURN_NODE_IDS[i],  turn_cmds[i])
                odrive_can.send_velocity_command(DRIVE_NODE_IDS[i], drive_cmds[i])

            # 50 Hz loop
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("Stopping by user request.")
