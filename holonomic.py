#!/usr/bin/env python3

"""
Demonstration code combining:
 - ODrive CAN control of 3 swerve modules (6 motors total).
 - Ruckig-based advanced motion planning (similar to the Phoenix 6 approach).
 - A real-time control loop at ~250 Hz.
 - Setting rolling motor current limits to 5A.

IMPORTANT: This is only a skeleton example. Adapt to your geometry, gear ratios,
encoder offsets, node IDs, etc. Thoroughly test in safe conditions!

Dependencies:
 - python-can (for ODrive CAN)
 - ruckig (for trajectory generation)
 - Possibly threadpoolctl (if you want to limit BLAS usage)
"""

import os
import sys
import time
import math
import queue
import signal
import threading
import struct

import can
import ruckig
import numpy as np

# =====================
# USER CONSTANTS
# =====================
CONTROL_FREQ = 50         # 250 Hz control loop
CONTROL_PERIOD = 1.0 / CONTROL_FREQ
SWERVE_MODULE_COUNT = 3       # We have 3 modules => 6 motors

# ODrive node ID assignments
# Turning motors:  [0, 2, 4]
# Rolling motors:  [1, 3, 5]
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

# We’ll set 5 A max motor current for rolling motors
CURRENT_LIMIT_ROLLING = 5.0

# Example: velocity limits (m/s or rad/s in your operational space) – up to you
MAX_LINEAR_VEL = 0.5   # m/s (example)
MAX_LINEAR_ACC = 0.25  # m/s^2
MAX_ANG_VEL = 1.0      # rad/s
MAX_ANG_ACC = 0.5      # rad/s^2

# =====================
# ODRIVE CAN PROTOCOL
# =====================
FUNC_SET_AXIS_STATE    = 0x07
FUNC_CLEAR_ERRORS      = 0x08
FUNC_SET_LIMITS        = 0x11
FUNC_SET_INPUT_POS     = 0x0C
FUNC_SET_INPUT_VEL     = 0x0D
FUNC_GET_ENCODER_EST   = 0x09
FUNC_HEARTBEAT         = 0x01

AXIS_STATE_IDLE            = 1
AXIS_STATE_CLOSED_LOOP     = 8

def to_arbid(node_id, func_code):
    return (node_id << 5) | func_code

# Helper: set axis state
def set_axis_state(bus, node_id, state):
    arbid = to_arbid(node_id, FUNC_SET_AXIS_STATE)
    data = struct.pack('<I', state)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

# Helper: clear errors
def clear_errors(bus, node_id):
    arbid = to_arbid(node_id, FUNC_CLEAR_ERRORS)
    msg = can.Message(arbitration_id=arbid, data=[], is_extended_id=False)
    bus.send(msg)

# Helper: set velocity/current limit
def set_limits(bus, node_id, vel_limit, curr_limit):
    # According to ODrive docs, Set_Limits (0x11) => <float velocity_limit, float current_limit>
    arbid = to_arbid(node_id, FUNC_SET_LIMITS)
    data = struct.pack('<ff', vel_limit, curr_limit)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

# Helper: set input position
def set_input_pos(bus, node_id, pos):
    arbid = to_arbid(node_id, FUNC_SET_INPUT_POS)
    # <float position, int16 velocity_ff, int16 torque_ff>
    data = struct.pack('<fhh', pos, 0, 0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

# Helper: set input velocity
def set_input_vel(bus, node_id, vel):
    arbid = to_arbid(node_id, FUNC_SET_INPUT_VEL)
    # <float velocity, float torque_ff>
    data = struct.pack('<ff', vel, 0.0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

# =====================
# CLASSES
# =====================

class ODriveMotor:
    """
    Simple wrapper for an ODrive motor node.
    - turning = True => handle as “steering” motor
    - turning = False => handle as “rolling” motor
    """
    def __init__(self, bus, node_id, turning=False):
        self.bus = bus
        self.node_id = node_id
        self.turning = turning

    def set_idle(self):
        set_axis_state(self.bus, self.node_id, AXIS_STATE_IDLE)

    def set_closed_loop(self):
        set_axis_state(self.bus, self.node_id, AXIS_STATE_CLOSED_LOOP)

    def clear_errors(self):
        clear_errors(self.bus, self.node_id)

    def set_limits(self, vel_limit, curr_limit):
        set_limits(self.bus, self.node_id, vel_limit, curr_limit)

    def set_pos(self, pos):
        """
        If a turning motor is commanded in position mode,
        e.g. pos in [turns], or [rad/(2*pi)] depending on your preference.
        """
        set_input_pos(self.bus, self.node_id, pos)

    def set_vel(self, vel):
        """
        If a rolling motor is commanded in velocity mode,
        e.g. vel in [turns/s], or [rad/(2*pi) / s].
        """
        set_input_vel(self.bus, self.node_id, vel)

class SwerveModule:
    """
    A single swerve module with:
      - 1 turning motor
      - 1 rolling motor
    """
    def __init__(self, bus, turn_node, roll_node):
        self.turn_motor = ODriveMotor(bus, turn_node, turning=True)
        self.roll_motor = ODriveMotor(bus, roll_node, turning=False)

    def set_idle(self):
        self.turn_motor.set_idle()
        self.roll_motor.set_idle()

    def set_closed_loop(self):
        self.turn_motor.set_closed_loop()
        self.roll_motor.set_closed_loop()

    def clear_errors(self):
        self.turn_motor.clear_errors()
        self.roll_motor.clear_errors()

    def set_rolling_limit(self, vel_limit, curr_limit):
        """
        For rolling motors, we might want to do something like:
        set_limits( ~some velocity limit in turns/s~, 5.0 )
        """
        self.roll_motor.set_limits(vel_limit, curr_limit)

    def command(self, steer_cmd, roll_cmd):
        """
        Example: 
          steer_cmd = steering angle or steering velocity (in “turns or rad/(2*pi)“).
          roll_cmd  = rolling velocity (turns/s).
        In a real system you might do:
         - position mode for steering
         - velocity mode for rolling
        """
        # For demonstration, let's do:
        #  - Steering in position control (just as an example)
        #  - Rolling in velocity control
        self.turn_motor.set_pos(steer_cmd)
        self.roll_motor.set_vel(roll_cmd)


class SwerveVehicle:
    """
    Full vehicle with 3 swerve modules (6 motors).
    Uses Ruckig for advanced trajectory generation (like your code snippet).
    Real-time loop at 250 Hz.
    """

    def __init__(self):
        # Setup CAN
        self.bus = can.interface.Bus("can0", bustype="socketcan")

        # Create 3 modules
        self.modules = [
            SwerveModule(self.bus, TURNING_NODE_IDS[i], ROLLING_NODE_IDS[i])
            for i in range(SWERVE_MODULE_COUNT)
        ]

        # Clear errors, set closed-loop, set rolling motor current limits
        for mod in self.modules:
            mod.clear_errors()
            time.sleep(0.05)
        for mod in self.modules:
            mod.set_closed_loop()
            time.sleep(0.05)
        # Example: set rolling motor current limit = 5 A, velocity limit = ??? (30 turns/s?)
        for mod in self.modules:
            mod.set_rolling_limit(30.0, CURRENT_LIMIT_ROLLING)

        # Setup Ruckig. We’ll do a 3-DOF example: (vx, vy, w) in “global” coords,
        # or you might do something simpler. 
        self.n_dofs = 3
        self.ruckig = ruckig.Ruckig(self.n_dofs, CONTROL_PERIOD)
        self.inp = ruckig.InputParameter(self.n_dofs)
        self.out = ruckig.OutputParameter(self.n_dofs)
        # Max velocity, acceleration for: (vx, vy, w)
        self.inp.max_velocity = [MAX_LINEAR_VEL, MAX_LINEAR_VEL, MAX_ANG_VEL]
        self.inp.max_acceleration = [MAX_LINEAR_ACC, MAX_LINEAR_ACC, MAX_ANG_ACC]
        self.inp.current_position = [0.0, 0.0, 0.0]  # starting
        self.inp.current_velocity = [0.0, 0.0, 0.0]

        # Start with a velocity command
        # We'll do velocity control in Ruckig (like "control_interface = Velocity")
        self.inp.control_interface = ruckig.ControlInterface.Velocity
        self.inp.target_velocity = [0.0, 0.0, 0.0]

        # Real-time thread
        self.command_queue = queue.Queue(maxsize=1)
        self.run_thread = True
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)

    def start_control(self):
        self.control_thread.start()

    def stop_control(self):
        self.run_thread = False
        self.control_thread.join()
        # Put modules into IDLE
        for mod in self.modules:
            mod.set_idle()

    def control_loop(self):
        """
        Main ~250 Hz loop, reading from Ruckig to get (vx_d, vy_d, w_d).
        Then we must figure out each module’s steering + rolling velocities/positions.
        For simplicity here, we do a naive approach:
         - Each module is at some angle (we’re ignoring real feedback in this snippet).
         - Command rolling velocity = magnitude
         - Command steering angle = direction
        """
        last_time = time.time()
        while self.run_thread:
            # Timing
            t0 = time.time()
            dt = t0 - last_time
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_time = t0

            # If a new command arrived, update Ruckig’s target velocity
            if not self.command_queue.empty():
                new_cmd = self.command_queue.get_nowait()
                if new_cmd["type"] == "velocity":
                    # e.g. new_cmd["vel"] = [vx, vy, w]
                    self.inp.control_interface = ruckig.ControlInterface.Velocity
                    self.inp.target_velocity = new_cmd["vel"]

            # Update Ruckig
            res = self.ruckig.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)  # recommended for next iteration

            # Current commanded velocity in the “global” frame
            vx_d, vy_d, w_d = self.out.new_velocity

            # Example: naive swerve “inverse kinematics”
            # Just for demonstration: each of 3 modules spaced 120° around center
            # Real code: you’d do a more thorough transform based on module geometry
            # We'll do (theta_module_i, speed_module_i)
            # Here, let's just do an all-forward approach: each module’s steer=0, roll = same
            # so the robot goes forward in vx_d.
            # This is obviously oversimplified. Replace with your real transform.

            # For demonstration:
            #   - If vx_d or vy_d is nonzero, find heading angle = atan2(vy_d, vx_d)
            #   - Rolling speed = sqrt(vx_d^2 + vy_d^2)
            heading = math.atan2(vy_d, vx_d) if (abs(vx_d) + abs(vy_d) > 1e-6) else 0.0
            speed = math.hypot(vx_d, vy_d)

            # Then each module: set steer angle = heading + offset_i, set roll velocity = speed
            # ignoring w_d (the rotational velocity) for brevity
            # In real swerve, you'd incorporate w_d as well.

            # Command each module
            for i, mod in enumerate(self.modules):
                # offset_i for module i if you want them at different angles:
                # For a 3-module robot, let's do 120 deg spacing maybe. But a simpler approach
                # is just all modules are the same orientation. Up to you.
                offset_i = 0.0
                steer_angle = heading + offset_i

                # For ODrive turning: let's assume 1.0 in “pos” = 1 turn.
                # If you prefer rad-based, do: steer_angle / (2*pi).
                # (We’re ignoring actual steer angle feedback here!)
                steer_cmd_odrive = steer_angle / (2 * math.pi)

                # Rolling velocity: in ODrive “turns/s”
                roll_cmd_odrive = speed  # naive: 1 m/s => 1 turn/s? You must calibrate

                mod.command(steer_cmd_odrive, roll_cmd_odrive)

        # end while loop

    def set_target_velocity(self, vx, vy, w):
        """
        Sends a velocity command in the global frame
        """
        cmd = {
            "type": "velocity",
            "vel": [vx, vy, w]
        }
        if self.command_queue.full():
            print("Warning: command queue is full. Control loop might be stalled.")
        else:
            self.command_queue.put(cmd, block=False)


# =====================
# MAIN
# =====================

def main():
    vehicle = SwerveVehicle()
    vehicle.start_control()

    try:
        print("Sending a small circular velocity command for 10 seconds...")
        t_start = time.time()
        while time.time() - t_start < 10.0:
            # Let’s do a slow circle
            # vx=0.2 m/s forward, w=0.3 rad/s
            vehicle.set_target_velocity(0.2, 0.0, 0.3)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("KeyboardInterrupt - stopping control.")
    finally:
        print("Stopping control thread, setting motors IDLE.")
        vehicle.stop_control()

if __name__ == "__main__":
    main()
