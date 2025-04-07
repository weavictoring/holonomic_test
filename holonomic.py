#!/usr/bin/env python3

"""
Demonstration script that:
1) Loads motor_config.json for each ODrive node => flip, offset
2) Spawns a background thread to parse ODrive encoder feedback (ENCODER_ESTIMATES, 0x09)
3) Sets rolling motors to a 5A current limit
4) Uses Ruckig for advanced control
5) Commands turning motors with position = offset + flip * desiredAngle
   and rolling motors with velocity = flip * desiredVelocity
6) Runs at ~250 Hz, but do adapt as needed

Adapt carefully to your real swerve geometry, calibration, offsets, units, etc.

Dependencies:
 - python-can
 - ruckig
 - (optional) threadpoolctl if you want to limit BLAS usage

Test carefully at low current & speed with the robot wheels off the ground!
"""

import os
import sys
import math
import json
import time
import queue
import signal
import struct
import threading
import can
import ruckig
import numpy as np

# ========= USER CONFIG =========
CONTROL_FREQ = 50
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0  # 5 A limit for rolling motors
VEL_LIMIT_ROLLING = 2     # example velocity limit in "turns/s" – adjust as needed

# If you do advanced 2D control, let's say we have 3 DOFs: (vx, vy, w).
# We'll keep this example simpler: just command a single linear velocity & single turning angle
MAX_LINEAR_VEL = 1.0   # Example: 1 turn/s or a placeholder if you do actual  m/s
MAX_ANG_VEL = 0.5      # Example: 0.5 turns/s for turning. Adjust to your liking

# Paths
MOTOR_CONFIG_FILE = "motor_config.json"

# ========= ODrive CAN PROTOCOL =========
FUNC_SET_AXIS_STATE  = 0x07
FUNC_CLEAR_ERRORS    = 0x08
FUNC_SET_LIMITS      = 0x11
FUNC_SET_INPUT_POS   = 0x0C
FUNC_SET_INPUT_VEL   = 0x0D
FUNC_ENCODER_EST     = 0x09
FUNC_HEARTBEAT       = 0x01

AXIS_STATE_IDLE          = 1
AXIS_STATE_CLOSED_LOOP   = 8

def to_arbid(node_id, func):
    return (node_id << 5) | func

def set_axis_state(bus, node_id, state):
    arbid = to_arbid(node_id, FUNC_SET_AXIS_STATE)
    data = struct.pack('<I', state)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

def clear_errors(bus, node_id):
    arbid = to_arbid(node_id, FUNC_CLEAR_ERRORS)
    msg = can.Message(arbitration_id=arbid, data=[], is_extended_id=False)
    bus.send(msg)

def set_limits(bus, node_id, vel_limit, curr_limit):
    arbid = to_arbid(node_id, FUNC_SET_LIMITS)
    data = struct.pack('<ff', vel_limit, curr_limit)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

def set_input_pos(bus, node_id, pos):
    """
    ODrive expects <float position, int16 velocity_ff, int16 torque_ff>, all little-endian
    'pos' is in turns. E.g. 1.0 => 1 turn => 2π rad if you're in default config
    """
    arbid = to_arbid(node_id, FUNC_SET_INPUT_POS)
    data = struct.pack('<fhh', pos, 0, 0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

def set_input_vel(bus, node_id, vel):
    """
    ODrive expects <float velocity, float torque_ff>
    'vel' is in turns/s
    """
    arbid = to_arbid(node_id, FUNC_SET_INPUT_VEL)
    data = struct.pack('<ff', vel, 0.0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

# ========= BACKGROUND FEEDBACK THREAD =========
class ODriveFeedbackCollector(threading.Thread):
    """
    A daemon thread that listens to 'bus.recv()'
    and captures ENCODER_ESTIMATES (pos, vel) for each node.
    The raw feedback is stored in a dict: feedback[node_id] = (pos, vel)
    'pos' and 'vel' are in ODrive "turns" and "turns/s" by default.
    """
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.daemon = True
        self.run_flag = True
        # We store the raw pos, vel in a thread-safe dict
        self.feedback = {}
        for nid in ALL_NODE_IDS:
            self.feedback[nid] = (0.0, 0.0)

    def run(self):
        while self.run_flag:
            msg = self.bus.recv(timeout=0.1)
            if not msg:
                continue
            func = (msg.arbitration_id & 0x1F)
            node_id = msg.arbitration_id >> 5
            # Check if it's a response or broadcast of ENCODER_ESTIMATES
            # According to ODrive docs, broadcast uses ID = node<<5 | 0x09
            if func == FUNC_ENCODER_EST and node_id in ALL_NODE_IDS:
                # parse <float pos, float vel> in little-endian
                pos, vel = struct.unpack('<ff', msg.data)
                self.feedback[node_id] = (pos, vel)

    def stop(self):
        self.run_flag = False

# ========= MOTOR WRAPPERS =========
class ODriveMotor:
    """
    Simple wrapper for a single ODrive node, with offset & flip from motor_config.json
    """
    def __init__(self, bus, node_id, offset=0.0, flip=1.0, is_turning=False):
        self.bus = bus
        self.node_id = node_id
        self.offset = offset
        self.flip = flip
        self.is_turning = is_turning

    def clear_errors(self):
        clear_errors(self.bus, self.node_id)

    def set_closed_loop(self):
        set_axis_state(self.bus, self.node_id, AXIS_STATE_CLOSED_LOOP)

    def set_idle(self):
        set_axis_state(self.bus, self.node_id, AXIS_STATE_IDLE)

    def set_limits(self, vel_lim, curr_lim):
        set_limits(self.bus, self.node_id, vel_lim, curr_lim)

    def command_position(self, raw_angle):
        """
        'raw_angle' is your final ODrive 'turns'.
        Typically = offset + flip*(some angle in turns).
        """
        set_input_pos(self.bus, self.node_id, raw_angle)

    def command_velocity(self, raw_vel):
        """
        'raw_vel' is your final ODrive velocity in turns/s.
        Typically = flip*(some commanded speed).
        """
        set_input_vel(self.bus, self.node_id, raw_vel)

class SwerveModule:
    """
    One swerve module with a turning motor + rolling motor
    """
    def __init__(self, turn_motor, roll_motor):
        self.turn_motor = turn_motor
        self.roll_motor = roll_motor

    def clear_errors(self):
        self.turn_motor.clear_errors()
        self.roll_motor.clear_errors()

    def set_closed_loop(self):
        self.turn_motor.set_closed_loop()
        self.roll_motor.set_closed_loop()

    def set_idle(self):
        self.turn_motor.set_idle()
        self.roll_motor.set_idle()

    def set_rolling_limits(self, vel_limit, curr_limit):
        self.roll_motor.set_limits(vel_limit, curr_limit)

    def command(self, steer_angle_in_turns, roll_vel_in_turns_s):
        """
        Example usage:
          steer_angle_in_turns = offset + flip * desiredAngle
          roll_vel_in_turns_s  = flip * desiredVelocity
        """
        self.turn_motor.command_position(steer_angle_in_turns)
        self.roll_motor.command_velocity(roll_vel_in_turns_s)

# ========= VEHICLE CLASS WITH RUCKIG LOOP =========
class SwerveVehicle:
    """
    3 swerve modules => 6 motors total. 
    We'll do a single advanced control loop at ~250 Hz using Ruckig, 
    commanding (some linear velocity, maybe an orientation, etc.)
    This is a simplified demonstration, ignoring real geometry.
    """
    def __init__(self, bus, feedback_collector, module_list):
        self.bus = bus
        self.modules = module_list
        self.feedback = feedback_collector  # background thread storing raw pos, vel
        # Start feedback
        self.feedback.start()

        # Ruckig setup – let's say 1-DOF for demonstration: target forward velocity
        # If you want 3 DOFs (vx, vy, w), set dofs=3, etc.
        self.dofs = 1
        self.rk = ruckig.Ruckig(self.dofs, CONTROL_PERIOD)
        self.inp = ruckig.InputParameter(self.dofs)
        self.out = ruckig.OutputParameter(self.dofs)

        # Max velocity, accel
        self.inp.max_velocity = [MAX_LINEAR_VEL]
        self.inp.max_acceleration = [0.5 * MAX_LINEAR_VEL]  # example
        self.inp.current_position = [0.0]
        self.inp.current_velocity = [0.0]
        self.inp.control_interface = ruckig.ControlInterface.Velocity
        self.inp.target_velocity = [0.0]

        self.cmd_queue = queue.Queue(maxsize=1)
        self.run_flag = True
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True)

    def start(self):
        self.ctrl_thread.start()

    def stop(self):
        self.run_flag = False
        self.ctrl_thread.join()
        self.feedback.stop()
        # Put modules in IDLE
        for m in self.modules:
            m.set_idle()

    def set_target_velocity(self, v):
        """
        Sets new 1D velocity for demonstration.
        If you want multi-DOF, do e.g. target_velocity=[vx,vy,w].
        """
        if self.cmd_queue.full():
            print("Warning: command queue is full.")
        else:
            self.cmd_queue.put({"type":"vel","value":v}, block=False)

    def control_loop(self):
        """
        ~50 Hz. 
        Example approach:
          - read Ruckig output => final velocity
          - compute each module's turning angle & wheel velocity
          - apply offset+flip to turning angle
          - apply flip to rolling velocity
          - send to ODrive
        """
        last_t = time.time()
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now

            # Check new command
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get_nowait()
                if cmd["type"] == "vel":
                    self.inp.target_velocity = [cmd["value"]]

            # Run Ruckig
            self.rk.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            v_d = self.out.new_velocity[0]

            # For demonstration: 
            #   We'll just command each module to the same steer angle = 0,
            #   rolling velocity = v_d. 
            # Real swerve geometry would do a proper inverse kinematics.
            # Then apply offset+flip for turning, flip for rolling.

            # We DO have offset & flip data in each motor, so let's do:
            # turningMotorFinalPos = offset + flip * desiredAngle
            # rollingMotorFinalVel = flip * desiredVelocity
            # We'll set desiredAngle=0 for demonstration.

            desired_steer_angle_in_turns = 0.0  # no turning
            desired_roll_vel_in_turns_s = v_d   # naive

            # Command each module
            for mod in self.modules:
                mod.command(desired_steer_angle_in_turns, desired_roll_vel_in_turns_s)

            # We also have raw feedback in `self.feedback.feedback[node_id]`
            # If you want real-time logging or closed-loop kinematics, read it here.

# ========= MAIN SCRIPT =========
def main():
    # 1) Load motor_config
    if not os.path.exists(MOTOR_CONFIG_FILE):
        print(f"Error: missing {MOTOR_CONFIG_FILE}. Please create it.")
        sys.exit(1)
    with open(MOTOR_CONFIG_FILE, 'r') as f:
        mcfg = json.load(f)  # dict of dict

    # 2) Create CAN bus
    bus = can.interface.Bus("can0", bustype="socketcan")

    # 3) Clear errors, set closed-loop, set rolling current limit
    #    Also create ODriveMotor objects
    turning_motors = []
    rolling_motors = []
    for nid in TURNING_NODE_IDS:
        offset = mcfg.get(str(nid), {}).get("offset", 0.0)
        flip   = mcfg.get(str(nid), {}).get("flip", 1.0)
        m = ODriveMotor(bus, nid, offset, flip, is_turning=True)
        turning_motors.append(m)

    for nid in ROLLING_NODE_IDS:
        offset = mcfg.get(str(nid), {}).get("offset", 0.0)
        flip   = mcfg.get(str(nid), {}).get("flip", 1.0)
        m = ODriveMotor(bus, nid, offset, flip, is_turning=False)
        rolling_motors.append(m)

    # 4) Pair them into swerve modules
    #    If you have 3 modules, we expect turning_motors[i] pairs with rolling_motors[i]
    #    Adjust indexing carefully based on your real node ID layout
    modules = []
    for i in range(len(turning_motors)):
        mod = SwerveModule(turning_motors[i], rolling_motors[i])
        modules.append(mod)

    # 5) Clear errors, set closed-loop, set rolling 5A limit
    for mod in modules:
        mod.clear_errors()
        time.sleep(0.05)
    time.sleep(0.2)

    for mod in modules:
        mod.set_closed_loop()
        time.sleep(0.05)
        mod.set_rolling_limits(VEL_LIMIT_ROLLING, CURRENT_LIMIT_ROLLING)

    # 6) Start background feedback collector
    fb_collector = ODriveFeedbackCollector(bus)

    # 7) Create your advanced "vehicle" object with Ruckig or other logic
    vehicle = SwerveVehicle(bus, fb_collector, modules)
    vehicle.start()

    # 8) Demo: ramp up velocity for 5 seconds, then stop
    try:
        print("Starting 5s demonstration: gradually ramp to velocity 0.5 turns/s")
        t0 = time.time()
        while time.time() - t0 < 5.0:
            # Example: set velocity 0.5 turns/s
            vehicle.set_target_velocity(0.5)
            time.sleep(0.05)

        print("Done. Setting velocity=0, stopping control in 2s...")
        vehicle.set_target_velocity(0.0)
        time.sleep(2.0)

    except KeyboardInterrupt:
        print("CTRL+C - stopping.")
    finally:
        vehicle.stop()
        print("All done.")

if __name__ == "__main__":
    main()
