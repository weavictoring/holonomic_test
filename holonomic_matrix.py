#!/usr/bin/env python3
"""
3-Wheel Holonomic Base with Powered Caster Drive Using Joystick Control (via evdev)
 
This script:
  1) Loads motor_config.json for each ODrive node (including flip, offset, and optionally wheel_positions).
  2) Spawns a background thread to parse ODrive encoder feedback.
  3) Sets rolling motors to a 5A current limit.
  4) Uses a simplified control loop (without Ruckig) for 3 DOFs (vx, vy, ω).
  5) Computes each module’s desired turning angle and rolling velocity by combining:
       - Translation: from the left joystick (x,y plane)
       - Rotation: from the right joystick (rotation about z)
     A caster offset is factored in for each wheel.
  6) Uses a proportional controller for the turning motors—here the inverse-kinematics calculations are done 
     via matrices (vectorized computation) instead of a loop.
  7) Logs live status: for each module the desired (turn) angle, the real-time encoder angle (with offset applied),
     and the rolling command.
  8) Runs the control loop at ~50 Hz.
 
Test carefully at low current & speed with the robot wheels off the ground!
"""

import os, sys, math, json, time, threading, queue, struct, logging
import can, numpy as np
from evdev import InputDevice, categorize, ecodes, list_devices

# ------------------ Logging Setup ------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

# ------------------ USER CONFIG ------------------
CONTROL_FREQ = 50                         # Control loop frequency (Hz)
CONTROL_PERIOD = 1.0 / CONTROL_FREQ         # Control loop period (s)

# ODrive node IDs for turning and rolling motors.
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0    # Rolling motors current limit (A)
VEL_LIMIT_ROLLING = 2          # Rolling motor velocity limit (turns/s)

MAX_LINEAR_VEL = 1.0         # Maximum translational speed (turns/s equivalent)
MAX_ANG_VEL    = 3           # Maximum rotational speed (turns/s)

# Parameters for turning (angle) controller (velocity mode)
KP_TURN = 2.0                # Proportional gain for turning angle error
MAX_TURN_VEL = 1.0           # Maximum turning wheel velocity (turns/s)

# Joystick parameters
JOYSTICK_DEADZONE = 0.1  
# Left stick (ABS_X, ABS_Y) for translation, right stick (ABS_RX) for rotation.
MOTOR_CONFIG_FILE = "motor_config.json"

# ------------------ ODrive CAN PROTOCOL ------------------
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
    arbid = to_arbid(node_id, FUNC_SET_INPUT_POS)
    data = struct.pack('<fhh', pos, 0, 0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanOperationError as e:
        logging.error(f"CAN send error (position) for node {node_id}: {e}")

def set_input_vel(bus, node_id, vel):
    arbid = to_arbid(node_id, FUNC_SET_INPUT_VEL)
    data = struct.pack('<ff', vel, 0.0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    try:
        bus.send(msg)
    except can.CanOperationError as e:
        logging.error(f"CAN send error (velocity) for node {node_id}: {e}")

# ------------------ FEEDBACK THREAD ------------------
class ODriveFeedbackCollector(threading.Thread):
    """
    Continuously reads CAN messages to update encoder feedback for all motor nodes.
    """
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.daemon = True
        self.run_flag = True
        # feedback: dictionary {node_id: (position, velocity)}
        self.feedback = {nid: (0.0, 0.0) for nid in ALL_NODE_IDS}
    
    def run(self):
        while self.run_flag:
            msg = self.bus.recv(timeout=0.1)
            if not msg:
                continue
            func = msg.arbitration_id & 0x1F
            node_id = msg.arbitration_id >> 5
            if func == FUNC_ENCODER_EST and node_id in ALL_NODE_IDS:
                pos, vel = struct.unpack('<ff', msg.data)
                self.feedback[node_id] = (pos, vel)
    
    def stop(self):
        self.run_flag = False

# ------------------ MOTOR WRAPPERS ------------------
class ODriveMotor:
    """
    Low-level wrapper for an ODrive motor.
    """
    def __init__(self, bus, node_id, offset=0.0, flip=1.0, is_turning=False):
        self.bus = bus
        self.node_id = node_id
        self.offset = offset  # Motor-specific offset (in turns)
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
        computed_angle = self.offset + self.flip * raw_angle
        set_input_pos(self.bus, self.node_id, computed_angle)
    
    def command_velocity(self, raw_vel):
        computed_vel = (self.flip * raw_vel) if self.is_turning else (self.offset + self.flip * raw_vel)
        set_input_vel(self.bus, self.node_id, computed_vel)
        logging.debug(f"Node {self.node_id} velocity command: {computed_vel:.3f} turns/s")

class SwerveModule:
    """
    Combines one turning and one rolling motor.
    The turning motor is driven via a P controller (velocity mode) toward a desired angle.
    The rolling motor is commanded directly with a speed.
    """
    def __init__(self, turn_motor, roll_motor):
        self.turn_motor = turn_motor
        self.roll_motor = roll_motor
        # Store (desired turning angle, roll speed) for logging.
        self.last_command = (0.0, 0.0)
    
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
    
    def command(self, turn_vel_command, roll_vel, desired_turn_angle):
        # Save the desired turn angle (0 to 1) and roll command for logging.
        self.last_command = (desired_turn_angle, roll_vel)
        self.turn_motor.command_velocity(turn_vel_command)
        self.roll_motor.command_velocity(roll_vel)

# ------------------ HELPER: WHEEL GEOMETRY ------------------
def generate_wheel_positions(num_wheels, robot_size):
    """
    Generate default wheel positions arranged uniformly on a circle.
    Returns a list of np.array([x, y]) positions; circle radius = robot_size / 2.
    """
    R = robot_size / 2.0
    positions = []
    for i in range(num_wheels):
        angle = 2 * math.pi * i / num_wheels
        x = R * math.cos(angle)
        y = R * math.sin(angle)
        positions.append(np.array([x, y]))
    return positions

# ------------------ SWERVE VEHICLE CLASS (MATRIX VERSION) ------------------
class SwerveVehicle:
    """
    Simplified vehicle control using a matrix-based (vectorized) method.
    
    The target velocity is a 3-element vector: [vx, vy, ω] (from joysticks).
    
    For each module:
      - The translational velocity [vx,vy] is combined with the rotational contribution:
            v_rot = ω * [-y_effective, x_effective]
        where effective position = nominal position + caster_offset vector.
      - The total wheel velocity vector is computed for all modules in a matrix operation.
      - The desired turning angle is derived via arctan2 and mapped to [0,1) turns.
      - The current turning feedback (with motor offset added) is also wrapped to [0,1).
      - The error (smallest angular difference) is computed vectorized, and the P controller
        outputs a turning velocity command.
      - Commands are then sent to each module individually.
    
    Live logging outputs the desired turning angle and the real-time (offset-applied) encoder angle.
    """
    def __init__(self, bus, feedback_collector, module_list, robot_size, num_wheels, caster_offset, nominal_wheel_positions=None):
        self.bus = bus
        self.modules = module_list
        self.feedback = feedback_collector
        self.feedback.start()
        self.target_velocity = [0.0, 0.0, 0.0]  # [vx, vy, ω]
        self.run_flag = True
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.status_thread = threading.Thread(target=self.log_status, daemon=True)
        self.robot_size = robot_size
        self.num_wheels = num_wheels
        self.caster_offset = caster_offset
        if nominal_wheel_positions is None:
            self.nominal_wheel_positions = generate_wheel_positions(num_wheels, robot_size)
        else:
            self.nominal_wheel_positions = [np.array(pos) for pos in nominal_wheel_positions]
    
    def start(self):
        self.ctrl_thread.start()
        self.status_thread.start()
    
    def stop(self):
        self.run_flag = False
        self.ctrl_thread.join()
        self.status_thread.join()
        self.feedback.stop()
        for m in self.modules:
            m.set_idle()
    
    def set_target_velocity(self, command):
        """Update target velocity command: [vx, vy, ω]."""
        self.target_velocity = command
    
    def control_loop(self):
        """
        Runs at CONTROL_FREQ Hz.
        Uses matrix operations to compute:
          - effective nominal positions for all modules (with caster offset)
          - the rotational contribution v_rot for each module
          - the total wheel velocity vectors (v_total) for all modules
          - desired turning angles from v_total (mapped to [0, 1))
          - retrieves current turning angles (with motor offset added) into an array
          - computes the error vector and outputs turning velocity commands
        """
        last_t = time.time()
        num = self.num_wheels
        # Precompute nominal positions matrix of shape (n, 2)
        P = np.stack(self.nominal_wheel_positions)  # shape (n,2)
        # Compute module norms
        norms = np.linalg.norm(P, axis=1)
        # Avoid division by zero: if norm is zero, just leave as zero.
        effective_nominal = np.empty_like(P)
        for i in range(num):
            if norms[i] != 0:
                effective_nominal[i,:] = P[i,:] + (P[i,:] / norms[i]) * self.caster_offset
            else:
                effective_nominal[i,:] = P[i,:]
    
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now
            
            vx, vy, omega = self.target_velocity
            logging.debug(f"Target velocity: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f}")
            
            # Compute rotational contribution for all modules: for each module [ -effective_y, effective_x ]
            v_rot = omega * np.column_stack((-effective_nominal[:,1], effective_nominal[:,0]))
            # Total wheel velocity vector for each module: v_total = [vx,vy] + v_rot
            v_total = np.tile(np.array([vx, vy]), (num,1)) + v_rot
            # Rolling speed for each module is the norm of its v_total vector
            roll_speed = np.linalg.norm(v_total, axis=1)
            # Desired turning angle (radians) for each module via arctan2, then map to turns in [0,1)
            desired_angle = np.arctan2(v_total[:,1], v_total[:,0])
            desired_angle_turns = (desired_angle / (2 * math.pi)) % 1.0

            # Retrieve current turning angles from feedback (with motor offset applied)
            current_angles = []
            offsets = []
            for mod in self.modules:
                cur, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0,0.0))
                current_angles.append(cur)
                offsets.append(mod.turn_motor.offset)
            current_angles = np.array(current_angles) % 1.0
            offsets = np.array(offsets) % 1.0
            # Effective current turning angles as seen by the motor:
            effective_current = (current_angles + offsets) % 1.0
            
            # Compute angle error vector (smallest angular difference)
            error = (desired_angle_turns - effective_current) % 1.0
            error = np.where(error > 0.5, error - 1.0, error)
            # Compute turning velocity commands via P controller and clamp them.
            turn_vel_command = np.clip(KP_TURN * error, -MAX_TURN_VEL, MAX_TURN_VEL)
            
            # Send commands to each module.
            for i, mod in enumerate(self.modules):
                mod.command(turn_vel_command[i], roll_speed[i], desired_angle_turns[i])
            logging.debug(f"Matrix Computation: desired angles={desired_angle_turns}, effective current={effective_current}, error={error}")
    
    def log_status(self):
        """
        Every second, log for each module:
          - Desired turning angle (in turns, [0,1))
          - Roll command (speed)
          - Real-time encoder turning angle with offset applied (in turns)
          - Rolling feedback.
        """
        while self.run_flag:
            time.sleep(1.0)
            for i, mod in enumerate(self.modules):
                desired_turn_angle, roll_cmd = mod.last_command
                turn_fb, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                turn_fb_with_offset = (turn_fb + mod.turn_motor.offset) % 1.0
                roll_fb, _ = self.feedback.feedback.get(mod.roll_motor.node_id, (0.0, 0.0))
                logging.info(f"Module {i}: Desired Turn Angle = {desired_turn_angle:.3f} turns, "
                             f"Roll Cmd = {roll_cmd:.3f} turns/s, "
                             f"Turn FB (with offset) = {turn_fb_with_offset:.3f} turns, "
                             f"Roll FB = {roll_fb:.3f} turns")

# ------------------ JOYSTICK INPUT THREAD ------------------
def joystick_input_thread(vehicle, joystick):
    """
    Reads joystick events to update the target velocity command.
    
    Mapping:
      - Left joystick controls translation:
          * ABS_X: lateral (vy)
          * ABS_Y: forward/back (vx) [inverted so pushing forward yields positive vx]
      - Right joystick horizontal (ABS_RX) controls rotation (ω).
      - BTN_SOUTH clears errors on all modules.
    
    Translation [vx, vy] is scaled by MAX_LINEAR_VEL and rotation by MAX_ANG_VEL.
    """
    # Retrieve left stick properties for translation.
    try:
        absinfo_left_x = joystick.absinfo(ecodes.ABS_X)
        absinfo_left_y = joystick.absinfo(ecodes.ABS_Y)
        center_left_x = (absinfo_left_x.min + absinfo_left_x.max) / 2
        center_left_y = (absinfo_left_y.min + absinfo_left_y.max) / 2
        range_left_x = (absinfo_left_x.max - center_left_x)
        range_left_y = (absinfo_left_y.max - center_left_y)
        logging.info(f"Joystick ABS_X: min={absinfo_left_x.min}, max={absinfo_left_x.max}, center={center_left_x}")
        logging.info(f"Joystick ABS_Y: min={absinfo_left_y.min}, max={absinfo_left_y.max}, center={center_left_y}")
    except Exception as e:
        logging.error("Left joystick axes not available.")
        center_left_x = center_left_y = 0.0
        range_left_x = range_left_y = 1.0

    # Retrieve right stick property for rotation.
    try:
        absinfo_rx = joystick.absinfo(ecodes.ABS_RX)
        center_rx = (absinfo_rx.min + absinfo_rx.max) / 2
        range_rx = (absinfo_rx.max - center_rx)
        logging.info(f"Joystick ABS_RX: min={absinfo_rx.min}, max={absinfo_rx.max}, center={center_rx}")
    except Exception as e:
        logging.error("Right joystick ABS_RX not available; rotation control may not work.")
        center_rx = 0.0
        range_rx = 1.0

    left_x = left_y = right_x = 0.0
    
    for event in joystick.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:
                left_x = (event.value - center_left_x) / range_left_x
                if abs(left_x) < JOYSTICK_DEADZONE:
                    left_x = 0.0
            elif event.code == ecodes.ABS_Y:
                left_y = (center_left_y - event.value) / range_left_y
                if abs(left_y) < JOYSTICK_DEADZONE:
                    left_y = 0.0
            elif event.code == ecodes.ABS_RX:
                right_x = (event.value - center_rx) / range_rx
                if abs(right_x) < JOYSTICK_DEADZONE:
                    right_x = 0.0
            
            # Compute translational commands from left stick.
            vx = left_y * MAX_LINEAR_VEL
            vy = left_x * MAX_LINEAR_VEL
            # Compute rotational command from right stick.
            omega = right_x * MAX_ANG_VEL
            
            vehicle.set_target_velocity([vx, vy, omega])
            time.sleep(0.005)
        
        elif event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_SOUTH and event.value == 1:
                logging.info("Clear errors button pressed; clearing errors on all modules.")
                for mod in vehicle.modules:
                    mod.clear_errors()

# ------------------ MAIN SCRIPT ------------------
def main():
    if not os.path.exists(MOTOR_CONFIG_FILE):
        logging.error(f"Missing {MOTOR_CONFIG_FILE}. Please create it.")
        sys.exit(1)
    with open(MOTOR_CONFIG_FILE, 'r') as f:
        mcfg = json.load(f)
    
    bus = can.interface.Bus("can0", interface="socketcan")
    
    # Initialize turning and rolling motors.
    turning_motors = []
    rolling_motors = []
    for nid in TURNING_NODE_IDS:
        offset = mcfg.get(str(nid), {}).get("offset", 0.0)
        flip = mcfg.get(str(nid), {}).get("flip", 1.0)
        m = ODriveMotor(bus, nid, offset, flip, is_turning=True)
        turning_motors.append(m)
    for nid in ROLLING_NODE_IDS:
        offset = mcfg.get(str(nid), {}).get("offset", 0.0)
        flip = mcfg.get(str(nid), {}).get("flip", 1.0)
        m = ODriveMotor(bus, nid, offset, flip, is_turning=False)
        logging.info(f"Motor {nid}: Flip = {flip}")
        rolling_motors.append(m)
    
    # Create swerve modules.
    modules = []
    for i in range(len(turning_motors)):
        modules.append(SwerveModule(turning_motors[i], rolling_motors[i]))
    
    # Clear errors and initialize motors.
    for mod in modules:
        mod.clear_errors()
        time.sleep(0.05)
    time.sleep(0.2)
    for mod in modules:
        mod.set_closed_loop()
        time.sleep(0.05)
        mod.set_rolling_limits(VEL_LIMIT_ROLLING, CURRENT_LIMIT_ROLLING)
    
    fb_collector = ODriveFeedbackCollector(bus)
    
    from evdev import list_devices, InputDevice
    devices = [InputDevice(path) for path in list_devices()]
    joystick = None
    for dev in devices:
        if "Wireless Controller" in dev.name:
            joystick = dev
            break
    if joystick is None:
        logging.error("No joystick detected! Exiting.")
        sys.exit(1)
    logging.info(f"Joystick detected: {joystick.name}")
    
    try:
        absinfo_x = joystick.absinfo(ecodes.ABS_X)
        logging.info(f"Joystick ABS_X: min={absinfo_x.min}, max={absinfo_x.max}")
    except Exception:
        pass
    try:
        absinfo_rx = joystick.absinfo(ecodes.ABS_RX)
        logging.info(f"Joystick ABS_RX: min={absinfo_rx.min}, max={absinfo_rx.max}")
    except Exception:
        pass
    
    robot_size = 0.350      # Characteristic dimension (adjust as needed)
    num_wheels = 3
    caster_offset = 0.015   # Caster offset (adjust as needed)
    
    if "wheel_positions" in mcfg:
        nominal_wheel_positions = [np.array(pos) for pos in mcfg["wheel_positions"]]
        logging.info("Using configured wheel positions from motor_config.json")
    else:
        nominal_wheel_positions = None
    
    vehicle = SwerveVehicle(bus, fb_collector, modules, robot_size, num_wheels, caster_offset, nominal_wheel_positions)
    vehicle.start()
    
    js_thread = threading.Thread(target=joystick_input_thread, args=(vehicle, joystick), daemon=True)
    js_thread.start()
    
    try:
        logging.info("Control loop started. Use your joystick to drive the robot.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logging.info("CTRL+C detected, stopping vehicle.")
    finally:
        vehicle.stop()
        logging.info("All done.")

if __name__ == "__main__":
    main()
