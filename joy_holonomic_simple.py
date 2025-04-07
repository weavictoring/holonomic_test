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
     It factors in a caster offset for each wheel.
  6) Uses a proportional controller for the turning motors (driven in velocity mode) that drives them toward the desired turning angle.
  7) Logs live status (commands and feedback), where the turn command is output as the desired angle in the range 0–1.
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

# ODrive node IDs for turning and rolling motors
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0    # Rolling motors current limit (A)
VEL_LIMIT_ROLLING = 2          # Rolling motor velocity limit (turns/s)

MAX_LINEAR_VEL = 1.0         # Maximum translational speed (turns/s equivalent)
MAX_ANG_VEL    = 3           # Maximum rotational speed (turns/s)

# Parameters for turning wheel position controller (velocity mode)
KP_TURN = 2.0                # Proportional gain for turning angle error
MAX_TURN_VEL = 1.0           # Maximum turning wheel velocity (turns/s)

# Joystick parameters
JOYSTICK_DEADZONE = 0.1  
# Note: Translation is controlled via the left stick (ABS_X, ABS_Y) and rotation via right stick (ABS_RX)

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
    Low-level wrapper for a motor on an ODrive node.
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
        # Not used for turning motors in this scheme.
        computed_angle = self.offset + self.flip * raw_angle
        set_input_pos(self.bus, self.node_id, computed_angle)
    
    def command_velocity(self, raw_vel):
        # For turning motors, do not add the offset.
        computed_vel = self.flip * raw_vel if self.is_turning else self.offset + self.flip * raw_vel
        set_input_vel(self.bus, self.node_id, computed_vel)
        logging.debug(f"Node {self.node_id} velocity command: {computed_vel:.3f} turns/s")

class SwerveModule:
    """
    Combines one turning and one rolling motor.
    The turning motor is driven in velocity mode by a P controller toward a desired angle.
    The rolling motor is commanded with a direct speed.
    """
    def __init__(self, turn_motor, roll_motor):
        self.turn_motor = turn_motor
        self.roll_motor = roll_motor
        # last_command stores (desired_turn_angle, roll_speed) for logging
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
        # Save desired turn angle (in turns, mapped to [0,1)) and roll speed for live logging.
        self.last_command = (desired_turn_angle, roll_vel)
        # Command the turning motor (velocity mode) and rolling motor.
        self.turn_motor.command_velocity(turn_vel_command)
        self.roll_motor.command_velocity(roll_vel)

# ------------------ HELPER: WHEEL GEOMETRY ------------------
def generate_wheel_positions(num_wheels, robot_size):
    """
    Generate default wheel positions arranged uniformly on a circle.
    Uses a circle of radius = robot_size / 2.
    """
    R = robot_size / 2.0
    positions = []
    for i in range(num_wheels):
        angle = 2 * math.pi * i / num_wheels
        x = R * math.cos(angle)
        y = R * math.sin(angle)
        positions.append(np.array([x, y]))
    return positions

# ------------------ SWERVE VEHICLE CLASS ------------------
class SwerveVehicle:
    """
    Simplified vehicle control using the vector method.
    
    Target velocity is a 3-element vector: [vx, vy, ω], where:
      vx, vy are translational speeds (from left stick),
      ω is the rotational speed (from right stick).
    
    For each module:
      - The rotation contribution is computed based on its nominal position (plus caster offset).
      - The total wheel velocity vector is: v_total = [vx, vy] + ω * [-y_effective, x_effective]
      - The desired turning angle is derived from v_total using atan2 and then mapped to [0, 1)
        (i.e. 1 turn = 360°).
      - The current feedback from the turning motor is also wrapped into [0, 1).
      - A P-controller drives the turning motor velocity based on the difference between the desired and current angles.
    
    Live logging outputs the desired turn angle and roll command.
    """
    def __init__(self, bus, feedback_collector, module_list, robot_size, num_wheels, caster_offset, nominal_wheel_positions=None):
        self.bus = bus
        self.modules = module_list
        self.feedback = feedback_collector
        self.feedback.start()
        # Target velocity: [vx, vy, ω]. Initialize to zeros.
        self.target_velocity = [0.0, 0.0, 0.0]
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
        """
        Update target velocity command (list: [vx, vy, ω]).
        """
        self.target_velocity = command
    
    def control_loop(self):
        """
        Runs at CONTROL_FREQ Hz.
        For each module, compute:
          - The effective wheel velocity vector combining translation and rotation.
          - The desired turning angle (in turns) is computed using atan2 and then wrapped to [0,1).
          - The current turning angle is wrapped into [0,1).
          - Error is computed as the smallest angular difference and fed into a P controller.
          - Commands are sent to the module.
        """
        last_t = time.time()
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now
            
            vx, vy, omega = self.target_velocity
            logging.debug(f"Target velocity: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f}")
            
            for i, mod in enumerate(self.modules):
                # Calculate effective module position (adding caster offset)
                nominal = self.nominal_wheel_positions[i]
                norm = np.linalg.norm(nominal)
                offset_vec = (nominal / norm) * self.caster_offset if norm != 0 else np.array([0.0, 0.0])
                effective_nominal = nominal + offset_vec
                
                # Compute rotational component for this module.
                v_rot = omega * np.array([-effective_nominal[1], effective_nominal[0]])
                # Total wheel velocity vector is the sum of translational and rotational components.
                v_total = np.array([vx, vy]) + v_rot
                roll_speed = np.linalg.norm(v_total)
                # Compute desired turning angle (radians) using atan2.
                desired_angle = math.atan2(v_total[1], v_total[0])
                # Map desired_angle to turns in the range [0,1) using modulo.
                desired_angle_turns = (desired_angle / (2 * math.pi)) % 1.0
                
                # Get current turning angle from feedback and wrap into [0,1).
                current_angle, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                current_angle = current_angle % 1.0
                
                # Compute angle error (smallest angular difference).
                error = (desired_angle_turns - current_angle) % 1.0
                if error > 0.5:
                    error -= 1.0
                
                # Proportional controller computes turning velocity command.
                turn_vel_command = KP_TURN * error
                # Clamp turning velocity to ±MAX_TURN_VEL.
                turn_vel_command = max(min(turn_vel_command, MAX_TURN_VEL), -MAX_TURN_VEL)
                
                # Command the module.
                mod.command(turn_vel_command, roll_speed, desired_angle_turns)
                logging.debug(f"Module {i}: desired angle={desired_angle_turns:.3f} turns, current angle={current_angle:.3f} turns, "
                              f"error={error:.3f}, turn vel cmd={turn_vel_command:.3f} turns/s, roll speed={roll_speed:.3f} turns/s")
    
    def log_status(self):
        """
        Logs live status every 1 second:
          - The desired turning angle (in turns) and rolling command.
          - The current encoder feedback for turning and rolling motors.
        """
        while self.run_flag:
            time.sleep(1.0)
            for i, mod in enumerate(self.modules):
                desired_turn_angle, roll_cmd = mod.last_command
                turn_fb, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                roll_fb, _ = self.feedback.feedback.get(mod.roll_motor.node_id, (0.0, 0.0))
                logging.info(f"Module {i}: Desired Turn Angle = {desired_turn_angle:.3f} turns, Roll Cmd = {roll_cmd:.3f} turns/s, "
                             f"Turn FB = {turn_fb:.3f} turns, Roll FB = {roll_fb:.3f} turns")
                
# ------------------ JOYSTICK INPUT THREAD ------------------
def joystick_input_thread(vehicle, joystick):
    """
    Reads joystick events to update the target velocity command.
    
    Mapping:
      - Left joystick controls translation:
          * ABS_X: lateral (vy)
          * ABS_Y: forward/back (vx) [inverted so pushing forward yields positive vx]
      - Right joystick horizontal (ABS_RX) controls rotation (ω).
      - BTN_SOUTH clears errors.
    
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
                # Invert so that pushing forward (lower value) yields positive forward velocity.
                left_y = (center_left_y - event.value) / range_left_y
                if abs(left_y) < JOYSTICK_DEADZONE:
                    left_y = 0.0
            elif event.code == ecodes.ABS_RX:
                right_x = (event.value - center_rx) / range_rx
                if abs(right_x) < JOYSTICK_DEADZONE:
                    right_x = 0.0
            
            # Compute translational commands from left stick.
            # vx: forward/back speed; vy: lateral speed.
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
        flip   = mcfg.get(str(nid), {}).get("flip", 1.0)
        m = ODriveMotor(bus, nid, offset, flip, is_turning=True)
        turning_motors.append(m)
    for nid in ROLLING_NODE_IDS:
        offset = mcfg.get(str(nid), {}).get("offset", 0.0)
        flip   = mcfg.get(str(nid), {}).get("flip", 1.0)
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
    
    # Log some joystick axis info.
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
