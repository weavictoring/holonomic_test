#!/usr/bin/env python3
"""
3-Wheel Holonomic Base with Powered Caster Drive Using Joystick Control (via evdev)

This script:
  1) Loads motor_config.json for each ODrive node (including flip, offset, and optionally wheel_positions).
  2) Spawns a background thread to parse ODrive encoder feedback.
  3) Sets rolling motors to a 5A current limit.
  4) Uses a simplified control loop (without Ruckig) that reads joystick inputs directly.
  5) Uses the left joystick's vertical axis for forward/backward (rolling) speed 
     and the right joystick's horizontal axis for the desired turning angle.
  6) Implements a proportional controller for the turning motors (in velocity mode) that drives them toward
     the joystick-specified angle.
  7) Commands rolling motors with the joystick-set translation speed.
  8) Runs the control loop at ~50 Hz and logs live status (commands and feedback) at 1 Hz.
  
Test carefully at low current & speed with the robot wheels off the ground!
"""

import os, sys, math, json, time, queue, struct, threading, logging
import can, numpy as np
from evdev import InputDevice, categorize, ecodes, list_devices

# ------------------ Logging Setup ------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

# ------------------ USER CONFIG ------------------
CONTROL_FREQ = 50                        # Control loop frequency (Hz)
CONTROL_PERIOD = 1.0 / CONTROL_FREQ        # Control loop period (s)

# ODrive node IDs for turning and rolling motors
TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0    # Rolling motors current limit (A)
VEL_LIMIT_ROLLING = 2          # Rolling motor velocity limit (turns/s)

# Maximum rolling (translation) speed (turns/s)
MAX_LINEAR_VEL = 1.0  
# Maximum turning wheel velocity (turns/s) used by the P controller
MAX_TURN_VEL = 1.0     
# Proportional gain for the turning angle controller
KP_TURN = 2.0          

# Joystick parameters
JOYSTICK_DEADZONE = 0.1  
# For the turning angle command: we will map the right joystick from [-1,1] to [0,1] turns.
# For translation, the left joystick's vertical axis is scaled by MAX_LINEAR_VEL.

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
    This thread continuously reads CAN messages to update the encoder feedback
    for each motor node.
    """
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.daemon = True
        self.run_flag = True
        # Initialize feedback dictionary for all nodes: {node_id: (position, velocity)}
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
        # For turning motors, we do not add the offset.
        if self.is_turning:
            computed_vel = self.flip * raw_vel
        else:
            computed_vel = self.offset + self.flip * raw_vel
        set_input_vel(self.bus, self.node_id, computed_vel)
        logging.debug(f"Node {self.node_id} velocity command: {computed_vel:.3f} turns/s")

class SwerveModule:
    """
    Combines one turning and one rolling motor.
    The turning motor is controlled in velocity mode via a P controller,
    and the rolling motor is commanded with a direct speed.
    """
    def __init__(self, turn_motor, roll_motor):
        self.turn_motor = turn_motor
        self.roll_motor = roll_motor
        # Store last command for live status logging.
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
    
    def command(self, turn_vel_command, roll_vel):
        # Save commands for logging.
        self.last_command = (turn_vel_command, roll_vel)
        # Command turning motor in velocity mode.
        self.turn_motor.command_velocity(turn_vel_command)
        # Command rolling motor in velocity mode.
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
    Simplified vehicle control that directly uses joystick commands.
    
    - translation_command: commanded rolling (translation) speed (turns/s)
    - turning_command: desired turning angle in turns (0 to 1; where 1 turn = 360°)
    
    A proportional controller drives each turning motor toward the desired angle.
    A live status logger prints the last commands and motor feedback.
    """
    def __init__(self, bus, feedback_collector, module_list, robot_size, num_wheels, caster_offset, nominal_wheel_positions=None):
        self.bus = bus
        self.modules = module_list
        self.feedback = feedback_collector
        self.feedback.start()
        # Initialize commands
        self.translation_command = 0.0  # Rolling speed (turns/s)
        self.turning_command = 0.5      # Desired turning angle (in turns, default 0.5 = 180°)
        self.run_flag = True
        # Start control loop and status logger threads
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
    
    def set_target_command(self, translation, turning_angle):
        """
        Update target commands:
          translation: desired rolling speed (turns/s) from left joystick.
          turning_angle: desired turning angle in turns (0 to 1) from right joystick.
        """
        self.translation_command = translation
        self.turning_command = turning_angle
    
    def control_loop(self):
        """
        Main control loop at CONTROL_FREQ Hz.
        For each module:
          - Retrieve current turning angle feedback.
          - Compute error (with wrapping over 1 turn).
          - Compute turning velocity command via a proportional controller.
          - Command the module with the turning velocity and rolling speed.
        """
        last_t = time.time()
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now
            
            for i, mod in enumerate(self.modules):
                # Get current turning angle (in turns) from encoder feedback.
                current_angle, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                # Compute error modulo 1 turn.
                error = (self.turning_command - current_angle) % 1.0
                if error > 0.5:
                    error -= 1.0
                # Compute turning velocity command using a proportional controller.
                turn_vel_command = KP_TURN * error
                # Clamp turning velocity to ±MAX_TURN_VEL.
                turn_vel_command = max(min(turn_vel_command, MAX_TURN_VEL), -MAX_TURN_VEL)
                
                # Command the module: turning and rolling speeds.
                mod.command(turn_vel_command, self.translation_command)
                logging.debug(f"Module {i}: Desired angle = {self.turning_command:.3f} turns, "
                              f"Current angle = {current_angle:.3f} turns, Error = {error:.3f}, "
                              f"Turn Vel Cmd = {turn_vel_command:.3f} turns/s, Roll Cmd = {self.translation_command:.3f} turns/s")
    
    def log_status(self):
        """
        Every second, log the last commands sent and the current encoder feedback.
        """
        while self.run_flag:
            time.sleep(1.0)
            for i, mod in enumerate(self.modules):
                turn_cmd, roll_cmd = mod.last_command
                turn_fb, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                roll_fb, _ = self.feedback.feedback.get(mod.roll_motor.node_id, (0.0, 0.0))
                logging.info(f"Module {i}: Turn Cmd = {turn_cmd:.3f} turns/s, Roll Cmd = {roll_cmd:.3f} turns/s, "
                             f"Turn FB = {turn_fb:.3f} turns, Roll FB = {roll_fb:.3f} turns")

# ------------------ JOYSTICK INPUT THREAD ------------------
def joystick_input_thread(vehicle, joystick):
    """
    Reads joystick events to update the target commands.
      - Left joystick vertical axis (ABS_Y) controls rolling speed.
      - Right joystick horizontal axis (ABS_RX) controls desired turning angle.
      - BTN_SOUTH clears errors.
    
    For the left joystick (translation):
      * Forward (pushed) gives a positive translation command.
    For the right joystick (turning):
      * The value is normalized from [-1, 1] and then mapped to [0, 1] turns.
    """
    # Get properties for left joystick (translation)
    try:
        absinfo_left_y = joystick.absinfo(ecodes.ABS_Y)
        center_left_y = (absinfo_left_y.min + absinfo_left_y.max) / 2
        range_left_y = (absinfo_left_y.max - center_left_y)
        logging.info(f"Joystick ABS_Y: min={absinfo_left_y.min}, max={absinfo_left_y.max}, center={center_left_y}")
    except Exception as e:
        logging.error("ABS_Y not available on this joystick.")
        center_left_y = 0.0
        range_left_y = 1.0

    # Get properties for right joystick (turning)
    try:
        absinfo_rx = joystick.absinfo(ecodes.ABS_RX)
        center_rx = (absinfo_rx.min + absinfo_rx.max) / 2
        range_rx = (absinfo_rx.max - center_rx)
        logging.info(f"Joystick ABS_RX: min={absinfo_rx.min}, max={absinfo_rx.max}, center={center_rx}")
    except Exception as e:
        logging.error("ABS_RX not available on this joystick; turning control may not work.")
        center_rx = 0.0
        range_rx = 1.0

    translation_cmd = 0.0
    turning_cmd = 0.5  # Default turning angle (0.5 turns = 180°)
    
    for event in joystick.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_Y:
                # Left joystick vertical: invert so pushing forward is positive.
                norm_y = (center_left_y - event.value) / range_left_y
                # Apply deadzone and scale by MAX_LINEAR_VEL.
                if abs(norm_y) < JOYSTICK_DEADZONE:
                    norm_y = 0.0
                translation_cmd = norm_y * MAX_LINEAR_VEL
            elif event.code == ecodes.ABS_RX:
                # Right joystick horizontal: normalize to [-1,1] then map to [0,1] turns.
                norm_rx = (event.value - center_rx) / range_rx
                if abs(norm_rx) < JOYSTICK_DEADZONE:
                    norm_rx = 0.0
                # Map [-1,1] to [0,1]: -1 -> 0, 0 -> 0.5, +1 -> 1.0.
                turning_cmd = (norm_rx + 1) / 2.0
            
            # Update vehicle with new commands.
            vehicle.set_target_command(translation_cmd, turning_cmd)
            time.sleep(0.005)
        
        elif event.type == ecodes.EV_KEY:
            # Clear errors when BTN_SOUTH is pressed.
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
    
    # Create turning and rolling motor objects based on node IDs.
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
    
    # Combine motors into swerve modules.
    modules = []
    for i in range(len(turning_motors)):
        modules.append(SwerveModule(turning_motors[i], rolling_motors[i]))
    
    # Clear errors and set motors to closed-loop mode.
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
    
    # Log some joystick axes info.
    try:
        absinfo_y = joystick.absinfo(ecodes.ABS_Y)
        logging.info(f"Joystick ABS_Y: min={absinfo_y.min}, max={absinfo_y.max}")
    except Exception:
        pass
    try:
        absinfo_rx = joystick.absinfo(ecodes.ABS_RX)
        logging.info(f"Joystick ABS_RX: min={absinfo_rx.min}, max={absinfo_rx.max}")
    except Exception:
        pass
    
    robot_size = 0.350   # Characteristic dimension (adjust as needed)
    num_wheels = 3
    caster_offset = 0.015  # Caster offset (adjust as needed)
    
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
