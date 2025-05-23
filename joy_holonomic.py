#!/usr/bin/env python3
"""
3-Wheel Holonomic Base with Powered Caster Drive Using Joystick Control (via evdev)

This script:
  1) Loads motor_config.json for each ODrive node (including flip, offset, and optionally wheel_positions).
  2) Spawns a background thread to parse ODrive encoder feedback.
  3) Sets rolling motors to a 5A current limit.
  4) Uses Ruckig for advanced command smoothing in 3 DOFs (vx, vy, ω).
  5) Computes each module’s desired turning angle and rolling velocity from the 
     smoothed joystick commands, factoring in a caster offset.
  6) Commands turning motors in velocity mode using a position controller that computes 
     a velocity command based on the error between the desired and current angle,
     and rolling motors with:   offset + flip * desiredVelocity (in turns/s).
  7) Runs a control loop at ~50 Hz.
  8) Logs real-time output and errors, including a live status update of commands and positions.

Joystick input is read via the evdev library.
Test carefully at low current & speed with the robot wheels off the ground!
"""

import os, sys, math, json, time, queue, struct, threading, logging
import can, ruckig, numpy as np
from evdev import InputDevice, categorize, ecodes, list_devices

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)

# ========= USER CONFIG =========
CONTROL_FREQ = 50
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0    # 5 A current limit for rolling motors
VEL_LIMIT_ROLLING = 2          # Example velocity limit in turns/s

MAX_LINEAR_VEL = 1.0   # Maximum forward velocity (in turns/s)
MAX_ANG_VEL    = 3     # Maximum angular velocity (in turns/s)

# New parameters for turning wheel position controller (velocity mode)
KP_TURN = 2.0          # Proportional gain for turning error
MAX_TURN_VEL = 1.0     # Maximum turning wheel velocity (in turns/s)

# Joystick parameters
JOYSTICK_DEADZONE = 0.1
JOYSTICK_SCALE_W = MAX_ANG_VEL     

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

# ========= BACKGROUND FEEDBACK THREAD =========
class ODriveFeedbackCollector(threading.Thread):
    def __init__(self, bus):
        super().__init__()
        self.bus = bus
        self.daemon = True
        self.run_flag = True
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
            if func == FUNC_ENCODER_EST and node_id in ALL_NODE_IDS:
                pos, vel = struct.unpack('<ff', msg.data)
                self.feedback[node_id] = (pos, vel)

    def stop(self):
        self.run_flag = False

# ========= MOTOR WRAPPERS =========
class ODriveMotor:
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
        # Not used for turning motors in the new scheme
        logging.debug(f"Node {self.node_id} position command: {raw_angle:.3f} turns")
        computed_angle = self.offset + self.flip * raw_angle
        set_input_pos(self.bus, self.node_id, computed_angle)

    def command_velocity(self, raw_vel):
        # For turning motors, do not add the offset.
        if self.is_turning:
            computed_vel = self.flip * raw_vel
        else:
            computed_vel = self.offset + self.flip * raw_vel
        logging.debug(f"Node {self.node_id} velocity command: {computed_vel:.3f} turns/s")
        set_input_vel(self.bus, self.node_id, computed_vel)

class SwerveModule:
    def __init__(self, turn_motor, roll_motor):
        self.turn_motor = turn_motor
        self.roll_motor = roll_motor
        # Store the last command sent for logging
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

    def command(self, turn_vel_command, roll_vel_in_turns_s):
        # Store the command for live status logging
        self.last_command = (turn_vel_command, roll_vel_in_turns_s)
        # Command turning motor in velocity mode using the computed velocity command.
        self.turn_motor.command_velocity(turn_vel_command)
        self.roll_motor.command_velocity(roll_vel_in_turns_s)

# ========= Helper: Generate Wheel Geometry =========
def generate_wheel_positions(num_wheels, robot_size):
    """
    Generate default wheel positions arranged uniformly on a circle.
    
    Parameters:
      num_wheels (int): Number of wheels.
      robot_size (float): A characteristic dimension of the robot (e.g. the diameter).
    
    Returns:
      List of np.array([x, y]) positions (in the robot's coordinate frame) for each wheel.
      Here we use a circle of radius = robot_size / 2.
    """
    R = robot_size / 2.0
    positions = []
    for i in range(num_wheels):
        angle = 2 * math.pi * i / num_wheels  # Uniformly distributed angles
        x = R * math.cos(angle)
        y = R * math.sin(angle)
        positions.append(np.array([x, y]))
    return positions

# ========= VEHICLE CLASS WITH JOYSTICK (evdev) & RUCKIG CONTROL =========
class SwerveVehicle:
    """
    Advanced control loop using Ruckig for 3 DOFs (vx, vy, ω). 
    Joystick commands (from evdev) are fed into a command queue.
    Inverse kinematics for a powered caster design is applied: each module’s effective 
    wheel velocity is computed as:
         v_module = [vx, vy] + ω * [-y_effective, x_effective]
    where (x_effective,y_effective) = nominal_position + caster_offset_vector.
    
    Turning motors are now controlled in velocity mode. A position controller in the main
    control loop computes a velocity command for each turning motor based on the error between
    the desired turning angle and the actual turning angle from feedback.
    """
    def __init__(self, bus, feedback_collector, module_list, robot_size, num_wheels, caster_offset, nominal_wheel_positions=None):
        self.bus = bus
        self.modules = module_list
        self.feedback = feedback_collector
        self.feedback.start()
        self.dofs = 3
        self.rk = ruckig.Ruckig(self.dofs, CONTROL_PERIOD)
        self.inp = ruckig.InputParameter(self.dofs)
        self.out = ruckig.OutputParameter(self.dofs)
        self.inp.max_velocity = [MAX_LINEAR_VEL, MAX_LINEAR_VEL, MAX_ANG_VEL]
        self.inp.max_acceleration = [0.5*MAX_LINEAR_VEL, 0.5*MAX_LINEAR_VEL, 0.5*MAX_ANG_VEL]
        self.inp.current_position = [0.0, 0.0, 0.0]
        self.inp.current_velocity = [0.0, 0.0, 0.0]
        self.inp.control_interface = ruckig.ControlInterface.Velocity
        self.inp.target_velocity = [0.0, 0.0, 0.0]
        self.cmd_queue = queue.Queue(maxsize=1)
        self.run_flag = True
        self.ctrl_thread = threading.Thread(target=self.control_loop, daemon=True)
        # Start a new thread for live status logging.
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
        """Command is a list: [vx, vy, ω]."""
        # Flush any pending command so we always process the latest command
        while not self.cmd_queue.empty():
            try:
                self.cmd_queue.get_nowait()
            except queue.Empty:
                break
        self.cmd_queue.put({"type": "vel", "value": command})

    def control_loop(self):
        last_t = time.time()
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now

            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get_nowait()
                if cmd["type"] == "vel":
                    self.inp.target_velocity = cmd["value"]
                    logging.debug(f"New target velocity: {self.inp.target_velocity}")

            self.rk.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            vx, vy, omega = self.out.new_velocity
            logging.debug(f"Smoothed velocity: vx={vx:.3f}, vy={vy:.3f}, ω={omega:.3f}")

            # For each module, compute the effective command.
            for i, mod in enumerate(self.modules):
                nominal = self.nominal_wheel_positions[i]
                norm = np.linalg.norm(nominal)
                offset_vec = (nominal / norm) * self.caster_offset if norm != 0 else np.array([0.0, 0.0])
                effective_nominal = nominal + offset_vec
                v_rot = omega * np.array([-effective_nominal[1], effective_nominal[0]])
                v_total = np.array([vx, vy]) + v_rot
                speed = np.linalg.norm(v_total)
                desired_angle = math.atan2(v_total[1], v_total[0])
                desired_angle_turns = desired_angle / (2 * math.pi)
                
                # Get current turning angle from feedback for this module's turning motor.
                current_angle_turns, _ = self.feedback.feedback[mod.turn_motor.node_id]
                # Compute error using a modulo approach so that we choose the positive rotation
                error = (desired_angle_turns - current_angle_turns) % 1.0
                if error > 0.5:
                    error -= 1.0
                # Compute turning velocity command using a proportional controller
                turn_vel_command = KP_TURN * error
                # Clamp the turning velocity command to ±MAX_TURN_VEL
                turn_vel_command = max(min(turn_vel_command, MAX_TURN_VEL), -MAX_TURN_VEL)
                
                mod.command(turn_vel_command, speed)
                logging.debug(f"Module {i}: desired angle={desired_angle_turns:.3f} turns, error={error:.3f}, "
                              f"turn_vel_command={turn_vel_command:.3f} turns/s, roll speed={speed:.3f} turns/s")

    def log_status(self):
        """Live log of last commands and motor positions every 1 second."""
        while self.run_flag:
            time.sleep(1.0)
            for i, mod in enumerate(self.modules):
                # Retrieve last commanded values
                turn_cmd, roll_cmd = mod.last_command
                # Get the latest feedback (position) for both turning and rolling motors
                turn_fb, _ = self.feedback.feedback.get(mod.turn_motor.node_id, (0.0, 0.0))
                roll_fb, _ = self.feedback.feedback.get(mod.roll_motor.node_id, (0.0, 0.0))
                logging.info(f"Module {i}: Turn Cmd = {turn_cmd:.3f} turns/s, Roll Cmd = {roll_cmd:.3f} turns/s, "
                             f"Turn FB = {turn_fb:.3f} turns, Roll FB = {roll_fb:.3f} turns")

# ========= JOYSTICK INPUT THREAD (using evdev) =========
def joystick_input_thread(vehicle, joystick):
    # Get properties for the right joystick's horizontal axis (rotation)
    try:
        absinfo_rx = joystick.absinfo(ecodes.ABS_RX)
        center_rx = (absinfo_rx.min + absinfo_rx.max) / 2
        range_rx = (absinfo_rx.max - center_rx)
        logging.info(f"Joystick ABS_RX: min={absinfo_rx.min}, max={absinfo_rx.max}, center={center_rx}")
    except Exception as e:
        logging.error("ABS_RX not available on this joystick; rotation control may not work.")
        center_rx = 0.0
        range_rx = 1.0

    # For D-pad translation, we expect discrete values on ABS_HAT0X and ABS_HAT0Y.
    try:
        absinfo_hat0x = joystick.absinfo(ecodes.ABS_HAT0X)
        absinfo_hat0y = joystick.absinfo(ecodes.ABS_HAT0Y)
        logging.info(f"Joystick ABS_HAT0X: min={absinfo_hat0x.min}, max={absinfo_hat0x.max}")
        logging.info(f"Joystick ABS_HAT0Y: min={absinfo_hat0y.min}, max={absinfo_hat0y.max}")
    except Exception as e:
        logging.warning("D-pad (ABS_HAT0X/ABS_HAT0Y) info not available on this device.")

    # Initialize state variables.
    dpad_x = 0  # Left/right for lateral translation (vy)
    dpad_y = 0  # Up/down for forward/backward (vx)
    rotation_val = 0.0  # Normalized right joystick value for rotation

    for event in joystick.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_HAT0X:
                dpad_x = event.value  # Typically -1 (left), 0 (center), or 1 (right)
            elif event.code == ecodes.ABS_HAT0Y:
                dpad_y = event.value  # Typically -1 (up) or 1 (down)
            elif event.code == ecodes.ABS_RX:
                norm_rx = (event.value - center_rx) / range_rx
                if abs(norm_rx) < JOYSTICK_DEADZONE:
                    norm_rx = 0.0
                rotation_val = norm_rx

            target_vx = (-dpad_y) * MAX_LINEAR_VEL
            target_vy = dpad_x * MAX_LINEAR_VEL
            target_omega = rotation_val * JOYSTICK_SCALE_W

            vehicle.set_target_velocity([target_vx, target_vy, target_omega])
            time.sleep(0.005)

        elif event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_SOUTH and event.value == 1:
                logging.info("Clear errors button pressed; clearing errors on all modules.")
                for mod in vehicle.modules:
                    mod.clear_errors()

# ========= MAIN SCRIPT =========
def main():
    if not os.path.exists(MOTOR_CONFIG_FILE):
        logging.error(f"Missing {MOTOR_CONFIG_FILE}. Please create it.")
        sys.exit(1)
    with open(MOTOR_CONFIG_FILE, 'r') as f:
        mcfg = json.load(f)

    bus = can.interface.Bus("can0", interface="socketcan")

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
        logging.info(f'Motor {nid}: Flip = {flip}')
        rolling_motors.append(m)

    modules = []
    for i in range(len(turning_motors)):
        modules.append(SwerveModule(turning_motors[i], rolling_motors[i]))

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
    absinfo_y = joystick.absinfo(ecodes.ABS_Y)
    absinfo_x = joystick.absinfo(ecodes.ABS_X)
    logging.info(f"Joystick ABS_Y: min={absinfo_y.min}, max={absinfo_y.max}")
    logging.info(f"Joystick ABS_X: min={absinfo_x.min}, max={absinfo_x.max}")

    robot_size = 0.350   # Adjust as needed
    num_wheels = 3
    caster_offset = 0.015  # Adjust as needed

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
