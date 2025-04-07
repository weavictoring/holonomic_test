"""
3-Wheel Holonomic Base with Powered Caster Drive Using Joystick Control (via evdev)

This script:
  1) Loads motor_config.json for each ODrive node (including flip and offset).
  2) Spawns a background thread to parse ODrive encoder feedback.
  3) Sets rolling motors to a 5A current limit.
  4) Uses Ruckig for advanced command smoothing in 3 DOFs (vx, vy, ω).
  5) Computes each module’s desired turning angle and rolling velocity from the 
     smoothed joystick commands, factoring in a caster offset.
  6) Commands turning motors with:  offset + flip * (desiredAngle in turns)
     and rolling motors with:   flip * desiredVelocity (in turns/s).
  7) Runs a control loop at ~250 Hz.

Joystick input is read via the evdev library. The joystick’s ABS_Y axis (inverted so that 
pushing forward gives positive velocity) is used for forward/backward command, and the 
ABS_X axis is used for turning command. Lateral (vy) is set to 0 here but could be extended.

Test carefully at low current & speed with the robot wheels off the ground!
"""

import os, sys, math, json, time, queue, struct, threading
import can, ruckig, numpy as np
from evdev import InputDevice, categorize, ecodes, list_devices

# ========= USER CONFIG =========
CONTROL_FREQ = 50
CONTROL_PERIOD = 1.0 / CONTROL_FREQ

TURNING_NODE_IDS = [0, 2, 4]
ROLLING_NODE_IDS = [1, 3, 5]
ALL_NODE_IDS = TURNING_NODE_IDS + ROLLING_NODE_IDS

CURRENT_LIMIT_ROLLING = 5.0    # 5 A current limit for rolling motors
VEL_LIMIT_ROLLING = 2          # Example velocity limit in turns/s

MAX_LINEAR_VEL = 1.0   # Maximum forward velocity (in turns/s, or your chosen unit)
MAX_ANG_VEL    = 0.5   # Maximum angular velocity (in turns/s for turning)

# Joystick parameters
JOYSTICK_DEADZONE = 0.1
JOYSTICK_SCALE_V = MAX_LINEAR_VEL  # Scale normalized joystick value to velocity
JOYSTICK_SCALE_W = MAX_ANG_VEL     # Scale normalized joystick value to rotational velocity

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
    bus.send(msg)

def set_input_vel(bus, node_id, vel):
    arbid = to_arbid(node_id, FUNC_SET_INPUT_VEL)
    data = struct.pack('<ff', vel, 0.0)
    msg = can.Message(arbitration_id=arbid, data=data, is_extended_id=False)
    bus.send(msg)

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
        set_input_pos(self.bus, self.node_id, raw_angle)

    def command_velocity(self, raw_vel):
        set_input_vel(self.bus, self.node_id, raw_vel)

class SwerveModule:
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
        self.turn_motor.command_position(steer_angle_in_turns)
        self.roll_motor.command_velocity(roll_vel_in_turns_s)

# ========= Helper: Generate Wheel Geometry =========
def generate_wheel_positions(num_wheels, robot_size):
    if num_wheels == 3:
        # Arrange 3 wheels in an equilateral triangle.
        R = robot_size / 2  # radius of circumscribed circle
        positions = []
        for i in range(3):
            theta = math.radians(90 + i * 120)
            positions.append(np.array([R * math.cos(theta), R * math.sin(theta)]))
        return positions
    else:
        # Default: arrange along a square's perimeter.
        half_size = robot_size / 2
        perimeter = 4 * robot_size
        positions = []
        for i in range(num_wheels):
            d = (i * perimeter) / num_wheels
            if d < robot_size:
                x = -half_size + d; y = half_size
            elif d < 2 * robot_size:
                x = half_size; y = half_size - (d - robot_size)
            elif d < 3 * robot_size:
                x = half_size - (d - 2 * robot_size); y = -half_size
            else:
                x = -half_size; y = -half_size + (d - 3 * robot_size)
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
    where x_effective,y_effective = nominal position + caster offset (in radial direction).
    """
    def __init__(self, bus, feedback_collector, module_list, robot_size, num_wheels, caster_offset):
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
        self.robot_size = robot_size
        self.num_wheels = num_wheels
        self.caster_offset = caster_offset
        self.nominal_wheel_positions = generate_wheel_positions(num_wheels, robot_size)

    def start(self):
        self.ctrl_thread.start()

    def stop(self):
        self.run_flag = False
        self.ctrl_thread.join()
        self.feedback.stop()
        for m in self.modules:
            m.set_idle()

    def set_target_velocity(self, command):
        """Command is a list: [vx, vy, ω]."""
        try:
            self.cmd_queue.put({"type": "vel", "value": command}, block=False)
        except queue.Full:
            print("Warning: command queue full.")

    def control_loop(self):
        last_t = time.time()
        while self.run_flag:
            now = time.time()
            dt = now - last_t
            if dt < CONTROL_PERIOD:
                time.sleep(CONTROL_PERIOD - dt)
            last_t = now

            # Check for new command from joystick thread
            if not self.cmd_queue.empty():
                cmd = self.cmd_queue.get_nowait()
                if cmd["type"] == "vel":
                    self.inp.target_velocity = cmd["value"]

            self.rk.update(self.inp, self.out)
            self.out.pass_to_input(self.inp)
            vx, vy, omega = self.out.new_velocity

            # Compute commands for each module.
            for i, mod in enumerate(self.modules):
                nominal = self.nominal_wheel_positions[i]
                norm = np.linalg.norm(nominal)
                offset_vec = (nominal / norm) * self.caster_offset if norm != 0 else np.array([0.0, 0.0])
                effective_nominal = nominal + offset_vec
                v_rot = omega * np.array([-effective_nominal[1], effective_nominal[0]])
                v_total = np.array([vx, vy]) + v_rot
                speed = np.linalg.norm(v_total)
                angle = math.atan2(v_total[1], v_total[0])
                angle_turns = angle / (2 * math.pi)
                mod.command(angle_turns, speed)
    
# ========= JOYSTICK INPUT THREAD (using evdev) =========
def joystick_input_thread(vehicle, joystick):
    # Get calibration info...
    absinfo_y = joystick.absinfo(ecodes.ABS_Y)
    absinfo_x = joystick.absinfo(ecodes.ABS_X)
    center_y = (absinfo_y.min + absinfo_y.max) / 2
    range_y = (absinfo_y.max - center_y)
    center_x = (absinfo_x.min + absinfo_x.max) / 2
    range_x = (absinfo_x.max - center_x)
    print(f"Joystick ABS_Y: min={absinfo_y.min}, max={absinfo_y.max}, center={center_y}")
    print(f"Joystick ABS_X: min={absinfo_x.min}, max={absinfo_x.max}, center={center_x}")

    # Initialize defaults
    norm_y = 0.0
    norm_x = 0.0

    for event in joystick.read_loop():
        if event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_Y:
                val_y = event.value
                norm_y = (center_y - val_y) / range_y  # invert for forward positive
                norm_y = norm_y if abs(norm_y) > JOYSTICK_DEADZONE else 0.0
            elif event.code == ecodes.ABS_X:
                val_x = event.value
                norm_x = (val_x - center_x) / range_x
                norm_x = norm_x if abs(norm_x) > JOYSTICK_DEADZONE else 0.0

            target_vx = norm_y * JOYSTICK_SCALE_V
            target_vy = 0.0  # not used in this demo
            target_omega = norm_x * JOYSTICK_SCALE_W
            vehicle.set_target_velocity([target_vx, target_vy, target_omega])
            time.sleep(0.005)


# ========= MAIN SCRIPT =========
def main():
    # 1) Load motor_config.json
    if not os.path.exists(MOTOR_CONFIG_FILE):
        print(f"Error: missing {MOTOR_CONFIG_FILE}. Please create it.")
        sys.exit(1)
    with open(MOTOR_CONFIG_FILE, 'r') as f:
        mcfg = json.load(f)

    # 2) Create CAN bus (adjust device name as needed)
    bus = can.interface.Bus("can0", interface="socketcan")

    # 3) Create ODriveMotor objects
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

    # 4) Pair into swerve modules (expecting 3 modules)
    modules = []
    for i in range(len(turning_motors)):
        modules.append(SwerveModule(turning_motors[i], rolling_motors[i]))

    # 5) Clear errors, set closed-loop, and set rolling motor limits
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

    # 7) Locate a joystick device using evdev
    from evdev import list_devices
    devices = [InputDevice(path) for path in list_devices()]
    joystick = None
    for dev in devices:
        if "Wireless Controller" in dev.name:
            joystick = dev
            break
    if joystick is None:
        print("No joystick detected! Exiting.")
        sys.exit(1)
    print(f"Joystick detected: {joystick.name}")

    # 8) Create the SwerveVehicle with desired geometry settings.
    robot_size = 0.350    # Adjust as needed (units consistent with ODrive configuration)
    num_wheels = 3
    caster_offset = 0.015  # Adjust caster offset as needed
    vehicle = SwerveVehicle(bus, fb_collector, modules, robot_size, num_wheels, caster_offset)
    vehicle.start()

    # 9) Start a separate thread to process joystick input via evdev.
    js_thread = threading.Thread(target=joystick_input_thread, args=(vehicle, joystick), daemon=True)
    js_thread.start()

    # 10) Run until KeyboardInterrupt.
    try:
        print("Control loop started. Use your joystick to drive the robot.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("CTRL+C detected, stopping vehicle.")
    finally:
        vehicle.stop()
        print("All done.")

if __name__ == "__main__":
    main()
