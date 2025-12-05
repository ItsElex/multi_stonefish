#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandLong, SetMode
from mavros_msgs.msg import OverrideRCIn, MountControl, State


class BlueROVJoystick(Node):
    def __init__(self):
        super().__init__('bluerov_joystick_node')
        self.get_logger().info('Starting BlueROV joystick node (manual/auto + depth-hold)')

        # ----------------- Parameters -----------------
        self.declare_parameter('light_pin', 12.0)        # default: lights on pin 12
        self.declare_parameter('gripper_pin', 10.0)      # default: gripper on pin 10
        self.declare_parameter('camera_servo_pin', 16.0) # default: camera tilt on pin 16

        self.light_pin = float(self.get_parameter('light_pin').value)
        self.gripper_pin = float(self.get_parameter('gripper_pin').value)
        self.camera_servo_pin = float(self.get_parameter('camera_servo_pin').value)

        # ----------------- Publishers -----------------
        self.override_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.mount_pub = self.create_publisher(MountControl, '/mavros/mount_control/command', 10)

        # ----------------- QoS & Subscribers -----------------
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, qos_profile)
        self.cmdvel_sub = self.create_subscription(Twist, '/bluerov/cmd_vel', self.vel_callback, qos_profile)
        self.state_sub = self.create_subscription(State, '/mavros/state', self.state_callback, 10)

        # ----------------- Services -----------------
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        while not self.cmd_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Waiting for MAVROS CommandLong service...')

        self.set_mode_cli = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for MAVROS SetMode service...')

        # ----------------- Timers -----------------
        self.timer = self.create_timer(0.05, self.continuous_control_callback)

        # ----------------- Button / control state -----------------
        self.dpad_left_held = False
        self.dpad_right_held = False
        self.dpad_up_held = False
        self.dpad_down_held = False

        # Lights PWM
        self.light = 1100.0
        self.light_min = 1100.0
        self.light_max = 1900.0
        self.light_step = 15.0

        # Camera tilt (deg)
        self.tilt = 0.0
        self.tilt_step = 2.0
        self.tilt_min = -60.0
        self.tilt_max = 60.0

        # Gripper PWM
        self.gripper = 1150.0
        self.gripper_min = 1150.0
        self.gripper_max = 1580.0
        self.gripper_step = 430.0

        self.rt_was_pressed = False
        self.lt_was_pressed = False

        # ----------------- PWM and scaling -----------------
        self.PWM_CENTER = 1500
        self.PWM_MIN = 1100
        self.PWM_MAX = 1900
        self.THRUSTER_SAFE_MIN = 1300
        self.THRUSTER_SAFE_MAX = 1700

        self.axis_deadzone = 0.08
        self.axis_expo = 0.5
        self.scale_surge = 0.25
        self.scale_lateral = 0.25
        self.scale_yaw = 0.25
        self.scale_heave = 0.20

        # Control modes: [manual, automatic]
        self.set_mode = [True, False]   # start in MANUAL
        self.arming = False

        # Depth-hold flag: toggles FCU mode MANUAL <-> ALT_HOLD, but does NOT change set_mode
        self.depth_hold = False
        self.prev_depth_hold_btn = 0

        self.latest_cmd_vel = Twist()
        self.have_cmd_vel = False

        self.get_logger().info('BlueROV joystick initialized.')

    # ----------------- MAVROS helpers -----------------

    def state_callback(self, msg: State):
        # Can be used for diagnostics if desired
        pass

    def set_flight_mode(self, mode_str: str):
        if not self.set_mode_cli.service_is_ready():
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode_str  # e.g. 'MANUAL', 'ALT_HOLD'
        self.set_mode_cli.call_async(req)

    def send_neutral_override(self):
        msg_override = OverrideRCIn()
        msg_override.channels = [int(self.PWM_CENTER)] * 18
        self.override_pub.publish(msg_override)

    # ----------------- Joystick callback -----------------

    def joy_callback(self, data: Joy):
        # Buttons (Xbox-style)
        btn_arm = data.buttons[7]
        btn_disarm = data.buttons[6]
        btn_manual_mode = data.buttons[3]
        btn_automatic_mode = data.buttons[2]
        btn_depth_hold_mode = data.buttons[0]
        btn_camera_rest = data.buttons[9]

        btn_gripper_open_axis = data.axes[2]   # LT
        btn_gripper_close_axis = data.axes[5]  # RT

        btn_light = data.axes[6]               # D-pad left/right
        btn_camera_tilt = data.axes[7]         # D-pad up/down

        self.dpad_left_held = btn_light == 1.0
        self.dpad_right_held = btn_light == -1.0
        self.dpad_up_held = btn_camera_tilt == 1.0
        self.dpad_down_held = btn_camera_tilt == -1.0

        # Arm / disarm via MAV_CMD_COMPONENT_ARM_DISARM (400)
        if btn_disarm == 1 and self.arming:
            self.get_logger().info("Disarm requested.")
            self.arming = False
            self.arm_disarm(False)
        if btn_arm == 1 and not self.arming:
            self.get_logger().info("Arm requested.")
            self.arming = True
            self.arm_disarm(True)

        # Mode toggles for RC mapping
        if btn_manual_mode and not self.set_mode[0]:
            self.set_mode = [True, False]
            self.get_logger().info("Switched to MANUAL control (joystick → RC).")
            # Flight mode stays whatever it currently is (MANUAL or ALT_HOLD)
        elif btn_automatic_mode and not self.set_mode[1]:
            self.set_mode = [False, True]
            self.get_logger().info("Switched to AUTOMATIC control (cmd_vel → RC).")

        # Depth-hold toggle: only changes FCU mode MANUAL <-> ALT_HOLD
        if btn_depth_hold_mode == 1 and self.prev_depth_hold_btn == 0:
            if not self.depth_hold:
                self.depth_hold = True
                self.set_flight_mode('ALT_HOLD')
                self.get_logger().info("Depth Hold ON (ALT_HOLD).")
            else:
                self.depth_hold = False
                self.set_flight_mode('MANUAL')
                self.get_logger().info("Depth Hold OFF (MANUAL).")
        self.prev_depth_hold_btn = btn_depth_hold_mode

        # Camera tilt reset
        if btn_camera_rest == 1:
            self.tilt = 0.0
            self.send_camera_tilt_command(self.tilt)
            self.get_logger().info("Camera tilt reset to 0 degrees")

        # Gripper controls (momentary)
        rt_pressed = btn_gripper_close_axis < -0.5
        lt_pressed = btn_gripper_open_axis > 0.5

        if rt_pressed and not self.rt_was_pressed and self.gripper < self.gripper_max:
            self.gripper = min(self.gripper + self.gripper_step, self.gripper_max)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper closing PWM: {self.gripper}")
        if lt_pressed and not self.lt_was_pressed and self.gripper > self.gripper_min:
            self.gripper = max(self.gripper - self.gripper_step, self.gripper_min)
            self.send_servo_command(self.gripper_pin, self.gripper)
            self.get_logger().info(f"Gripper opening PWM: {self.gripper}")

        self.rt_was_pressed = rt_pressed
        self.lt_was_pressed = lt_pressed

        # MANUAL control path: joystick → RC override
        if self.set_mode[0]:
            surge_pwm = self.mapValueScalSat(data.axes[1], self.scale_surge)
            lateral_pwm = self.mapValueScalSat(-data.axes[0], self.scale_lateral)
            # Always pass heave through; ALT_HOLD in FC handles depth
            heave_pwm = self.mapValueScalSat(data.axes[4], self.scale_heave)
            yaw_pwm = self.mapValueScalSat(-data.axes[3], self.scale_yaw)
            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER
            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)

    # ----------------- cmd_vel + continuous control -----------------

    def vel_callback(self, cmd_vel: Twist):
        self.latest_cmd_vel = cmd_vel
        self.have_cmd_vel = True

    def continuous_control_callback(self):
        # Handle camera tilt and lights from DPAD
        changed_tilt = False
        changed_light = False

        if self.dpad_up_held:
            self.tilt = min(self.tilt + self.tilt_step, self.tilt_max)
            changed_tilt = True
        elif self.dpad_down_held:
            self.tilt = max(self.tilt - self.tilt_step, self.tilt_min)
            changed_tilt = True

        if self.dpad_right_held:
            self.light = min(self.light + self.light_step, self.light_max)
            changed_light = True
        elif self.dpad_left_held:
            self.light = max(self.light - self.light_step, self.light_min)
            changed_light = True

        if changed_tilt:
            self.send_camera_tilt_command(self.tilt)
        if changed_light:
            self.send_servo_command(self.light_pin, self.light)

        # MANUAL control is applied in joy_callback; nothing else here
        if self.set_mode[0]:
            return

        # AUTOMATIC: cmd_vel → RC override
        if self.set_mode[1]:
            if not self.have_cmd_vel:
                # No command yet: neutral
                self.setOverrideRCIN(
                    self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER,
                    self.PWM_CENTER, self.PWM_CENTER, self.PWM_CENTER
                )
                return

            surge_pwm = self.mapValueScalSat(self._clamp_unit(self.latest_cmd_vel.linear.x),
                                             self.scale_surge)
            lateral_pwm = self.mapValueScalSat(self._clamp_unit(self.latest_cmd_vel.linear.y),
                                               self.scale_lateral)
            # Again: always pass heave; FC does depth hold in ALT_HOLD
            heave_pwm = self.mapValueScalSat(self._clamp_unit(self.latest_cmd_vel.linear.z),
                                             self.scale_heave)
            yaw_pwm = self.mapValueScalSat(self._clamp_unit(self.latest_cmd_vel.angular.z),
                                           self.scale_yaw)

            pitch_pwm = self.PWM_CENTER
            roll_pwm = self.PWM_CENTER
            self.setOverrideRCIN(pitch_pwm, roll_pwm, heave_pwm, yaw_pwm, surge_pwm, lateral_pwm)
            return

    # ----------------- Low-level helpers -----------------

    def _clamp_unit(self, v: float):
        return max(-1.0, min(1.0, float(v)))

    def send_servo_command(self, pin_number: float, value: float):
        # MAV_CMD_DO_SET_SERVO (183): param1 = servo number, param2 = PWM
        if not self.cmd_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 183
        req.confirmation = 0
        req.param1 = float(pin_number)
        req.param2 = float(value)
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        self.cmd_client.call_async(req)

    def send_camera_tilt_command(self, tilt_angle_deg: float):
        msg = MountControl()
        msg.mode = 2  # MAV_MOUNT_MODE_MAVLINK_TARGETING
        msg.pitch = float(tilt_angle_deg)
        msg.roll = 0.0
        msg.yaw = 0.0
        msg.altitude = 0.0
        msg.latitude = 0.0
        msg.longitude = 0.0
        self.mount_pub.publish(msg)
        # Optional PWM center for servo
        self.send_servo_command(self.camera_servo_pin, self.PWM_CENTER)

    def arm_disarm(self, armed: bool):
        # MAV_CMD_COMPONENT_ARM_DISARM (400) via CommandLong
        if not self.cmd_client.service_is_ready():
            return
        req = CommandLong.Request()
        req.broadcast = False
        req.command = 400
        req.confirmation = 0
        req.param1 = 1.0 if armed else 0.0
        req.param2 = 0.0
        req.param3 = 0.0
        req.param4 = 0.0
        req.param5 = 0.0
        req.param6 = 0.0
        req.param7 = 0.0
        self.cmd_client.call_async(req)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle,
                        channel_yaw, channel_forward, channel_lateral):
        msg_override = OverrideRCIn()
        msg_override.channels = [int(self.PWM_CENTER)] * 18
        msg_override.channels[0] = int(self._clamp_pwm(channel_pitch))
        msg_override.channels[1] = int(self._clamp_pwm(channel_roll))
        msg_override.channels[2] = int(self._clamp_pwm(channel_throttle))
        msg_override.channels[3] = int(self._clamp_pwm(channel_yaw))
        msg_override.channels[4] = int(self._clamp_pwm(channel_forward))
        msg_override.channels[5] = int(self._clamp_pwm(channel_lateral))
        msg_override.channels[6] = int(self.PWM_CENTER)
        msg_override.channels[7] = int(self.PWM_CENTER)
        self.override_pub.publish(msg_override)

    def _clamp_pwm(self, pwm_value):
        try:
            pwm = int(pwm_value)
        except Exception:
            pwm = self.PWM_CENTER
        pwm = max(self.PWM_MIN, min(self.PWM_MAX, pwm))
        pwm = max(self.THRUSTER_SAFE_MIN, min(self.THRUSTER_SAFE_MAX, pwm))
        return pwm

    def mapValueScalSat(self, value: float, scale: float):
        return self.axis_to_pwm(value, scale, self.axis_expo)

    def axis_to_pwm(self, axis_val: float, scale: float, expo: float):
        if abs(axis_val) < self.axis_deadzone:
            return self.PWM_CENTER
        shaped = (1.0 - expo) * axis_val + expo * (axis_val ** 3)
        shaped = max(-1.0, min(1.0, shaped))
        pwm_signal = self.PWM_CENTER + int(shaped * scale * (self.PWM_MAX - self.PWM_CENTER))
        pwm_signal = min(max(self.PWM_MIN, pwm_signal), self.PWM_MAX)
        return pwm_signal


def main(args=None):
    rclpy.init(args=args)
    node = BlueROVJoystick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_neutral_override()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

