#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from mavros_msgs.srv import CommandBool, SetMode

class JoystickManager(Node):
    def __init__(self):
        super().__init__('joystick_manager')

        # Subscribe to the global joystick topic
        self.sub_joy = self.create_subscription(Joy, '/joy', self.joy_cb, 10)

        # Clients for Robot 1 (Leader) ONLY
        self.cli_arm_1 = self.create_client(CommandBool, '/bluerov1/mavros/cmd/arming')
        self.cli_mode_1 = self.create_client(SetMode, '/bluerov1/mavros/set_mode')

        # State tracking
        self.last_start = 0
        self.last_back = 0
        self.last_a = 0

        self.get_logger().info("Manager Ready: START=Arm R1, BACK=Disarm R1, A=Manual R1")

    def joy_cb(self, msg):
        # Xbox 360/One Mapping: 0:A, 6:BACK, 7:START
        
        # START: ARM + GUIDED (Robot 1)
        if msg.buttons[7] == 1 and self.last_start == 0:
            self.get_logger().info("START: Arming Robot 1 (GUIDED)...")
            self.set_state(self.cli_mode_1, self.cli_arm_1, 'GUIDED', True)

        # BACK: DISARM (Robot 1)
        if msg.buttons[6] == 1 and self.last_back == 0:
            self.get_logger().warn("BACK: DISARMING Robot 1!")
            self.set_state(self.cli_mode_1, self.cli_arm_1, 'MANUAL', False)

        # A: MANUAL (Robot 1 - No Disarm)
        if msg.buttons[0] == 1 and self.last_a == 0:
            self.get_logger().info("A: Switching Robot 1 to MANUAL mode...")
            self.set_state(self.cli_mode_1, self.cli_arm_1, 'MANUAL', None)

        self.last_start = msg.buttons[7]
        self.last_back = msg.buttons[6]
        self.last_a = msg.buttons[0]

    def set_state(self, mode_cli, arm_cli, mode, arm):
        if mode and mode_cli.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = mode
            mode_cli.call_async(req)
        
        if arm is not None and arm_cli.service_is_ready():
            req = CommandBool.Request()
            req.value = arm
            arm_cli.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
