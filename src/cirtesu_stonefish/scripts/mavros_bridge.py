#!/usr/bin/env python3
"""
-ROS2 Cmd_vel subscriber for BlueROV2 with MAVLink override
-functional startup: heartbeat, mode, arm, RC override
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32, Bool
from pymavlink import mavutil
import numpy as np
from std_msgs.msg import UInt16MultiArray, Float64MultiArray
import time
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

class CmdVelMAV(Node):
    def __init__(self):
        super().__init__('cmd_vel_mavlink')

        # ---------------- MAVLink connection ----------------
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
        self.mav.wait_heartbeat()
        self.get_logger().info("Connected to MAVLink UDP port")

        self.target_system = self.mav.target_system
        self.target_component = self.mav.target_component
        self.rc_channel_values = [1500 for _ in range(18)]


        # ---------------- Velocity mapping -----------------
        self.servvelant = 1500
        a = 1.3871668161975536e-25
        b = -2.080750463000313e-21
        c = 1.3983334487807407e-17
        d = -5.543948233018681e-14
        e = 1.4359451930265564e-10
        f = -2.5387836010458446e-07
        g = 0.0003102899349247361
        h = -0.25886076958834003
        i = 141.07497402237286
        j = -45353.82880615414
        k = 6531725.54047377
        coefficients = np.array([a,b,c,d,e,f,g,h,i,j,k])

        x = np.linspace(1100, 1900, 801)
        y = np.polyval(coefficients, x)
        self.velocidades = np.zeros(801)
        for idx in range(len(x)):
            self.velocidades[idx] = -y[idx] if idx < 400 else y[idx]

        # ---------------- Subscribers ----------------------
        # self.create_subscription(Twist, '/bluerov/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Int32, '/bluerov/camera', self.cam_callback, 10)
        self.create_subscription(Bool, '/bluerov/arm_disarm', self.arm_callback, 10)
        # self.create_subscription(Int32, '/bluerov/luces_pwm', self.luces_callback, 10)
        # self.create_subscription(Int32, '/bluerov/gripper_pwm', self.gripper_callback, 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/bluerov/navigator/odometry',
            self.odom_callback,
            10
        )
        self.depth_publisher = self.create_publisher(Float64, '/bluerov2/global_position/rel_alt', 10)

        self.pwm_sub = self.create_subscription(
            OverrideRCIn,
            '/bluerov2/rc/override',
            self.channel_pwm_callback,
            10)

        # ---------------- Timers ---------------------------
        self.heartbeat_timer = self.create_timer(0.5, self.send_heartbeat)
        self.mav_timer = self.create_timer(0.05, self.process_mavlink)
    #     self.rc_resend_timer = self.create_timer(0.1, self.resend_rc_values)

    # def resend_rc_values(self):
    #     self.mav.mav.rc_channels_override_send(
    #         self.target_system,
    #         self.target_component,
    #         *self.rc_channel_values
    #     )
        # ---------------- Startup sequence -----------------
        self.get_logger().info("Setting MANUAL mode and arming ROV...")
        self.set_mode("manual")
        time.sleep(0.5)  # small delay to allow mode change
        self.arm_throttle(True)

        self.get_logger().info("CmdVelMAV Node Initialized")

    # ---------------- MAVLink helpers -------------------
    def send_heartbeat(self):
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
            mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
            0, 0, 0
        )

    def odom_callback(self, msg: Odometry):
        z_value = msg.pose.pose.position.z
        out_msg = Float64()
        out_msg.data = -z_value
        self.depth_publisher.publish(out_msg)

    def process_mavlink(self):
        """Process incoming MAVLink messages to prevent timeout"""
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break

    def set_rc_channel_pwm(self, channel_id, pwm):
        if channel_id < 1 or channel_id > 18:
            self.get_logger().warn(f"Invalid RC channel: {channel_id}")
            return
        
        self.rc_channel_values[channel_id-1] = pwm
        self.mav.mav.rc_channels_override_send(
            self.target_system,
            self.target_component,
            *self.rc_channel_values
        )

    def arm_throttle(self, arm: bool):
        if arm:
            self.mav.arducopter_arm()
            self.get_logger().info("ROV Armed")
        else:
            self.mav.arducopter_disarm()
            self.get_logger().info("ROV Disarmed")

    def set_mode(self, mode):
        """Set ArduSub mode"""
        mode = mode.upper()
        if mode not in self.mav.mode_mapping():
            self.get_logger().warn(f"Unknown mode: {mode}")
            return
        mode_id = self.mav.mode_mapping()[mode]
        self.mav.set_mode(mode_id)
        self.get_logger().info(f"Mode set to {mode}")

    # ---------------- cmd_vel callback -----------------
    
    def cmd_vel_callback(self, msg):
        # Map velocities to thrusters

        self.map_velocity(msg.linear.x, 5, "X")
        self.map_velocity(msg.linear.y, 6, "Y")
        self.map_velocity(msg.linear.z, 3, "Z")
        self.map_velocity(msg.angular.z, 4, "Yaw")
        self.map_velocity(msg.angular.y, 1, "Pitch")
        self.map_velocity(msg.angular.x, 2, "Roll")
        

    def map_velocity(self, value, channel, name=""):
        if -1.0 <= value <= 1.0 and value != 0.0:
            idx = np.argmin(np.abs(self.velocidades - value))
            pwm = idx + 1100
            self.set_rc_channel_pwm(channel, pwm)
        else:
            self.set_rc_channel_pwm(channel, 1500)
  
    
    def channel_pwm_callback(self, msg):
    # msg.data is a list of floats, typically normalized between -1 and 1 or 1100–1900 range
        # for channel, value in enumerate(msg.data, start=1):
        #     pwm = int(value) if value > 100 else int(1500 + value * 400)  # adjust if normalized
        #     self.set_rc_channel_pwm(channel, pwm)
        for i in range(8):
            self.set_rc_channel_pwm(i+1, msg.channels[i])
            self.get_logger().info(f"PWM Override Received (ch {i+1}): {msg.channels[i]}")



    # ---------------- Camera callback ------------------
    def cam_callback(self, msg):
        pwm = msg.data
        if 1100 <= pwm <= 1900:
            self.set_rc_channel_pwm(15, pwm)
            time.sleep(3)
            self.servvelant = pwm
            self.set_rc_channel_pwm(15, 1500)

    # ---------------- Arm/Disarm callback ---------------
    def arm_callback(self, msg: Bool):
        self.arm_throttle(msg.data)

    # ---------------- Lights PWM callback --------------
    def luces_callback(self, msg):
        pwm = msg.data
        if 1100 <= pwm <= 1900:
            self.set_rc_channel_pwm(9, pwm)
            

    # ---------------- Gripper PWM callback -------------
    def gripper_callback(self, msg):
        pwm = msg.data
        if 1100 <= pwm <= 1900:
            self.set_rc_channel_pwm(3, pwm)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMAV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.arm_throttle(False)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()