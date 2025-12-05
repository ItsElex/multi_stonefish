#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CmdVelToPWM(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_pwm')

        # Parameters
        self.declare_parameter('robot_name', 'bluerov2')
        self.declare_parameter('max_rpm', 2000.0)  # Conservative max RPM
        self.declare_parameter('max_linear_vel', 1.0)  # m/s
        self.declare_parameter('max_angular_vel', 1.0)  # rad/s

        robot_name = self.get_parameter('robot_name').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value

        # Subscribe to cmd_vel
        self.create_subscription(
            Twist,
            f'/{robot_name}/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish to setpoint/pwm (actually RPM)
        self.pwm_pub = self.create_publisher(
            Float64MultiArray,
            f'/{robot_name}/setpoint/pwm',
            10
        )

        self.get_logger().info(
            f'CmdVelToPWM started for {robot_name}\n'
            f'  Listening: /{robot_name}/cmd_vel\n'
            f'  Publishing: /{robot_name}/setpoint/pwm\n'
            f'  Max RPM: {self.max_rpm}'
        )

    def cmd_vel_callback(self, msg: Twist):
        # Extract desired velocities
        vx = msg.linear.x  # Forward velocity (m/s)
        wz = msg.angular.z  # Yaw rate (rad/s)

        # Normalize to -1.0 to 1.0
        vx_norm = max(-1.0, min(1.0, vx / self.max_linear))
        wz_norm = max(-1.0, min(1.0, wz / self.max_angular))

        # Thruster mixing for BlueROV Heavy (8 thrusters, vectored configuration)
        # T1-T4: horizontal thrusters at 45° angles
        # T5-T8: vertical thrusters (keep at 0 for now)
        
        # Based on thruster specs we saw:
        # T1 (R_FORWARD_FRONT): inverted=true, right=true, 45°
        # T2 (L_FORWARD_FRONT): inverted=true, right=true, -45°
        # T3 (R_FORWARD_REAR): inverted=false, right=false, -45°
        # T4 (L_FORWARD_REAR): inverted=true, right=true, 45°
        
        # Simple differential mixing:
        # Forward: all horizontal thrusters contribute
        # Yaw: differential between left and right
        
        t1 = (-vx_norm + wz_norm) * self.max_rpm
        t2 = (-vx_norm - wz_norm) * self.max_rpm
        t3 = (vx_norm + wz_norm) * self.max_rpm
        t4 = (vx_norm - wz_norm) * self.max_rpm
        
        # Vertical thrusters (neutral for now)
        t5 = t6 = t7 = t8 = 0.0

        # Clamp to max RPM
        def clamp(val):
            return max(-self.max_rpm, min(self.max_rpm, val))

        rpm_array = Float64MultiArray()
        rpm_array.data = [
            clamp(t1), clamp(t2), clamp(t3), clamp(t4),
            t5, t6, t7, t8
        ]

        self.pwm_pub.publish(rpm_array)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToPWM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
