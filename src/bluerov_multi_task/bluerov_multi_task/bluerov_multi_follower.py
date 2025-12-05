#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
import math

class SimpleFollower(Node):
    def __init__(self):
        super().__init__('simple_follower')

        # Subscribe to Leader and Self
        self.sub_leader = self.create_subscription(
            PoseStamped, '/bluerov1/mavros/local_position/pose', self.leader_cb, 10)
        self.sub_self = self.create_subscription(
            PoseStamped, '/bluerov2/mavros/local_position/pose', self.self_cb, 10)

        # Publisher for Velocity
        self.pub_vel = self.create_publisher(
            Twist, '/bluerov2/mavros/setpoint_velocity/cmd_vel_unstamped', 10)

        self.leader_pose = None
        self.self_pose = None
        
        # TARGET DISTANCE: 1.0 METER
        self.target_dist = 1.0 

        self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"Follower Ready: Maintaining {self.target_dist}m distance")

    def leader_cb(self, msg): self.leader_pose = msg.pose
    def self_cb(self, msg): self.self_pose = msg.pose

    def control_loop(self):
        if not self.leader_pose or not self.self_pose:
            return

        # Calculate vector to leader
        dx = self.leader_pose.position.x - self.self_pose.position.x
        dy = self.leader_pose.position.y - self.self_pose.position.y
        dz = self.leader_pose.position.z - self.self_pose.position.z 

        distance = math.sqrt(dx**2 + dy**2)
        cmd = Twist()
        
        # If > 1.0m, move closer
        if distance > self.target_dist:
            speed = 0.5 # Max speed
            scale = speed / distance
            cmd.linear.x = dx * scale
            cmd.linear.y = dy * scale
        else:
            # Stop if within 1m
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        # Always match depth
        cmd.linear.z = dz * 1.0 

        self.pub_vel.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
