#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.srv import CommandLong
import cv2
import numpy as np

class BlueROVKeyboardControl(Node):
    def __init__(self):
        super().__init__('bluerov_keyboard_control')

        # Publishers and service client
        self.pub_rc = self.create_publisher(OverrideRCIn, 'rc/override', 10)
        self.cmd_client = self.create_client(CommandLong, 'cmd/command')

        # Camera servo config
        self.camera_servo_pin = 15.0
        self.servo_min = 1100.0
        self.servo_max = 1900.0
        self.tilt = 1500.0

        # Thruster PWM
        self.forward_pwm = 1500
        self.get_logger().info("Keyboard control node initialized")

        # Camera setup
        self.cap = cv2.VideoCapture(1)  # Change if using RTSP or other index
        if not self.cap.isOpened():
            self.get_logger().error("Camera not found! Check /dev/video0 or your stream URL.")
        self.recording = False
        self.video_writer = None

        self.run()

    def run(self):
        self.get_logger().info("Press 'E' to exit.")
        self.get_logger().info("Controls: ↑ ↓ (camera) | F/B (thrusters) | Enter (record toggle)")
        
        while rclpy.ok():
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().warn("Camera frame not received")
                continue

            # Show video stream
            cv2.imshow("BlueROV Camera", frame)

            # Handle keypress
            key = cv2.waitKey(50) & 0xFF  # 50ms delay (~20 FPS)
            if key == 27 or key == ord('e'):  # ESC or 'e'
                self.stop_all()
                break
            elif key == ord('f'):
                self.move_thrusters(1550)  # Forward
            elif key == ord('b'):
                self.move_thrusters(1550)  # Backward
            elif key == 82 or key == ord('w'):  # Up arrow or 'w'
                self.tilt = min(self.tilt + 50, self.servo_max)
                self.send_servo_command(self.camera_servo_pin, self.tilt)
            elif key == 84 or key == ord('s'):  # Down arrow or 's'
                self.tilt = max(self.tilt - 50, self.servo_min)
                self.send_servo_command(self.camera_servo_pin, self.tilt)
            elif key == 13:  # Enter key
                self.toggle_record(frame)

            # If recording, write frames
            if self.recording and self.video_writer is not None:
                self.video_writer.write(frame)

        self.cap.release()
        cv2.destroyAllWindows()

    def move_thrusters(self, pwm_value):
        """ Send PWM to forward thrusters """
        msg = OverrideRCIn()
        msg.channels[0:6] = [1500] * 6
        msg.channels[4] = pwm_value  # Surge (forward/back)
        self.pub_rc.publish(msg)
        self.get_logger().info(f"Thrusters set to PWM {pwm_value}")

    def send_servo_command(self, pin_number, value):
        """ Send MAV_CMD_DO_SET_SERVO command """
        if not self.cmd_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn("cmd/command service not available")
            return
        req = CommandLong.Request()
        req.command = 183
        req.param1 = pin_number
        req.param2 = value
        self.cmd_client.call_async(req)
        self.get_logger().info(f"Camera tilt PWM = {value}")

    def toggle_record(self, frame):
        """ Toggle recording on/off """
        if not self.recording:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.video_writer = cv2.VideoWriter('bluerov_record.avi', fourcc, 20.0,
                                                (frame.shape[1], frame.shape[0]))
            self.recording = True
            self.get_logger().info("Recording started...")
        else:
            self.recording = False
            self.video_writer.release()
            self.video_writer = None
            self.get_logger().info("Recording stopped and saved as bluerov_record.avi")

    def stop_all(self):
        """ Stop thrusters, release camera, and exit """
        self.move_thrusters(1500)  # Stop motion
        self.send_servo_command(self.camera_servo_pin, 1500)  # Neutral tilt
        if self.video_writer:
            self.video_writer.release()
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info("System shutdown complete.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = BlueROVKeyboardControl()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
