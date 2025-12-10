#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time

class ToFPIDNode(Node):
    def __init__(self):
        super().__init__('tof_pid_node')

        # ----- Serial setup -----
        self.port = '/dev/ttyAMA0'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to {self.port} at {self.baudrate} baud")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            rclpy.shutdown()
            return

        # ----- Publisher -----
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_tof', 10)
        # Publisher for front 3 ToF distances
        self.tof_dist_pub = self.create_publisher(Float32MultiArray, '/tof_distances', 10)


        # ----- PID parameters -----
        self.kp = 0.01       # proportional gain
        self.ki = 0.0         # integral gain
        self.kd = 0.001       # derivative gain
        self.target_dist = 1000.0  # target distance in mm

        self.max_angular_z = 0.75  # rad/s (maximum angular velocity)

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

        # Loop at 20 Hz
        self.timer = self.create_timer(0.03, self.control_loop)

    def control_loop(self):
        """Read sensor data and perform angular correction with PID."""
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                if not line:
                    return

                # Expecting: 8 comma-separated readings
                parts = line.split(',')
                if len(parts) != 8:
                    self.get_logger().warn(f"Invalid data: {line}")
                    return

                readings = [float(x) for x in parts]

                # Extract relevant sensors
                front_right = readings[4]
                front_left = readings[3]
                front_center = readings[5]
                left = readings[0]
                right = readings[1]

                # Publish front & side distances
                dist_msg = Float32MultiArray()
                dist_msg.data = [front_left, front_center, front_right, left, right]
                self.tof_dist_pub.publish(dist_msg)

                # Log readings
                self.get_logger().info(
                    f"FL: {front_left:.0f} | FC: {front_center:.0f} | FR: {front_right:.0f} | L: {left:.0f} | R: {right:.0f}"
                )

                # PID for angular.z
                error = (front_left - front_right)
                current_time = time.time()
                dt = current_time - self.last_time if self.last_time else 0.1

                self.integral += error * dt
                derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

                pid_output = (
                    self.kp * error +
                    self.ki * self.integral +
                    self.kd * derivative
                )

                # Limit maximum angular speed
                pid_output = max(-self.max_angular_z, min(self.max_angular_z, pid_output))

                # Build Twist message
                twist = Twist()
                twist.angular.z = float(pid_output)
                twist.linear.x = 0.07  # Default forward speed
                twist.linear.y = 0.0   # Default strafe speed

                # Front-center obstacle avoidance
                if front_center < 300.0:
                    backoff_speed = -0.1
                    twist.linear.x = backoff_speed
                    self.get_logger().warn(f"Front center {front_center:.0f}mm < 300mm: backing off {backoff_speed} m/s")

                # Left/right obstacle avoidance (strafe)
                side_threshold = 300.0  # mm
                strafe_speed = 0.2     # m/s

                if right < side_threshold:
                    twist.linear.y = strafe_speed  # Move right
                    self.get_logger().warn(f"Left {left:.0f}mm < {side_threshold}mm: strafing right {strafe_speed} m/s")
                elif left < side_threshold:
                    twist.linear.y = -strafe_speed  # Move left
                    self.get_logger().warn(f"Right {right:.0f}mm < {side_threshold}mm: strafing left {strafe_speed} m/s")

                # Publish command
                self.cmd_pub.publish(twist)

                self.prev_error = error
                self.last_time = current_time

                # Optional: clear integral when no obstacle
                self.integral = 0.0

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ToFPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()