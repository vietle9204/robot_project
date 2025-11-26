#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TwistStamped, Twist   
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states_raw', self.joint_callback, 10)
        
        
        # Publisher
        self.vel_encoder_pub = self.create_publisher(TwistStamped, '/vel_encoder/data', 10)
        
        self.wheel_radius = 0.0325  # meters
        self.wheel_separation = 0.175  # meters

        self.last_time = self.get_clock().now()
        
    def joint_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Create odometry message
        vel_msg = TwistStamped()
        vel_msg.header.stamp = msg.header.stamp
        vel_msg.header.frame_id = "base_link"
        
        # kinematics
        v = (msg.velocity[0] + msg.velocity[1]) * 0.5 * self.wheel_radius
        omega = (msg.velocity[1] - msg.velocity[0]) * self.wheel_radius / self.wheel_separation

        vel_msg.twist.linear.x = float(v)
        vel_msg.twist.linear.y = 0.0
        vel_msg.twist.angular.z = float(omega)  

        self.vel_encoder_pub.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()