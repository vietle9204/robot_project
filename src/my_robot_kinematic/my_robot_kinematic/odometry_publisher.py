#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist   
import math

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Subscriber
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odometry/data', 10)
        
        self.wheel_radius = 0.0325  # meters
        self.wheel_separation = 0.175  # meters

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = self.get_clock().now()
        
    def joint_callback(self, msg: JointState):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # kinematics
        v = (msg.velocity[0] + msg.velocity[1]) * 0.5 * self.wheel_radius
        omega = (msg.velocity[1] - msg.velocity[0]) * self.wheel_radius / self.wheel_separation

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        #set odom values
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)
        
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(omega)
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()