#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        # Declare Topic Name Parameters
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
        self.joint_states_topic = self.get_parameter('joint_states_topic').get_parameter_value().string_value
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value

        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.joint_state_pub = self.create_publisher(JointState, self.joint_states_topic, 10)
        
        self.wheel_radius = 0.0325  # meters
        self.wheel_separation = 0.175  # meters

        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0

        self.last_time = self.get_clock().now()
        
    def encoder_callback(self, msg: TwistStamped):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # kinematics
        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        v_left = (v - omega * self.wheel_separation) / (self.wheel_radius)
        v_right = (v + omega * self.wheel_separation) / (self.wheel_radius)

        self.left_wheel_pos += v_left * dt
        self.right_wheel_pos += v_right * dt

        joint_state = JointState()
        joint_state.header.stamp = msg.header.stamp
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [v_left, v_right]

        self.joint_state_pub.publish(joint_state)

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()