#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import math
from rclpy.qos import qos_profile_sensor_data


class WheelOdomPublisher(Node):
    def __init__(self):
        super().__init__('wheel_odom_publisher')
        
        # Robot parameters
        self.wheel_radius = 0.0325  # meters
        self.wheel_separation = 0.175  # meters
        
        # Joint positions
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        
        # Previous time for integration
        self.last_time = None
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # ✅ Subscriber with QoS compatible with odometry/data (Best Effort)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odometry/data',
            self.odom_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info('Wheel odometry publisher started')

    def odom_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        
        # Get velocities from odometry
        linear_vel = msg.twist.twist.linear.x
        angular_vel = msg.twist.twist.angular.z
        
        # Differential drive kinematics
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Integrate positions
        self.left_wheel_pos += left_wheel_vel * dt
        self.right_wheel_pos += right_wheel_vel * dt
        
        # Publish joint states
        joint_state = JointState()
        joint_state.header.stamp = current_time.to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [left_wheel_vel, right_wheel_vel]
        
        self.joint_pub.publish(joint_state)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()