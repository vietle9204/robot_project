#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Pose, TwistStamped, Twist   
# import math

# class OdometryPublisher(Node):
#     def __init__(self):
#         super().__init__('odometry_publisher')
        
#         # Subscriber
#         self.joint_sub = self.create_subscription(JointState, '/joint_states_raw', self.joint_callback, 10)
        
        
#         # Publisher
#         self.vel_encoder_pub = self.create_publisher(TwistStamped, '/vel_encoder/data', 10)

#         self.time = 0.01
        
#         self.create_timer(0.01, timer_cb)
        
#         self.wheel_radius = 0.0325  # meters
#         self.wheel_separation = 0.175  # meters

#         self.last_time = none
#         self.stamp = none

#         # Create odometry message
#         self.vel_msg = TwistStamped()
#         self.vel_msg.header.frame_id = "base_link"

#     self.timer_cb(self):
#         if self.stamp is none:
#             return
#         self.vel_encoder_pub.publish(vel_msg)
        
        
#     def joint_callback(self, msg: JointState):
#         if self.last_time is None:
#             self.last_time = self.get_clock().now()
#             return
#         current_time = self.get_clock().now()
#         dt = (current_time - self.last_time).nanoseconds / 1e9
#         self.last_time = current_time
        
#         # kinematics
#         v = (msg.velocity[0] + msg.velocity[1]) * 0.5 * self.wheel_radius
#         omega = (msg.velocity[1] - msg.velocity[0]) * self.wheel_radius / self.wheel_separation

#         self.vel_msg.twist.linear.x = float(v)
#         self.vel_msg.twist.linear.y = 0.0
#         self.vel_msg.twist.angular.z = float(omega)
#         self.vel_msg.header.stamp = msg.header.stamp

        

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryPublisher()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped


class Encoder_vel(Node):
    def __init__(self):
        super().__init__('odometry_publisher')
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states_raw', self.joint_callback, 10
        )
        
        # Publisher
        self.vel_encoder_pub = self.create_publisher(
            TwistStamped, '/vel_encoder/data', 10
        )

        # Params
        self.wheel_radius = 0.0325          # meters
        self.wheel_separation = 0.175       # meters

        self.last_time = None
        self.stamp = None

        # Output message
        self.vel_msg = TwistStamped()
        self.vel_msg.header.frame_id = "base_link"

        # Timer (100 Hz)
        self.create_timer(0.02, self.timer_cb)

    def timer_cb(self):
        if self.stamp is None:   # chưa có dữ liệu joint
            return
        
        self.vel_msg.header.stamp = self.stamp
        self.vel_encoder_pub.publish(self.vel_msg)

    def joint_callback(self, msg: JointState):
        # Save timestamp to use in timer
        self.stamp = msg.header.stamp

        if len(msg.velocity) < 2:
            self.get_logger().warn("JointState velocity has <2 elements")
            return

        if self.last_time is None:
            self.last_time = self.get_clock().now()
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Wheel velocities
        wl = msg.velocity[0]
        wr = msg.velocity[1]

        # Differential-drive kinematics
        v = (wr + wl) * 0.5 * self.wheel_radius
        omega = (wr - wl) * self.wheel_radius / self.wheel_separation

        # Fill message
        self.vel_msg.twist.linear.x = float(v)
        self.vel_msg.twist.linear.y = 0.0
        self.vel_msg.twist.angular.z = float(omega)


def main(args=None):
    rclpy.init(args=args)
    node = Encoder_vel()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
