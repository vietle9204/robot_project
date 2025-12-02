#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# # _AXES2TUPLE = {
# #     'sxyz': (0, 0, 0, 0),
# #     'sxyx': (0, 0, 1, 0),
# #     'sxzy': (0, 1, 0, 0),
# #     'sxzx': (0, 1, 1, 0),
# #     'syzx': (1, 0, 0, 0),
# #     'syzy': (1, 0, 1, 0),
# #     'syxz': (1, 1, 0, 0),
# #     'syxy': (1, 1, 1, 0),
# #     'szxy': (2, 0, 0, 0),
# #     'szxz': (2, 0, 1, 0),
# #     'szyx': (2, 1, 0, 0),
# #     'szyz': (2, 1, 1, 0),
# #     'rzyx': (0, 0, 0, 1),  # rotating frame examples
# #     'rxyz': (0, 0, 0, 1),
# # }

# # _NEXT_AXIS = [1, 2, 0, 1]

# # def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
# #     try:
# #         firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
# #     except KeyError:
# #         raise ValueError(f"Invalid axes sequence {axes}")

# #     if frame:
# #         ai, ak = ak, ai
# #     if parity:
# #         ai, aj, ak = -ai, -aj, -ak

# #     half_ai = ai / 2.0
# #     half_aj = aj / 2.0
# #     half_ak = ak / 2.0

# #     ci = math.cos(half_ai)
# #     si = math.sin(half_ai)
# #     cj = math.cos(half_aj)
# #     sj = math.sin(half_aj)
# #     ck = math.cos(half_ak)
# #     sk = math.sin(half_ak)

# #     cc = ci * ck
# #     cs = ci * sk
# #     sc = si * ck
# #     ss = si * sk

# #     if repetition:
# #         q = [
# #             cj * (cs + sc),
# #             sj * (cc + ss),
# #             sj * (cs - sc),
# #             cj * (cc - ss)
# #         ]
# #     else:
# #         q = [
# #             ci * sj * ck + si * cj * sk,
# #             si * cj * ck - ci * sj * sk,
# #             ci * cj * sk + si * sj * ck,
# #             ci * cj * ck - si * sj * sk
# #         ]

# #     if parity:
# #         q[0] = -q[0]
# #         q[1] = -q[1]
# #         q[2] = -q[2]

# #     return tuple(q)  # (x, y, z, w)

# def euler_from_quaternion(qx, qy, qz, qw):
#     """
#     qx, qy, qz, qw : components of quaternion
#     return : roll, pitch, yaw (radians)
#     """
#     # roll (x-axis rotation)
#     sinr_cosp = 2 * (qw * qx + qy * qz)
#     cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
#     roll = math.atan2(sinr_cosp, cosr_cosp)

#     # pitch (y-axis rotation)
#     sinp = 2 * (qw * qy - qz * qx)
#     if abs(sinp) >= 1:
#         pitch = math.copysign(math.pi / 2, sinp)  # clamp
#     else:
#         pitch = math.asin(sinp)

#     # yaw (z-axis rotation)
#     siny_cosp = 2 * (qw * qz + qx * qy)
#     cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
#     yaw = math.atan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

class OdomFromVelEncoder(Node):
    def __init__(self):
        super().__init__('odom_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
        self.declare_parameter('odometry_topic', '/odometry/data')
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value

        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None

        # Covariances
        self.pose_covariance = [0.01] + [0.0]*35
        self.pose_covariance[-1] = 0.02  # yaw
        self.twist_covariance = [0.1] + [0.0]*35
        self.twist_covariance[-1] = 0.1  # yaw rate

    def encoder_callback(self, msg: TwistStamped):
        current_time = msg.header.stamp

        if self.last_time is None:
           self.last_time = current_time
           return

        dt = ((current_time.sec - self.last_time.sec) +
            (current_time.nanosec - self.last_time.nanosec) * 1e-9)
        if dt <= 0.0:
            return
            
        self.last_time = current_time

        v = msg.twist.linear.x
        omega = msg.twist.angular.z

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt

        # q = quaternion_from_euler(0.0, 0.0, self.theta)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        # odom.pose.pose.orientation.w = q[0]
        # odom.pose.pose.orientation.x = q[1]
        # odom.pose.pose.orientation.y = q[2]
        # odom.pose.pose.orientation.z = q[3]

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFromVelEncoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()