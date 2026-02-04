#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from transforms3d.quaternions import quat2mat, mat2quat


class MapToOdom(Node):
    def __init__(self):
        super().__init__("map_to_odom")

        # SLAM / EKF global pose: map -> base_link
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/ekf_slam/pose",
            self.pose_cb,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("✅ map_to_odom (Nav2-ready) started")

    # -------------------------------------------------

    def pose_cb(self, msg):
        # map -> base_link from SLAM
        T_map_base = self.pose_to_matrix(msg)

        stamp = Time.from_msg(msg.header.stamp)

        try:
            # odom -> base_link from odometry
            tf_odom_base = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                Time(),
                timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        T_odom_base = self.transform_to_matrix(tf_odom_base)

        # map -> odom
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        self.publish_map_to_odom(T_map_odom, msg.header.stamp)

    # -------------------------------------------------

    def publish_map_to_odom(self, T, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = "map"
        t.child_frame_id = "odom"

        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = 0.0

        # ---- yaw-only quaternion (Nav2 safe) ----
        yaw = np.arctan2(T[1, 0], T[0, 0])

        R = np.array([
            [np.cos(yaw), -np.sin(yaw), 0.0],
            [np.sin(yaw),  np.cos(yaw), 0.0],
            [0.0,          0.0,         1.0]
        ])

        q = mat2quat(R)  # w, x, y, z

        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)


    # -------------------------------------------------

    def pose_to_matrix(self, msg):
        q = msg.pose.pose.orientation
        R = quat2mat([q.w, q.x, q.y, q.z])

        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = msg.pose.pose.position.x
        T[1, 3] = msg.pose.pose.position.y
        return T

    def transform_to_matrix(self, tf):
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])

        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        return T


# =====================================================

def main(args=None):
    rclpy.init(args=args)
    node = MapToOdom()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
