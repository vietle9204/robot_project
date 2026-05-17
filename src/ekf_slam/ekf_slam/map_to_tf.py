#!/usr/bin/env python3

import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    TransformStamped
)

from tf2_ros import (
    TransformBroadcaster,
    Buffer,
    TransformListener
)

from transforms3d.quaternions import quat2mat

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy
)


# =====================================================
# QoS
# =====================================================

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=5
)


# =====================================================
# MAP -> ODOM TF BRIDGE
# =====================================================

class MapToOdom(Node):

    def __init__(self):

        super().__init__("map_to_odom")

        # =================================================
        # PARAMETERS
        # =================================================

        self.declare_parameter(
            "topic",
            "/ekf_slam/pose"
        )

        self.declare_parameter(
            "map_frame",
            "map"
        )

        self.declare_parameter(
            "odom_frame",
            "odom"
        )

        self.declare_parameter(
            "base_frame",
            "base_link"
        )

        self.topic = self.get_parameter(
            "topic"
        ).value

        self.map_frame = self.get_parameter(
            "map_frame"
        ).value

        self.odom_frame = self.get_parameter(
            "odom_frame"
        ).value

        self.base_frame = self.get_parameter(
            "base_frame"
        ).value

        # =================================================
        # TF
        # =================================================

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.tf_broadcaster = TransformBroadcaster(
            self
        )

        # =================================================
        # SUBSCRIBER
        # =================================================

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic,
            self.pose_cb,
            qos
        )

        self.get_logger().info(
            f"Bridge started: "
            f"{self.topic} -> "
            f"TF {self.map_frame}->{self.odom_frame}"
        )

    # =================================================
    # CALLBACK
    # =================================================

    def pose_cb(self, msg):

        # map -> base
        T_map_base = self.pose_to_matrix(msg)

        stamp_msg = msg.header.stamp

        try:

            # odom -> base tại cùng timestamp
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                stamp_msg,
                timeout=Duration(seconds=0.01)
            )

        except Exception as e:

            self.get_logger().warn(
                f"TF lookup failed: {str(e)}"
            )

            return

        # transform matrix
        T_odom_base = self.transform_to_matrix(
            tf_odom_base
        )

        # map -> odom
        T_map_odom = (
            T_map_base @
            np.linalg.inv(T_odom_base)
        )

        # kiểm tra NaN
        if np.any(np.isnan(T_map_odom)):

            self.get_logger().warn(
                "NaN detected in map->odom transform"
            )

            return

        # publish tf
        self.publish_map_to_odom(
            T_map_odom,
            stamp_msg
        )

    # =================================================
    # PUBLISH TF
    # =================================================

    def publish_map_to_odom(self, T, stamp):

        t = TransformStamped()

        t.header.stamp = stamp

        t.header.frame_id = self.map_frame

        t.child_frame_id = self.odom_frame

        # =============================================
        # translation
        # =============================================

        t.transform.translation.x = T[0, 3]

        t.transform.translation.y = T[1, 3]

        t.transform.translation.z = 0.0

        # =============================================
        # yaw only quaternion
        # =============================================

        yaw = np.arctan2(
            T[1, 0],
            T[0, 0]
        )

        cy = np.cos(yaw * 0.5)

        sy = np.sin(yaw * 0.5)

        t.transform.rotation.w = cy
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy

        # publish
        self.tf_broadcaster.sendTransform(t)

    # =================================================
    # POSE -> MATRIX
    # =================================================

    def pose_to_matrix(self, msg):

        q = msg.pose.pose.orientation

        R = quat2mat([
            q.w,
            q.x,
            q.y,
            q.z
        ])

        T = np.eye(4)

        T[:3, :3] = R

        T[0, 3] = msg.pose.pose.position.x

        T[1, 3] = msg.pose.pose.position.y

        T[2, 3] = msg.pose.pose.position.z

        return T

    # =================================================
    # TF -> MATRIX
    # =================================================

    def transform_to_matrix(self, tf):

        q = tf.transform.rotation

        R = quat2mat([
            q.w,
            q.x,
            q.y,
            q.z
        ])

        T = np.eye(4)

        T[:3, :3] = R

        T[0, 3] = tf.transform.translation.x

        T[1, 3] = tf.transform.translation.y

        T[2, 3] = tf.transform.translation.z

        return T


# =====================================================
# MAIN
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


# =====================================================

if __name__ == "__main__":

    main()