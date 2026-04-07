#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from transforms3d.quaternions import quat2mat, mat2quat
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class MapToOdom(Node):
    def __init__(self):
        super().__init__("map_to_odom")

        # Nạp tham số từ file YAML (Sửa lỗi "không nhận param")
        self.declare_parameter("topic", "/ekf_slam/pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")

        self.topic = self.get_parameter("topic").value
        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.topic,
            self.pose_cb,
            qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(f"Bridge started: {self.topic} -> TF {self.map_frame}->{self.odom_frame}")

    def pose_cb(self, msg):
        T_map_base = self.pose_to_matrix(msg)
        
        # Sử dụng thời gian của chính message để tránh robot bị giật
        stamp_msg = msg.header.stamp

        try:
            # Tìm vị trí robot trong odom TẠI THỜI ĐIỂM EKF phát ra
            tf_odom_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                stamp_msg, 
                # timeout=Duration(seconds=0.0001)
            )
        except Exception as e:
            # Nếu không tìm được TF cũ, lấy cái mới nhất (fallback)
            try:
                tf_odom_base = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, Time())
            except:
                return

        T_odom_base = self.transform_to_matrix(tf_odom_base)

        # Tính toán: map->odom = map->base * (odom->base)^-1
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)

        self.publish_map_to_odom(T_map_odom, stamp_msg)

    def publish_map_to_odom(self, T, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame

        t.transform.translation.x = T[0, 3]
        t.transform.translation.y = T[1, 3]
        t.transform.translation.z = 0.0 

        # Giữ nguyên góc quay từ ma trận T (bao gồm cả Roll/Pitch/Yaw)
        # Nếu muốn mượt hơn cho Nav2, dùng yaw-only như code của bạn
        q = mat2quat(T[:3, :3]) 
        
        t.transform.rotation.w = q[0]
        t.transform.rotation.x = q[1]
        t.transform.rotation.y = q[2]
        t.transform.rotation.z = q[3]

        self.tf_broadcaster.sendTransform(t)

    def pose_to_matrix(self, msg):
        q = msg.pose.pose.orientation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = msg.pose.pose.position.x
        T[1, 3] = msg.pose.pose.position.y
        T[2, 3] = msg.pose.pose.position.z
        return T

    def transform_to_matrix(self, tf):
        q = tf.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        T = np.eye(4)
        T[:3, :3] = R
        T[0, 3] = tf.transform.translation.x
        T[1, 3] = tf.transform.translation.y
        T[2, 3] = tf.transform.translation.z
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
