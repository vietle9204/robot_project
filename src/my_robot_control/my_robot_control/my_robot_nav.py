#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose
from tf2_geometry_msgs import do_transform_pose_stamped


qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

class MyRobotNav(Node):
    def __init__(self):
        super().__init__('my_robot_nav')

        # Subscribe path
        self.path_sub = self.create_subscription(Path, '/astar_path', self.path_callback, 10)

        # Publish goal
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_tmp', 10)

        # Subscribe odom
        self.create_subscription(Odometry, '/odometry/data', self.odom_cb, qos)

        # Store current odom
        self.current_pos = None

        # Path and index
        self.path = None
        self.index = 0

        # Timer
        self.timer = self.create_timer(0.05, self.process_navigation)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # self.pub_odom_path = self.create_publisher(Path, "/path_odom", 10)


    # -------------------------
    # Nhận Path mới
    # -------------------------
    def transform_pose(self, pose_stamped, target_frame="odom"):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose_stamped.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        return do_transform_pose_stamped(pose_stamped, transform)


    def path_callback(self, msg: Path):
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty path!")
            return
        
        self.get_logger().info(f"Received new path with {len(msg.poses)} points")

        self.path = msg.poses
        self.index = 0  # reset về điểm đầu tiên

        # Gửi ngay điểm đầu tiên
        new_pose = self.transform_pose(self.path[self.index], "odom")
        self.publish_goal(new_pose)

    
    # def path_callback(self, msg: Path):
    #     if len(msg.poses) == 0:
    #         self.get_logger().warn("Received empty path!")
    #         return

    #     odom_path = Path()
    #     odom_path.header.stamp = self.get_clock().now().to_msg()
    #     odom_path.header.frame_id = "odom"

    #     for p in msg.poses:
    #         new_pose = self.transform_pose(p, "odom")
    #         if new_pose is not None:
    #             odom_path.poses.append(new_pose)

    #     # Publish new path
    #     self.pub_odom_path.publish(odom_path)

    #     self.path = odom_path.poses
    #     self.index = 0

    #     if len(self.path):
    #         self.publish_goal(self.path[self.index])


    # -------------------------
    # Nhận odometry
    # -------------------------
    def odom_cb(self, msg: Odometry):
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # -------------------------
    # Gửi goal
    # -------------------------
    def publish_goal(self, pose_msg: PoseStamped):
        self.goal_pub.publish(pose_msg)
        self.get_logger().info(
            f"Published goal {self.index}: ({pose_msg.pose.position.x:.2f}, {pose_msg.pose.position.y:.2f})"
        )

    # -------------------------
    # Kiểm tra điều hướng
    # -------------------------
    def process_navigation(self):
        if self.path is None:
            return
        
        if self.current_pos is None:
            return

        if self.index >= len(self.path):
            # Đã xong path
            return
        
        new_pose = self.transform_pose(self.path[self.index], "odom")
        goal = new_pose.pose.position

        # Tính khoảng cách tới goal
        dx = goal.x - self.current_pos[0]
        dy = goal.y - self.current_pos[1]
        dist = math.sqrt(dx*dx + dy*dy)

        # Nếu đạt goal → chuyển sang goal tiếp theo
        if dist < 0.22:
            self.index += 1
            if self.index < len(self.path):
                self.publish_goal(new_pose)
            else:
                self.get_logger().info("Reached final goal!")
                



def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNav()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
