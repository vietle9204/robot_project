#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, QoSProfile

from sensor_msgs.msg import LaserScan, PointCloud2
from laser_geometry import LaserProjection
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=20
)

class ScanToCloudPoints(Node):
    def __init__(self):
        super().__init__('scan_to_cloudpoints')

        self.projector = LaserProjection()

        self.sub = self.create_subscription(
            LaserScan,
            '/robot1/scan',
            self.scan_callback,
            10
        )

        self.pub = self.create_publisher(
            PointCloud2,
            '/scan/cloudpoints',
            10
        )

        self.get_logger().info('Scan → PointCloud2 node started')

    def scan_callback(self, scan_msg: LaserScan):
        cloud = self.projector.projectLaser(scan_msg)
        cloud.header.stamp = scan_msg.header.stamp
        cloud.header.frame_id = scan_msg.header.frame_id
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloudPoints()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
