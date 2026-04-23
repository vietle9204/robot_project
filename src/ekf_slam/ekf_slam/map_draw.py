#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer

FREE = 0
OCCUPIED = 100
UNKNOWN = -1

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

qos2 = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)
qos_map = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL  
)

class OccupancyMapping(Node):
    def __init__(self):
        super().__init__('occupancy_mapping')

        # ===== Map config =====
        self.resolution = 0.05
        self.width = 100
        self.height = 100
        self.origin_x = -2.5
        self.origin_y = -2.5

        self.expand_step = 2.0
        self.max_draw_range = 5.0

        # ===== Log odds =====
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        self.lo_occ = 1.0
        self.lo_free = -0.3
        self.lo_min = -2.0
        self.lo_max = 20.0

        # ===== State =====
        self.last_pose_used = None

        # ===== Sync subscribers =====
        self.scan_sub = Subscriber(
            self,
            PointCloud2,
            '/scan/cloudpoints',
            qos_profile=qos
        )

        self.pose_sub = Subscriber(
            self,
            PoseWithCovarianceStamped,
            '/ekf_slam/pose',
            qos_profile=qos2
        )

        self.ts = ApproximateTimeSynchronizer(
            [self.scan_sub, self.pose_sub],
            queue_size=10,
            slop=0.025
        )
        self.ts.registerCallback(self.sync_cb)

        # ===== Publisher =====
        self.map_pub = self.create_publisher(OccupancyGrid, '/ekf_slam/binary_map', qos_map)

        self.get_logger().info("Occupancy Mapping FULL Started")

    # =========================
    # MAIN CALLBACK (SYNCED)
    # =========================
    def sync_cb(self, scan_msg, pose_msg):
        pose = pose_msg.pose.pose
        x, y = pose.position.x, pose.position.y
        yaw = self.quat_to_yaw(pose.orientation)

        # ===== Skip nếu robot không di chuyển =====
        if self.last_pose_used is not None:
            dx = x - self.last_pose_used[0]
            dy = y - self.last_pose_used[1]
            dtheta = abs(yaw - self.last_pose_used[2])

            if math.hypot(dx, dy) < 0.05 and dtheta < 0.05:
                return

        # ===== Expand map nếu cần =====
        self.ensure_map_contains(x, y, self.max_draw_range)

        # ===== Update =====
        self.update_map(scan_msg, x, y, yaw)

        # ===== Publish =====
        self.publish_map()

        self.last_pose_used = (x, y, yaw)

    # =========================
    # UPDATE MAP
    # =========================
    def update_map(self, cloud_msg, xr, yr, theta):
        robot_ix, robot_iy = self.world_to_map(xr, yr)
        if not self.in_map(robot_ix, robot_iy):
            return

        points = pc2.read_points(
            cloud_msg,
            field_names=("x", "y"),
            skip_nans=True
        )

        visited = set()

        for i, p in enumerate(points):

            # ===== Downsample (giảm tải CPU) =====
            if i % 5 != 0:
                continue

            xl, yl = p[0], p[1]
            dist = math.hypot(xl, yl)

            if dist > self.max_draw_range:
                continue

            # ===== Transform sang map =====
            xm = xr + xl * math.cos(theta) - yl * math.sin(theta)
            ym = yr + xl * math.sin(theta) + yl * math.cos(theta)

            ix, iy = self.world_to_map(xm, ym)

            if not self.in_map(ix, iy):
                continue

            # ===== Ray tracing =====
            cells = self.bresenham(robot_ix, robot_iy, ix, iy)

            for cx, cy in cells[:-1]:
                if (cx, cy) not in visited:
                    self.log_odds[cy, cx] += self.lo_free
                    visited.add((cx, cy))

            self.log_odds[iy, ix] += self.lo_occ

        self.log_odds = np.clip(self.log_odds, self.lo_min, self.lo_max)

    # =========================
    # MAP UTILS
    # =========================
    def world_to_map(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        return ix, iy

    def in_map(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def quat_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w*q.z + q.x*q.y),
            1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        )

    # =========================
    # BRESENHAM
    # =========================
    def bresenham(self, x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:
            cells.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x0 += sx

            if e2 < dx:
                err += dx
                y0 += sy

        return cells

    # =========================
    # MAP EXPANSION
    # =========================
    def ensure_map_contains(self, x, y, margin):
        min_x = self.origin_x
        min_y = self.origin_y
        max_x = min_x + self.width * self.resolution
        max_y = min_y + self.height * self.resolution

        if not (x - margin < min_x or x + margin > max_x or
                y - margin < min_y or y + margin > max_y):
            return

        self.get_logger().info("Expanding map...")

        cells = int(self.expand_step / self.resolution)

        add_left = cells if x - margin < min_x else 0
        add_right = cells if x + margin > max_x else 0
        add_down = cells if y - margin < min_y else 0
        add_up = cells if y + margin > max_y else 0

        new_width = self.width + add_left + add_right
        new_height = self.height + add_down + add_up

        new_log_odds = np.zeros((new_height, new_width), dtype=np.float32)

        new_log_odds[
            add_down:add_down+self.height,
            add_left:add_left+self.width
        ] = self.log_odds

        self.log_odds = new_log_odds
        self.width = new_width
        self.height = new_height

        self.origin_x -= add_left * self.resolution
        self.origin_y -= add_down * self.resolution

    # =========================
    # PUBLISH
    # =========================
    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0

        occ_grid = np.full(self.log_odds.shape, UNKNOWN, dtype=np.int8)
        occ_grid[self.log_odds > 3.0] = OCCUPIED
        occ_grid[self.log_odds < -1.2] = FREE

        msg.data = occ_grid.ravel().tolist()

        self.map_pub.publish(msg)


def main():
    rclpy.init()
    node = OccupancyMapping()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()