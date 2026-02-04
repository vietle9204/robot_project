#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid


FREE = 0
OCCUPIED = 100
UNKNOWN = -1
MAX_DRAW_RANGE = 5.0

class OccupancyMapping(Node):
    def __init__(self):
        super().__init__('occupancy_mapping')

        # ===== Map config =====
        self.resolution = 0.05  # m/cell
        self.width = 100        
        self.height = 100
        self.origin_x = -1.0
        self.origin_y = -1.0
        self.expand_step = 1.0  # mở map mỗi lần 5m
        self.max_draw_range = 5.0


        self.log_odds = np.zeros(
            (self.height, self.width),
            dtype=np.float32
        )
        self.lo_occ = 0.85
        self.lo_free = -0.2
        self.lo_min = -5.0
        self.lo_max = 10.0


        # ===== State =====
        self.latest_scan = None
        self.latest_pose = None
        self.last_pose_used = None

        # ===== ROS =====
        self.create_subscription(
            LaserScan, '/scan', self.scan_cb, 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/ekf_slam/pose', self.pose_cb, 10)

        self.map_pub = self.create_publisher(
            OccupancyGrid, '/map', 1)

        self.get_logger().info("Occupancy Mapping Node Started")

    # ================= CALLBACKS =================

    def scan_cb(self, msg):
        self.latest_scan = msg

    def pose_cb(self, msg):
        if self.latest_scan is None:
            return

        pose = msg.pose.pose   
        x = pose.position.x
        y = pose.position.y
        yaw = self.quat_to_yaw(pose.orientation)

        if self.last_pose_used is not None:
            dx = x - self.last_pose_used[0]
            dy = y - self.last_pose_used[1]
            dtheta = abs(yaw - self.last_pose_used[2])

            if math.hypot(dx, dy) < 0.02 and dtheta < 0.02:
                return

        self.ensure_map_contains(x, y, self.latest_scan.range_max)
        self.update_map(self.latest_scan, x, y, yaw)
        self.publish_map()
        self.last_pose_used = (x, y, yaw)


    # ================= MAP UPDATE =================
    def update_map(self, scan, xr, yr, theta):
        robot_ix, robot_iy = self.world_to_map(xr, yr)

        # Nếu robot ra ngoài map (trường hợp expand chưa kịp)
        if not self.in_map(robot_ix, robot_iy):
            return

        for i, r in enumerate(scan.ranges):

            # 1. Lọc tia lỗi
            if not math.isfinite(r):
                continue
            if r > self.max_draw_range:
                continue

            # 2. Góc tia
            angle = scan.angle_min + i * scan.angle_increment

            # 3. Laser point trong frame robot
            xl = r * math.cos(angle)
            yl = r * math.sin(angle)

            # 4. Transform sang map frame
            xm = xr + xl * math.cos(theta) - yl * math.sin(theta)
            ym = yr + xl * math.sin(theta) + yl * math.cos(theta)

            ix, iy = self.world_to_map(xm, ym)

            if not self.in_map(ix, iy):
                continue

            # 5. Ray tracing
            cells = self.bresenham(robot_ix, robot_iy, ix, iy)

            # Free space
            for cx, cy in cells[:-1]:
                self.log_odds[cy, cx] += self.lo_free

            # Occupied cell
            self.log_odds[iy, ix] += self.lo_occ

        # 6. Chặn log-odds
        self.log_odds = np.clip(
            self.log_odds,
            self.lo_min,
            self.lo_max
        )



    # ================= UTILS =================

    def world_to_map(self, x, y):
        ix = int((x - self.origin_x) / self.resolution)
        iy = int((y - self.origin_y) / self.resolution)
        return ix, iy

    def in_map(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def quat_to_yaw(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

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
    
    def ensure_map_contains(self, x, y, margin):
        min_x = self.origin_x
        max_x = self.origin_x + self.width * self.resolution
        min_y = self.origin_y
        max_y = self.origin_y + self.height * self.resolution

        expand_left = x - margin < min_x
        expand_right = x + margin > max_x
        expand_down = y - margin < min_y
        expand_up = y + margin > max_y

        if not (expand_left or expand_right or expand_down or expand_up):
            return

        self.get_logger().info("Expanding map...")

        new_origin_x = self.origin_x
        new_origin_y = self.origin_y
        add_left = add_right = add_down = add_up = 0

        cells = int(self.expand_step / self.resolution)

        if expand_left:
            add_left = cells
            new_origin_x -= self.expand_step
        if expand_right:
            add_right = cells
        if expand_down:
            add_down = cells
            new_origin_y -= self.expand_step
        if expand_up:
            add_up = cells

        new_width = self.width + add_left + add_right
        new_height = self.height + add_down + add_up

        new_log_odds = np.zeros(
            (new_height, new_width),
            dtype=np.float32
        )

        new_log_odds[
            add_down:add_down + self.height,
            add_left:add_left + self.width
        ] = self.log_odds

        self.log_odds = new_log_odds
        self.width = new_width
        self.height = new_height
        self.origin_x = new_origin_x
        self.origin_y = new_origin_y



    # ================= PUBLISH =================

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

        occ_grid = np.full(
            self.log_odds.shape,
            UNKNOWN,
            dtype=np.int8
        )

        occ_grid[self.log_odds > 1.0] = OCCUPIED
        occ_grid[self.log_odds < -1.0] = FREE

        msg.data = occ_grid.flatten().tolist()
        self.map_pub.publish(msg)



def main():
    rclpy.init()
    node = OccupancyMapping()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
