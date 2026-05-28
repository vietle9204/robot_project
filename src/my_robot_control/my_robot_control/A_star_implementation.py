#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, OccupancyGrid

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
import heapq
import threading
import math
import cv2


qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)


class AStarNode(Node):
    def __init__(self):
        super().__init__('a_star_node')

        # ================= PARAMETERS =================
        self.declare_parameter('robot_radius', 0.20)
        self.declare_parameter('occupied_threshold', 65)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('path_downsample', 3)

        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.allow_unknown = bool(self.get_parameter('allow_unknown').value)
        self.path_downsample = int(self.get_parameter('path_downsample').value)

        # ================= ROS INTERFACE =================
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/ekf_slam/pose',
            self.pose_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/ekf_slam/binary_map',
            self.map_callback,
            qos
        )

        # ================= INTERNAL STATES =================
        self.map_received = False
        self.start = None
        self.goal = None

        self.grid = None
        self.resolution = None
        self.origin = None
        self.rows = 0
        self.cols = 0

        self._plan_lock = threading.Lock()
        self._map_lock = threading.Lock()

        self.get_logger().info("AStarNode initialized.")

    # =====================================================
    # MAP CALLBACK
    # =====================================================
    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info("Received map from /map topic")

        resolution = msg.info.resolution
        origin = [
            msg.info.origin.position.x,
            msg.info.origin.position.y,
            0.0
        ]

        cols = msg.info.width
        rows = msg.info.height

        data = np.array(msg.data, dtype=np.int16).reshape((rows, cols))

        grid = np.zeros((rows, cols), dtype=np.int8)

        # ROS OccupancyGrid:
        # -1 unknown
        # 0 free
        # 1~100 probability occupied
        grid[data == 0] = 0
        grid[data >= self.occupied_threshold] = 1
        grid[data < 0] = -1

        # Flip để khớp với công thức world_to_grid đang dùng
        grid = np.flipud(grid)

        # Inflate obstacle theo robot_radius
        inflate_cells = max(1, int(math.ceil(self.robot_radius / resolution)))
        kernel_size = inflate_cells * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)

        obs = (grid == 1).astype(np.uint8)
        inflated = cv2.dilate(obs, kernel, iterations=1)

        # Chỉ inflate vào vùng free
        mask = (inflated == 1) & (grid == 0)
        grid[mask] = 1

        with self._map_lock:
            self.grid = grid
            self.resolution = resolution
            self.origin = origin
            self.cols = cols
            self.rows = rows
            self.map_received = True

        self.get_logger().info(
            f"Map updated: rows={rows}, cols={cols}, "
            f"res={resolution:.3f}, inflate_cells={inflate_cells}"
        )

    # =====================================================
    # POSE CALLBACK
    # =====================================================
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.start = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )

    # =====================================================
    # GOAL CALLBACK
    # =====================================================
    def goal_callback(self, msg: PoseStamped):
        self.goal = (
            msg.pose.position.x,
            msg.pose.position.y
        )

        self.get_logger().info(
            f"Received goal world: x={self.goal[0]:.3f}, y={self.goal[1]:.3f}"
        )

        threading.Thread(target=self._plan_thread, daemon=True).start()

    # =====================================================
    # THREAD WRAPPER
    # =====================================================
    def _plan_thread(self):
        if not self._plan_lock.acquire(blocking=False):
            self.get_logger().warn("Planner is busy. Skip this goal.")
            return

        try:
            self.try_plan()
        finally:
            self._plan_lock.release()

    # =====================================================
    # COORDINATE CONVERSION
    # =====================================================
    def world_to_grid(self, x, y, rows, resolution, origin):
        gx = int((x - origin[0]) / resolution)
        gy = int((y - origin[1]) / resolution)

        # Vì grid đã flip theo trục y
        gy = rows - gy - 1

        return gx, gy

    def grid_to_world(self, gx, gy, rows, resolution, origin):
        x = gx * resolution + origin[0] + resolution / 2.0
        y = (rows - gy - 1) * resolution + origin[1] + resolution / 2.0

        return x, y

    # =====================================================
    # MAIN PLANNER
    # =====================================================
    def try_plan(self):
        if not self.map_received:
            self.get_logger().warn("Map not received yet.")
            return

        if self.start is None:
            self.get_logger().warn("No robot pose from /ekf_slam/pose yet.")
            return

        if self.goal is None:
            self.get_logger().warn("No goal received yet.")
            return

        # Copy map để tránh map_callback sửa grid khi A* đang chạy
        with self._map_lock:
            grid = self.grid.copy()
            rows = self.rows
            cols = self.cols
            resolution = self.resolution
            origin = list(self.origin)

        start_grid = self.world_to_grid(
            self.start[0],
            self.start[1],
            rows,
            resolution,
            origin
        )

        goal_grid = self.world_to_grid(
            self.goal[0],
            self.goal[1],
            rows,
            resolution,
            origin
        )

        self.get_logger().info(
            f"Start grid: {start_grid}, Goal grid: {goal_grid}"
        )

        sx, sy = start_grid
        gx, gy = goal_grid

        if not self.is_inside(sx, sy, cols, rows):
            self.get_logger().error(f"Start out of bounds: {start_grid}")
            return

        if not self.is_inside(gx, gy, cols, rows):
            self.get_logger().error(f"Goal out of bounds: {goal_grid}")
            return

        self.get_logger().info(
            f"Grid value start={int(grid[sy, sx])}, goal={int(grid[gy, gx])}"
        )

        if not self.is_free(grid, sx, sy):
            self.get_logger().warn("Start is not free.")
            return

        if not self.is_free(grid, gx, gy):
            self.get_logger().warn("Goal is not free.")
            return

        path_grid = self.a_star(
            grid,
            start_grid,
            goal_grid,
            rows,
            cols
        )

        if path_grid is None:
            self.get_logger().warn("No path found.")
            return

        path_grid = self.reduce_path_line_of_sight(path_grid, grid, rows, cols)

        if self.path_downsample > 1 and len(path_grid) > 2:
            path_grid = path_grid[::self.path_downsample] + [path_grid[-1]]

        self.publish_path(path_grid, rows, resolution, origin)

    # =====================================================
    # BASIC CHECKS
    # =====================================================
    def is_inside(self, x, y, cols, rows):
        return 0 <= x < cols and 0 <= y < rows

    def is_free(self, grid, x, y):
        if self.allow_unknown:
            return grid[y, x] != 1

        return grid[y, x] == 0

    # =====================================================
    # A STAR
    # =====================================================
    def a_star(self, grid, start, goal, rows, cols):
        sx, sy = start
        gx, gy = goal

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))

        came_from = {}
        g_score = {(sx, sy): 0.0}

        closed_set = set()

        neighbors8 = [
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue

            closed_set.add(current)

            if current == (gx, gy):
                return self.reconstruct_path(came_from, current)

            cx, cy = current

            for dx, dy in neighbors8:
                nx = cx + dx
                ny = cy + dy

                if not self.is_inside(nx, ny, cols, rows):
                    continue

                if not self.is_free(grid, nx, ny):
                    continue

                # Chống cắt góc khi đi chéo
                if dx != 0 and dy != 0:
                    if not self.is_free(grid, cx + dx, cy):
                        continue
                    if not self.is_free(grid, cx, cy + dy):
                        continue

                move_cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
                tentative_g = g_score[current] + move_cost

                neighbor = (nx, ny)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f_score = tentative_g + self.octile_heuristic(
                        neighbor,
                        goal
                    )

                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def octile_heuristic(self, a, b):
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])

        return (dx + dy) + (math.sqrt(2.0) - 2.0) * min(dx, dy)

    def reconstruct_path(self, came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()
        return path

    # =====================================================
    # PATH SMOOTHING SIMPLE: LINE OF SIGHT PRUNING
    # =====================================================
    def reduce_path_line_of_sight(self, path, grid, rows, cols):
        if len(path) <= 2:
            return path

        reduced = [path[0]]
        current_index = 0

        while current_index < len(path) - 1:
            next_index = len(path) - 1

            while next_index > current_index + 1:
                if self.has_line_of_sight(
                    path[current_index],
                    path[next_index],
                    grid,
                    rows,
                    cols
                ):
                    break

                next_index -= 1

            reduced.append(path[next_index])
            current_index = next_index

        return reduced

    def has_line_of_sight(self, p1, p2, grid, rows, cols):
        x0, y0 = p1
        x1, y1 = p2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        x = x0
        y = y0

        while True:
            if not self.is_inside(x, y, cols, rows):
                return False

            if not self.is_free(grid, x, y):
                return False

            if x == x1 and y == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x += sx

            if e2 < dx:
                err += dx
                y += sy

        return True

    # =====================================================
    # PUBLISH PATH
    # =====================================================
    def publish_path(self, path_grid, rows, resolution, origin):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Final path length: {len(path_grid)}")

        for cx, cy in path_grid:
            wx, wy = self.grid_to_world(
                cx,
                cy,
                rows,
                resolution,
                origin
            )

            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published /astar_path.")


def main(args=None):
    rclpy.init(args=args)

    node = AStarNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
