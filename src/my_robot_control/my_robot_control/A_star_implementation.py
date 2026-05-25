# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import Path, Odometry, OccupancyGrid
# import yaml
# import numpy as np
# from PIL import Image
# import heapq
# import os
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from ament_index_python.packages import get_package_share_directory
# import threading
# import math
# import cv2

# qos = QoSProfile(
#     reliability=ReliabilityPolicy.BEST_EFFORT,
#     durability=DurabilityPolicy.VOLATILE,
#     depth=10
# )

# class AStarNode(Node):
#     def __init__(self):
#         super().__init__('a_star_node')

#         # # ---------- load yaml + pgm ----------
#         # pkg_path = get_package_share_directory('my_robot_control')
#         # default_yaml_path = os.path.join(pkg_path, 'maps', 'my_map.yaml')
#         # self.declare_parameter('map_file', default_yaml_path)
#         # yaml_path = self.get_parameter('map_file').value
#         # self.get_logger().info(f"Loading map from: {yaml_path}")

#         # if not os.path.exists(yaml_path):
#         #     self.get_logger().error(f"Map file not found at: {yaml_path}")
#         #     raise FileNotFoundError(f"Map file not found: {yaml_path}")

#         # with open(yaml_path, 'r') as file:
#         #     map_data = yaml.safe_load(file)

#         # # ensure pgm path resolves relative to yaml
#         # pgm_path = map_data.get('image')
#         # if not os.path.isabs(pgm_path):
#         #     pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)
#         # pgm_path = os.path.normpath(pgm_path)
#         # if not os.path.exists(pgm_path):
#         #     self.get_logger().error(f"PGM file not found at: {pgm_path}")
#         #     raise FileNotFoundError(f"PGM file not found: {pgm_path}")

#         # self.resolution = float(map_data['resolution'])
#         # self.origin = map_data['origin']  # [x, y, yaw]
#         # self.occupied_thresh = float(map_data.get('occupied_thresh', 0.65))
#         # self.free_thresh = float(map_data.get('free_thresh', 0.196))

#         # # load image -> normalize
#         # img = Image.open(pgm_path).convert('L')
#         # img = np.array(img, dtype=np.float32) / 255.0
#         # self.rows, self.cols = img.shape  # rows = height, cols = width
        
#         # # build grid:
#         # self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)
#         # self.grid[img >= self.occupied_thresh] = 0   #free
#         # self.grid[img <= self.free_thresh] = 1      #occoupied
#         # self.grid[(img > self.free_thresh) & (img < self.occupied_thresh)] = -1
#         # # flip vertically to match world->grid convention used later
#         # self.grid = np.flipud(self.grid)
#         # # self.grid = np.fliplr(self.grid)

#         # # mở rộng ô vật cản
#         # kernel = np.ones((5,5), np.uint8)
#         # obs = (self.grid == 1).astype(np.uint8)
#         # inflated = cv2.dilate(obs, kernel)
#         # mask = (inflated == 1) & (self.grid == 0)
#         # self.grid[mask] = 1

#         # self.get_logger().info(f"Map loaded: rows={self.rows}, cols={self.cols}, res={self.resolution}, origin={self.origin}")

#         # ---------- ROS interface ----------
#         self.path_pub = self.create_publisher(Path, '/astar_path', 10)
#         # self.create_subscription(Odometry, '/odometry/data', self.odom_cb, qos)
#         self.create_subscription(PoseWithCovarianceStamped, '/ekf_slam/pose', self.amcl_cb, 10)
#         self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

#         self.create_subscription(
#             OccupancyGrid,
#             'ekf_slam/binary_map',
#             self.map_callback,
#             qos
#         )

#         self.map_received = False
        

#         # start = odom, goal from topic
#         self.start = None
#         self.goal = None

#         # lock to prevent concurrent planners
#         self._lock = threading.Lock()

#         self.get_logger().info("AStarNode initialized.")
        

#     # world -> grid
#     def world_to_grid(self, x, y):
#         gx = int((x - self.origin[0]) / self.resolution)
#         gy = int((y - self.origin[1]) / self.resolution)

#         gy = self.rows - gy - 1

#         return gx, gy

#     # grid -> world (center of cell)
#     def grid_to_world(self, gx, gy):
#         # x = gx * self.resolution + self.origin[0] + self.resolution / 2.0
#         # y = (self.rows - gy - 1) * self.resolution + self.origin[1] + self.resolution / 2.0
#         # return x, y
#         x = gx * self.resolution + self.origin[0] + self.resolution / 2
#         y = (self.rows - gy - 1) * self.resolution + self.origin[1] + self.resolution / 2
#         return x, y
    
#     def map_callback(self, msg: OccupancyGrid):

#         # if self.map_received:
#         #     return

#         self.get_logger().info("Received map from /map topic")

#         self.resolution = msg.info.resolution
#         self.origin = [
#             msg.info.origin.position.x,
#             msg.info.origin.position.y,
#             0.0
#         ]

#         self.cols = msg.info.width
#         self.rows = msg.info.height

#         data = np.array(msg.data, dtype=np.int8).reshape((self.rows, self.cols))

#         # ROS occupancy convention
#         # -1 unknown
#         # 0 free
#         # 100 occupied

#         self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)

#         self.grid[data == 0] = 0        # free
#         self.grid[data == 100] = 1      # obstacle
#         self.grid[data == -1] = -1      # unknown

#         # flip vertical to match coordinate
#         self.grid = np.flipud(self.grid)

#         # obstacle inflation
#         kernel = np.ones((5,5), np.uint8)
#         obs = (self.grid == 1).astype(np.uint8)
#         inflated = cv2.dilate(obs, kernel)

#         mask = (inflated == 1) & (self.grid == 0)
#         self.grid[mask] = 1

#         self.map_received = True

#         self.get_logger().info(
#             f"Map received: rows={self.rows}, cols={self.cols}, res={self.resolution}"
#         )

#     # def odom_cb(self, msg: Odometry):
#     #     # update start from odometry
#     #     self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

#     def amcl_cb(self, msg: PoseWithCovarianceStamped):
#         self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)


#     def goal_callback(self, msg: PoseStamped):
#         self.goal = (msg.pose.position.x, msg.pose.position.y)
#         self.get_logger().info(f"Received goal (world): {self.goal}")

#         # run planner in background thread
#         threading.Thread(target=self._plan_thread, daemon=True).start()

#     def _plan_thread(self):
#         if not self._lock.acquire(blocking=False):
#             self.get_logger().warn("Planner busy — skipping this goal.")
#             return
#         try:
#             self.try_plan()
#         finally:
#             self._lock.release()

#     def try_plan(self):
#         if not self.map_received:
#             self.get_logger().warn("Map not received yet.")
#             return
#         # require both start (odom) and goal
#         if self.start is None:
#             self.get_logger().warn("No amcl_pose . waitting")
#             return
#         if self.goal is None:
#             self.get_logger().warn("No goal set.")
#             return

#         # convert to grid coordinates
#         try:
#             start_grid = self.world_to_grid(*self.start)
#             goal_grid = self.world_to_grid(*self.goal)
#         except Exception as e:
#             self.get_logger().error(f"Error world->grid: {e}")
#             return

#         self.get_logger().info(f"Start grid: {start_grid}, Goal grid: {goal_grid} (cols={self.cols}, rows={self.rows})")

#         sx, sy = start_grid
#         gx, gy = goal_grid
#         # bounds check
#         if not (0 <= sx < self.cols and 0 <= sy < self.rows):
#             self.get_logger().error(f"Start out of bounds: {start_grid}")
#             return
#         if not (0 <= gx < self.cols and 0 <= gy < self.rows):
#             self.get_logger().error(f"Goal out of bounds: {goal_grid}")
#             return

#         # log cell values (debug)
#         self.get_logger().info(f"Grid value at start = {int(self.grid[sy, sx])}, at goal = {int(self.grid[gy, gx])}")

#         # if start/goal in obstacle -> abort
#         if int(self.grid[sy, sx]) == 1:
#             self.get_logger().warn("Start is in obstacle!")
#             return
#         if int(self.grid[gy, gx]) == 1:
#             self.get_logger().warn("Goal is in obstacle!")
#             return

#         path_grid = self.a_star(self.grid, start_grid, goal_grid)
#         if path_grid is None:
#             self.get_logger().warn("No path found!")
#             return

#         # build Path messageS
#         path_msg = Path()
#         path_msg.header.frame_id = "map"
#         path_msg.header.stamp = self.get_clock().now().to_msg()

#         self.get_logger().info(f"Path length: {len(path_grid)}")
#         for i, (cx, cy) in enumerate(path_grid):
#             wx, wy = self.grid_to_world(cx, cy)
#             self.get_logger().info(f"[{i}] waypoint: x={wx:.3f}, y={wy:.3f}")
#             pose = PoseStamped()
#             pose.header.frame_id = "map"
#             pose.header.stamp = self.get_clock().now().to_msg()
#             pose.pose.position.x = wx
#             pose.pose.position.y = wy
#             pose.pose.position.z = 0.0
#             pose.pose.orientation.w = 1.0
#             path_msg.poses.append(pose)

#         self.path_pub.publish(path_msg)
#         self.get_logger().info("Published Path message.")

#     def a_star(self, grid, start, goal):
#         sx, sy = start
#         gx, gy = goal

#         # safety checks
#         if not (0 <= sx < self.cols and 0 <= sy < self.rows):
#             self.get_logger().error("Start out of bounds in a_star.")
#             return None
#         if not (0 <= gx < self.cols and 0 <= gy < self.rows):
#             self.get_logger().error("Goal out of bounds in a_star.")
#             return None

#         if grid[sy, sx] == 1 or grid[gy, gx] == 1:
#             # already logged earlier, but keep defensive
#             return None

#         open_set = []
#         heapq.heappush(open_set, (0.0, (sx, sy)))
#         came_from = {}
#         g_score = {(sx, sy): 0.0}
#         visited = set()

#         neighbors8 = [
#             (1, 0), (-1, 0), (0, 1), (0, -1),
#             (1, 1), (1, -1), (-1, 1), (-1, -1),
#         ]

#         def heuristic(a, b):
#             # Euclidean or Manhattan; keep Manhattan for speed
#             return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
#             # return abs(a[0] - b[0]) + abs(a[1] - b[1])

#         while open_set:
#             _, current = heapq.heappop(open_set)

#             if current in visited:
#                 continue
#             visited.add(current)

#             if current == (gx, gy):
#                 # reconstruct
#                 path = [current]
#                 while current in came_from:
#                     current = came_from[current]
#                     path.append(current)
#                 return path[::-1]

#             for dx, dy in neighbors8:
#                 nx, ny = current[0] + dx, current[1] + dy
#                 if not (0 <= nx < self.cols and 0 <= ny < self.rows):
#                     continue
#                 if grid[ny, nx] == 1:
#                     continue  # obstacle
#                 tentative_g = g_score[current] + np.hypot(dx, dy)
#                 if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
#                     g_score[(nx, ny)] = tentative_g
#                     came_from[(nx, ny)] = current
#                     f = tentative_g + heuristic((nx, ny), (gx, gy))
#                     heapq.heappush(open_set, (f, (nx, ny)))
#         return None

# def main(args=None):
#     rclpy.init(args=args)
#     node = AStarNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import Bool

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)


def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class VFHNode(Node):
    def __init__(self):
        super().__init__('vfh_controller')

        self.declare_param()
        self.load_parameters()

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_cb,
            qos
        )

        self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_cb,
            qos
        )

        self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_cb,
            10
        )

        if self.use_Stamped:
            self.cmd_pub = self.create_publisher(
                TwistStamped,
                self.cmd_vel_topic,
                10
            )
        else:
            self.cmd_pub = self.create_publisher(
                Twist,
                self.cmd_vel_topic,
                10
            )

        self.replan_pub = self.create_publisher(
            Bool,
            '/nav/replan_request',
            10
        )

        self.scan = None
        self.odom = None
        self.goal = None

        self.prev_angle = 0.0
        self.reverse_lidar = False

        self.stuck_count = 0
        self.stuck_threshold = 10

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("VFH controller with replan request initialized.")

    # ============================================================
    # Parameters
    # ============================================================
    def declare_param(self):
        self.declare_parameter('scan_topic', '/robot1/scan')
        self.declare_parameter('odom_topic', '/odometry/data')

        # Quan trọng: nhận waypoint từ my_robot_nav
        self.declare_parameter('goal_topic', '/goal_tmp')

        self.declare_parameter('cmd_vel_topic', '/robot1/cmd_vel')
        self.declare_parameter('use_Stamped', False)

        self.declare_parameter('safety_dist', 0.25)
        self.declare_parameter('influence_dist', 1.0)
        self.declare_parameter('max_speed', 0.5)
        self.declare_parameter('max_omega', 1.0)
        self.declare_parameter('sector_count', 72)
        self.declare_parameter('goal_tolerance', 0.08)

        self.declare_parameter('w_goal', 1.0)
        self.declare_parameter('w_obs', 2.0)
        self.declare_parameter('w_turn', 0.5)

        self.declare_parameter('neighbor_range', 10)
        self.declare_parameter('neighbor_penalty_base', 4.0)
        self.declare_parameter('neighbor_penalty_step', 0.6)

    def load_parameters(self):
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.use_Stamped = self.get_parameter('use_Stamped').value

        self.safety_dist = float(self.get_parameter('safety_dist').value)
        self.influence_dist = float(self.get_parameter('influence_dist').value)
        self.max_speed = float(self.get_parameter('max_speed').value)
        self.max_omega = float(self.get_parameter('max_omega').value)
        self.sector_count = int(self.get_parameter('sector_count').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.w_goal = float(self.get_parameter('w_goal').value)
        self.w_obs = float(self.get_parameter('w_obs').value)
        self.w_turn = float(self.get_parameter('w_turn').value)

        self.neighbor_range = int(self.get_parameter('neighbor_range').value)
        self.neighbor_penalty_base = float(
            self.get_parameter('neighbor_penalty_base').value
        )
        self.neighbor_penalty_step = float(
            self.get_parameter('neighbor_penalty_step').value
        )

    # ============================================================
    # Callbacks
    # ============================================================
    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    # ============================================================
    # Utils
    # ============================================================
    def get_yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_zero_cmd(self):
        if self.use_Stamped:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist)
        else:
            self.cmd_pub.publish(Twist())

    def publish_cmd(self, linear, omega):
        if self.use_Stamped:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x = float(linear)
            twist.twist.angular.z = float(omega)
            self.cmd_pub.publish(twist)
        else:
            twist = Twist()
            twist.linear.x = float(linear)
            twist.angular.z = float(omega)
            self.cmd_pub.publish(twist)

    def request_replan(self):
        msg = Bool()
        msg.data = True
        self.replan_pub.publish(msg)

    # ============================================================
    # Main control
    # ============================================================
    def control_loop(self):
        if self.scan is None or self.odom is None or self.goal is None:
            return

        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        yaw = self.get_yaw_from_quat(self.odom.pose.pose.orientation)

        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y

        dx = gx - rx
        dy = gy - ry

        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal < self.goal_tolerance:
            self.publish_zero_cmd()
            self.goal = None
            self.get_logger().info("Local waypoint reached.")
            return

        angle_to_goal_world = math.atan2(dy, dx)
        angle_to_goal = normalize_angle(angle_to_goal_world - yaw)

        scan = self.scan

        ranges = np.array(scan.ranges, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        ranges = np.where(ranges > 0.14, ranges, scan.range_max)

        angles = np.linspace(scan.angle_min, scan.angle_max, ranges.size)

        if self.reverse_lidar:
            angles = (angles + math.pi) % (2.0 * math.pi) - math.pi

        sector_angles = np.linspace(
            scan.angle_min,
            scan.angle_max,
            self.sector_count
        )

        if self.reverse_lidar:
            sector_angles = (sector_angles + math.pi) % (2.0 * math.pi) - math.pi

        sector_width = (
            sector_angles[1] - sector_angles[0]
            if self.sector_count > 1
            else scan.angle_max - scan.angle_min
        )

        hist = np.zeros(self.sector_count, dtype=np.float32)

        for i, a_center in enumerate(sector_angles):
            mask = np.abs(normalize_angle(angles - a_center)) <= (
                sector_width / 2.0 + 1e-9
            )

            if not np.any(mask):
                hist[i] = 0.0
                continue

            sec_ranges = ranges[mask]
            min_r = np.min(sec_ranges)

            if min_r >= self.influence_dist:
                mag = 0.0
            elif min_r < self.safety_dist:
                mag = 1.0
            else:
                mag = (
                    self.influence_dist - min_r
                ) / (
                    self.influence_dist - self.safety_dist
                )

            hist[i] = float(np.clip(mag, 0.0, 1.0))

        costs = np.zeros_like(hist)

        for i, a in enumerate(sector_angles):
            ang_diff = abs(normalize_angle(a - angle_to_goal))
            turn_diff = abs(normalize_angle(a - self.prev_angle))

            costs[i] += (
                self.w_goal * ang_diff
                + self.w_obs * hist[i]
                + self.w_turn * turn_diff
            )

            if hist[i] > 0.7:
                costs[i] += self.neighbor_penalty_base

                for offset in range(1, self.neighbor_range + 1):
                    penalty = self.neighbor_penalty_base * (
                        self.neighbor_penalty_step ** (offset - 1)
                    )

                    costs[(i - offset) % len(costs)] += penalty
                    costs[(i + offset) % len(costs)] += penalty

        best_idx = int(np.argmin(costs))
        chosen_angle = sector_angles[best_idx]
        self.prev_angle = chosen_angle

        angular_error = normalize_angle(chosen_angle)

        kp_ang = 1.5
        omega = np.clip(
            kp_ang * angular_error,
            -self.max_omega,
            self.max_omega
        )

        front_mask = np.abs(normalize_angle(angles)) < math.radians(10.0)
        front_min = (
            np.min(ranges[front_mask])
            if np.any(front_mask)
            else self.influence_dist
        )

        # Kẹt phía trước nhiều chu kỳ thì yêu cầu A* lập lại đường
        if front_min < self.safety_dist + 0.05 and dist_to_goal > self.goal_tolerance:
            self.stuck_count += 1
        else:
            self.stuck_count = 0

        if self.stuck_count >= self.stuck_threshold:
            self.get_logger().warn("Obstacle/stuck detected -> request replan.")
            self.request_replan()
            self.stuck_count = 0

        if front_min < self.safety_dist + 0.05:
            front_penalty = 0.05
        else:
            front_penalty = min(
                1.0,
                (front_min - self.safety_dist)
                / (self.influence_dist - self.safety_dist + 1e-6)
            )

        linear = self.max_speed * front_penalty * math.exp(-abs(angular_error) * 3.0)

        if abs(linear) < 0.01:
            linear = 0.0

        if abs(omega) < 0.01:
            omega = 0.0

        self.publish_cmd(linear, omega)

        self.get_logger().info(
            f"VFH: linear={linear:.2f}, omega={omega:.2f}, "
            f"front={front_min:.2f}, stuck={self.stuck_count}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VFHNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()