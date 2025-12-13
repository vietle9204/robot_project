# #!/usr/bin/env python3
# import yaml
# import numpy as np
# from PIL import Image
# import heapq
# import os
# from pathlib import Path

# script_dir = Path(__file__).resolve().parent
# src_dir = script_dir.parent.parent 
# yaml_path = src_dir / 'maps' / 'my_map.yaml'
# yaml_path = str(yaml_path)
# print(f"[Method 2] Using relative path: {yaml_path}")

# # Verify file exists
# if not os.path.exists(yaml_path):
#     raise FileNotFoundError(f"Map file not found at: {yaml_path}\nPlease check your workspace structure.")
# print(f"Loading map from: {yaml_path}\n")

# with open(yaml_path, 'r') as file:
#     map_data = yaml.safe_load(file)

# pgm_path = map_data['image']
# resolution = map_data['resolution']
# origin = map_data['origin']
# occupied_thresh = map_data.get('occupied_thresh')
# free_thresh = map_data.get('free_thresh')

# img = Image.open(pgm_path)
# img = np.array(img, dtype=np.float32) / 255.0 # Normalize to [0, 1]

# rows, cols = img.shape

# # Build occupancy grid
# grid = np.zeros((rows, cols), dtype=np.int8)

# grid[img >= occupied_thresh] = 0  # Occupied
# grid[img <= free_thresh] = 1      # Free
# grid[(img > free_thresh) & (img < occupied_thresh)] = -1  # Unknown

# grid = np.flipud(grid)  # Flip vertically to match coordinate system

# # Coordinate transformation functions
# def world_to_grid(x, y):
#     gx = int((x - origin[0]) / resolution)
#     gy = int((y - origin[1]) / resolution)
#     gy = rows - gy - 1  # Flip y-axis
#     return gx, gy

# def grid_to_world(gx, gy):
#     x = gx * resolution + origin[0] + resolution / 2
#     y = (rows - gy - 1) * resolution + origin[1] + resolution / 2
#     return x, y

# # A* Algorithm Implementation
# def a_star(grid, start, goal):
#     sx, sy = start
#     gx, gy = goal

#     if grid[sy, sx] == 1:
#         print("Start is in obstacle!")
#         return None
#     if grid[gy, gx] == 1:
#         print("Goal is in obstacle!")
#         return None

#     rows, cols = grid.shape
#     open_set = []
#     heapq.heappush(open_set, (0, (sx, sy)))

#     came_from = {}
#     g_score = { (sx, sy): 0 }

#     # 8-connected grid
#     neighbors8 = [
#         (1, 0), (-1, 0), (0, 1), (0, -1),
#         (1, 1), (1, -1), (-1, 1), (-1, -1),
#     ]

#     def heuristic(a, b):
#         return abs(a[0]-b[0]) + abs(a[1]-b[1])

#     while open_set:
#         _, current = heapq.heappop(open_set)

#         if current == (gx, gy):
#             # reconstruct path
#             path = [current]
#             while current in came_from:
#                 current = came_from[current]
#                 path.append(current)
#             return path[::-1]

#         for dx, dy in neighbors8:
#             nx, ny = current[0] + dx, current[1] + dy
#             if 0 <= nx < cols and 0 <= ny < rows:
#                 if grid[ny, nx] == 1:
#                     continue  # skip obstacles

#                 new_cost = g_score[current] + np.hypot(dx, dy)
#                 if (nx, ny) not in g_score or new_cost < g_score[(nx, ny)]:
#                     g_score[(nx, ny)] = new_cost
#                     came_from[(nx, ny)] = current
#                     f = new_cost + heuristic((nx, ny), (gx, gy))
#                     heapq.heappush(open_set, (f, (nx, ny)))

#     return None

# # Test with example start and goal
# start_world = (0.0, 0.0)  # Example start in world
# goal_world = (10.32, 9.87)   # Example goal in world

# start_grid = world_to_grid(*start_world)
# goal_grid = world_to_grid(*goal_world)

# print(f"Start grid: {start_grid}, Goal grid: {goal_grid}")
# path = a_star(grid, start_grid, goal_grid)
# if path:
#     path_world = [grid_to_world(x, y) for x, y in path]
#     print("Path found:")
#     for p in path_world:
#         print(f"{p}")
# else:
#     print("No path found.")
    

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path, Odometry
import yaml
import numpy as np
from PIL import Image
import heapq
import os
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
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

        # ---------- load yaml + pgm ----------
        pkg_path = get_package_share_directory('my_robot_control')
        yaml_path = os.path.join(pkg_path, 'maps', 'my_map.yaml')
        self.get_logger().info(f"Loading map from: {yaml_path}")

        if not os.path.exists(yaml_path):
            self.get_logger().error(f"Map file not found at: {yaml_path}")
            raise FileNotFoundError(f"Map file not found: {yaml_path}")

        with open(yaml_path, 'r') as file:
            map_data = yaml.safe_load(file)

        # ensure pgm path resolves relative to yaml
        pgm_path = map_data.get('image')
        if not os.path.isabs(pgm_path):
            pgm_path = os.path.join(os.path.dirname(yaml_path), pgm_path)
        pgm_path = os.path.normpath(pgm_path)
        if not os.path.exists(pgm_path):
            self.get_logger().error(f"PGM file not found at: {pgm_path}")
            raise FileNotFoundError(f"PGM file not found: {pgm_path}")

        self.resolution = float(map_data['resolution'])
        self.origin = map_data['origin']  # [x, y, yaw]
        self.occupied_thresh = float(map_data.get('occupied_thresh', 0.65))
        self.free_thresh = float(map_data.get('free_thresh', 0.196))

        # load image -> normalize
        img = Image.open(pgm_path).convert('L')
        img = np.array(img, dtype=np.float32) / 255.0
        self.rows, self.cols = img.shape  # rows = height, cols = width
        
        # build grid:
        self.grid = np.zeros((self.rows, self.cols), dtype=np.int8)
        self.grid[img >= self.occupied_thresh] = 0   #free
        self.grid[img <= self.free_thresh] = 1      #occoupied
        self.grid[(img > self.free_thresh) & (img < self.occupied_thresh)] = -1
        # flip vertically to match world->grid convention used later
        self.grid = np.flipud(self.grid)
        # self.grid = np.fliplr(self.grid)

        kernel = np.ones((5,5), np.uint8)
        obs = (self.grid == 1).astype(np.uint8)
        inflated = cv2.dilate(obs, kernel)
        mask = (inflated == 1) & (self.grid == 0)
        self.grid[mask] = 1

        self.get_logger().info(f"Map loaded: rows={self.rows}, cols={self.cols}, res={self.resolution}, origin={self.origin}")

        # ---------- ROS interface ----------
        self.path_pub = self.create_publisher(Path, '/astar_path', 10)
        # self.create_subscription(Odometry, '/odometry/data', self.odom_cb, qos)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        # start = odom, goal from topic
        self.start = None
        self.goal = None

        # lock to prevent concurrent planners
        self._lock = threading.Lock()

        self.get_logger().info("AStarNode initialized.")
        

    # world -> grid
    def world_to_grid(self, x, y):
        gx = int((x - self.origin[0]) / self.resolution)
        gy = int((y - self.origin[1]) / self.resolution)
        # flip vertical
        # gy = self.rows - gy - 1
        return gx, gy

    # grid -> world (center of cell)
    def grid_to_world(self, gx, gy):
        # x = gx * self.resolution + self.origin[0] + self.resolution / 2.0
        # y = (self.rows - gy - 1) * self.resolution + self.origin[1] + self.resolution / 2.0
        # return x, y
        x = gx * self.resolution + self.origin[0] + self.resolution / 2
        y = gy * self.resolution + self.origin[1] + self.resolution / 2
        return x, y

    # def odom_cb(self, msg: Odometry):
    #     # update start from odometry
    #     self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def amcl_cb(self, msg: PoseWithCovarianceStamped):
        self.start = (msg.pose.pose.position.x, msg.pose.pose.position.y)


    def goal_callback(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received goal (world): {self.goal}")

        # run planner in background thread
        threading.Thread(target=self._plan_thread, daemon=True).start()

    def _plan_thread(self):
        if not self._lock.acquire(blocking=False):
            self.get_logger().warn("Planner busy — skipping this goal.")
            return
        try:
            self.try_plan()
        finally:
            self._lock.release()

    def try_plan(self):
        # require both start (odom) and goal
        if self.start is None:
            self.get_logger().warn("No amcl_pose . waitting")
            return
        if self.goal is None:
            self.get_logger().warn("No goal set.")
            return

        # convert to grid coordinates
        try:
            start_grid = self.world_to_grid(*self.start)
            goal_grid = self.world_to_grid(*self.goal)
        except Exception as e:
            self.get_logger().error(f"Error world->grid: {e}")
            return

        self.get_logger().info(f"Start grid: {start_grid}, Goal grid: {goal_grid} (cols={self.cols}, rows={self.rows})")

        sx, sy = start_grid
        gx, gy = goal_grid
        # bounds check
        if not (0 <= sx < self.cols and 0 <= sy < self.rows):
            self.get_logger().error(f"Start out of bounds: {start_grid}")
            return
        if not (0 <= gx < self.cols and 0 <= gy < self.rows):
            self.get_logger().error(f"Goal out of bounds: {goal_grid}")
            return

        # log cell values (debug)
        self.get_logger().info(f"Grid value at start = {int(self.grid[sy, sx])}, at goal = {int(self.grid[gy, gx])}")

        # if start/goal in obstacle -> abort
        if int(self.grid[sy, sx]) == 1:
            self.get_logger().warn("Start is in obstacle!")
            return
        if int(self.grid[gy, gx]) == 1:
            self.get_logger().warn("Goal is in obstacle!")
            return

        path_grid = self.a_star(self.grid, start_grid, goal_grid)
        if path_grid is None:
            self.get_logger().warn("No path found!")
            return

        # build Path messageS
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(f"Path length: {len(path_grid)}")
        for i, (cx, cy) in enumerate(path_grid):
            wx, wy = self.grid_to_world(cx, cy)
            self.get_logger().info(f"[{i}] waypoint: x={wx:.3f}, y={wy:.3f}")
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info("Published Path message.")

    def a_star(self, grid, start, goal):
        sx, sy = start
        gx, gy = goal

        # safety checks
        if not (0 <= sx < self.cols and 0 <= sy < self.rows):
            self.get_logger().error("Start out of bounds in a_star.")
            return None
        if not (0 <= gx < self.cols and 0 <= gy < self.rows):
            self.get_logger().error("Goal out of bounds in a_star.")
            return None

        if grid[sy, sx] == 1 or grid[gy, gx] == 1:
            # already logged earlier, but keep defensive
            return None

        open_set = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from = {}
        g_score = {(sx, sy): 0.0}
        visited = set()

        neighbors8 = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ]

        def heuristic(a, b):
            # Euclidean or Manhattan; keep Manhattan for speed
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in visited:
                continue
            visited.add(current)

            if current == (gx, gy):
                # reconstruct
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            for dx, dy in neighbors8:
                nx, ny = current[0] + dx, current[1] + dy
                if not (0 <= nx < self.cols and 0 <= ny < self.rows):
                    continue
                if grid[ny, nx] == 1:
                    continue  # obstacle
                tentative_g = g_score[current] + np.hypot(dx, dy)
                if (nx, ny) not in g_score or tentative_g < g_score[(nx, ny)]:
                    g_score[(nx, ny)] = tentative_g
                    came_from[(nx, ny)] = current
                    f = tentative_g + heuristic((nx, ny), (gx, gy))
                    heapq.heappush(open_set, (f, (nx, ny)))
        return None

def main(args=None):
    rclpy.init(args=args)
    node = AStarNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
