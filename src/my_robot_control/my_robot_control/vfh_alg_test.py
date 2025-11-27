#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

def normalize_angle(a):
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return a

class VFHNode(Node):
    def __init__(self):
        super().__init__('vfh_controller')
        # Declare Topic Name Parameters
        self.declare_param()

        # load parameter
        self.load_parameters()

        # subscribers
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_cb, 10)
        
        # publisher
        if(self.use_Stamped):
            self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_vel_topic, 10)
        else:
            self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # parameters
        self.timer = self.create_timer(0.1, self.control_loop)  # 10 Hz
        self.prev_angle = 0.0

        # state
        self.scan = None
        self.odom = None
        self.goal = None

        self.reverse_lidar = True    #cấu hình lidar ngược hướng với base_link

        self.get_logger().info("VFH controller node initialized.")

    def declare_param(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odometry/data')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('use_Stamped', False)

        # robot parameter
        self.declare_parameter('safety_dist', 0.25)
        self.declare_parameter('influence_dist', 1.0)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_omega', 1.0)
        self.declare_parameter('sector_count', 72)
        self.declare_parameter('goal_tolerance', 0.1)
        # self.robot_radius = 0.11

        #cost parameters
        self.declare_parameter('w_goal', 1.0)
        self.declare_parameter('w_obs', 2.0)
        self.declare_parameter('w_turn', 0.5)
        self.declare_parameter('neighbor_range', 10)        #so sector hai ben
        self.declare_parameter('neighbor_penalty_base', 4.0)
        self.declare_parameter('neighbor_penalty_step', 0.6)

    def load_parameters(self):
        # Topic names
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.use_Stamped = self.get_parameter('use_Stamped').value

        # VFH parameters
        self.safety_dist = self.get_parameter('safety_dist').value
        self.influence_dist = self.get_parameter('influence_dist').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_omega = self.get_parameter('max_omega').value
        self.sector_count = self.get_parameter('sector_count').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # cost parameters
        self.w_goal = self.get_parameter('w_goal').value
        self.w_obs = self.get_parameter('w_obs').value
        self.w_turn = self.get_parameter('w_turn').value
        self.neighbor_range = self.get_parameter('neighbor_range').value
        self.neighbor_penalty_base = self.get_parameter('neighbor_penalty_base').value
        self.neighbor_penalty_step = self.get_parameter('neighbor_penalty_step').value

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    def odom_cb(self, msg: Odometry):
        self.odom = msg

    def goal_cb(self, msg: PoseStamped):
        self.goal = msg

    def get_yaw_from_quat(self, q):
        # q: geometry_msgs/Quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.scan is None or self.odom is None or self.goal is None:
            return

        # robot pose
        rx = self.odom.pose.pose.position.x
        ry = self.odom.pose.pose.position.y
        q = self.odom.pose.pose.orientation
        yaw = self.get_yaw_from_quat(q)

        # goal vector in world frame -> robot frame
        gx = self.goal.pose.position.x
        gy = self.goal.pose.position.y
        dx = gx - rx
        dy = gy - ry
        dist_to_goal = math.hypot(dx, dy)
        if dist_to_goal < self.goal_tolerance:
            # reached
            if self.use_Stamped:
                twist = TwistStamped()
                twist.header.stamp = self.get_clock().now().to_msg()
            else:
                twist = Twist()

            self.cmd_pub.publish(twist)
            self.get_logger().info("Goal reached.")
            self.goal = None  # clear goal
            return

        # angle to goal in robot frame
        angle_to_goal_world = math.atan2(dy, dx)
        angle_to_goal = normalize_angle(angle_to_goal_world - yaw)

        # prepare sectors
        scan = self.scan
        if self.reverse_lidar:
            scan.angle_min = 0.0                #theo góc nhìn của base_link
            scan.angle_max = 2.0 * math.pi 

        ranges = np.array(scan.ranges, dtype=np.float32)    # replace inf/nan
        ranges = np.where(np.isfinite(ranges), ranges, scan.range_max)
        angles = np.linspace(scan.angle_min, scan.angle_max, ranges.size)
        if self.reverse_lidar:
            # angles = np.where(angles > math.pi, angles - 2*math.pi, angles) # normalize to [-pi, pi]
            angles = (angles + np.pi) % (2 * np.pi) - np.pi

        # build histogram
        sector_angles = np.linspace(scan.angle_min, scan.angle_max, self.sector_count)
        if self.reverse_lidar:
            # sector_angles = np.where(sector_angles > math.pi, sector_angles - 2*math.pi, sector_angles) # normalize to [-pi, pi]
            sector_angles = (sector_angles + np.pi) % (2 * np.pi) - np.pi
        sector_width = (sector_angles[1] - sector_angles[0]) if self.sector_count > 1 else (scan.angle_max - scan.angle_min)
        hist = np.zeros(self.sector_count, dtype=np.float32)

        for i, a_center in enumerate(sector_angles):
            # take points inside sector
            mask = np.abs(normalize_angle(angles - a_center)) <= (sector_width / 2.0 + 1e-9)
            if not np.any(mask):
                hist[i] = 0.0
            else:
                sec_ranges = ranges[mask]
                min_r = np.min(sec_ranges)
                # influence: closer obstacles yield larger magnitude in [0,1]
                if min_r >= self.influence_dist:
                    mag = 0.0
                else:
                    if min_r < self.safety_dist:
                        mag = 1.0  
                    else:
                        mag = (self.influence_dist - min_r) / (self.influence_dist - self.safety_dist)
                    mag = min(1.0, max(0.0, mag))
                hist[i] = mag

        # cost: combine goal alignment and obstacle penalty
        
        costs = np.zeros_like(hist)
        for i, a in enumerate(sector_angles):
            ang_diff = abs(normalize_angle(a - angle_to_goal))       # phạt hướng lệch so với mục tiêu
            turn_diff = abs(normalize_angle(a - self.prev_angle))    # phạt hướng lệch lớn
            costs[i] += self.w_goal * ang_diff + self.w_obs * hist[i] + self.w_turn * turn_diff   

        # mở rộng penalty sang các sector lân cận
        #for i, mag in enumerate(hist):
            if hist[i] > 0.7:  # sector gần vật cản
                # tăng chi phí cho chính sector
                costs[i] += self.neighbor_penalty_base
                # tăng chi phí cho các sector lân cận
                for offset in range(1, self.neighbor_range+1):
                    penalty = self.neighbor_penalty_base * (self.neighbor_penalty_step ** (offset - 1))  # giảm 15% mỗi bước
                    if i - offset >= 0:
                        costs[i - offset] += penalty
                    else:
                        costs[i - offset + len(costs)] += penalty
                    if i + offset < len(costs):
                        costs[i + offset] += penalty
                    else:
                        costs[i + offset - len(costs)] += penalty

        # pick best sector with minimum cost
        best_idx = int(np.argmin(costs))
        chosen_angle = sector_angles[best_idx]
        self.prev_angle = chosen_angle
        self.get_logger().info(f"Chosen sector angle: {math.degrees(chosen_angle):.1f} deg, cost: {costs[best_idx]:.2f}")

        # compute angular command (in robot frame)
        angular_error = normalize_angle(chosen_angle)
        kp_ang = 1.5
        omega = max(-self.max_omega, min(self.max_omega, kp_ang * angular_error))

        # linear speed reduced when turning and when obstacles in front
        front_mask = np.abs(normalize_angle(angles)) < math.radians(10)
        front_min = np.min(ranges[front_mask]) if np.any(front_mask) else self.influence_dist
        front_penalty = 0.0
        if front_min < self.safety_dist + 0.05:
            front_penalty = 0.05
        else:
            # scale 0..1
            front_penalty = min(1.0, (front_min - self.safety_dist) / (self.influence_dist - self.safety_dist + 1e-6))

        linear = self.max_speed * front_penalty * math.exp(-abs(angular_error)*3)

        # slow as we approach goal
        linear = linear * min(1.0, dist_to_goal / 1.0)

        # clamp small values
        if abs(linear) < 0.01:
            linear = 0.0
        if abs(omega) < 0.01:
            omega = 0.0 

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

        self.get_logger().info(f"VFH command: linear={linear:.2f}, angular={omega:.2f}")    


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
