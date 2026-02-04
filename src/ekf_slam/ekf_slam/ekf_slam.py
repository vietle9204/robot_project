#!/usr/bin/env python3
from pyexpat import features
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField, PointCloud2, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=3
)

def normalize_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi

def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )

def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.
    All angles in radians.
    Return: (qx, qy, qz, qw)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw

class EKFSLAM(Node):
    def __init__(self):
        super().__init__("ekf_slam_node")
        # Declare Topic Name Parameters
        self.declare_param()
        # load parameter
        self.load_parameters()
        # Subcriptions
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ekf_slam/pose", 10)
        self.map_pub = self.create_publisher(MarkerArray, "/ekf_slam/map", 10)
        
        # last odom
        self.last_odom = None
        self.last_odom_time = None

        # State vector [xr, yr, theta, m1x, m1y, m2x, m2y, ...]
        self.x = np.zeros((3, 1))
        # Covariance matrix
        self.P = np.eye(3) * 1e-3
        # Noise
        self.Q = np.diag([0.002, 0.002, 0.002])  # motion noise
        self.R = np.diag([0.02, 0.02])        # measurement noise
        # lamarks
        self.num_landmarks = 0
        self.max_landmarks = 60     # giới hạn số landmark
        self.landmark_score = []     # độ tin cậy

        # measurements in current step
        self.z = []
        self.z_lm_ids = []
        self.new_features = []

        # motion noise params
        self.a1 = 0.5
        self.a2 = 0.5
        self.beta = 0.0

    def declare_param(self):
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odometry/data')

    def load_parameters(self):
        # Topic names
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

    def publish_pose(self, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "map"   # EKF-SLAM 

        # --- Pose ---
        msg.pose.pose.position.x = float(self.x[0, 0])
        msg.pose.pose.position.y = float(self.x[1, 0])
        msg.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, self.x[2, 0])
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        # --- Covariance (6x6) ---
        cov = np.zeros((6, 6))
        cov[0, 0] = self.P[0, 0]   # x
        cov[1, 1] = self.P[1, 1]   # y
        cov[5, 5] = self.P[2, 2]   # yaw

        msg.pose.covariance = cov.flatten().tolist()

        self.pose_pub.publish(msg)    

    def publish_map(self, stamp):
        if self.num_landmarks == 0:
            return

        marker_array = MarkerArray()

        for lm_id in range(self.num_landmarks):
            if lm_id >= len(self.lm_observed):
                continue

            idx = 3 + 2 * lm_id
            mx = float(self.x[idx, 0])
            my = float(self.x[idx + 1, 0])

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "landmarks"
            m.id = lm_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = mx
            m.pose.position.y = my
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0

            m.scale.x = m.scale.y = m.scale.z = 0.15

            m.color.a = 1.0
            if self.lm_observed[lm_id]:
                m.color.g = 1.0
            else:
                m.color.r = 1.0

            marker_array.markers.append(m)

        self.map_pub.publish(marker_array)

    def odom_cb(self, msg: Odometry):
        if self.last_odom is None:
            self.last_odom = msg
            return

        # Compute odometry increments
        yaw_now  = yaw_from_quaternion(msg.pose.pose.orientation)
        yaw_last = yaw_from_quaternion(self.last_odom.pose.pose.orientation)
        dx_odom = msg.pose.pose.position.x - self.last_odom.pose.pose.position.x
        dy_odom = msg.pose.pose.position.y - self.last_odom.pose.pose.position.y 
        dtheta = normalize_angle(yaw_now - yaw_last)

        # --- robot_motion ---
        dx_robot =  math.cos(yaw_now) * dx_odom + math.sin(yaw_now) * dy_odom
        dy_robot = -math.sin(yaw_now) * dx_odom + math.cos(yaw_now) * dy_odom
        self.last_odom = msg

        if abs(dx_robot) < 0.002 and abs(dy_robot) < 0.002 and abs(dtheta) < 0.002:
            dx_robot, dy_robot, dtheta = 0.0, 0.0, 0.0

        # EKF Prediction step
        self.predict((dx_robot, dy_robot, dtheta))


    def scan_cb(self, scan: LaserScan):
        # 1. Nhận scan mới → trích đặc trưng
        features = self.extract_features_from_scan(scan)
        if len(features) == 0: return

        # 2. Data association
        self.association(features)

        # 3. EKF UPDATE (landmark đã quan sát)
        for z, lm_id in zip(self.z, self.z_lm_ids):
            self.update(z, lm_id)
        self.x[2, 0] = normalize_angle(self.x[2, 0])

        # 4. Thêm landmark mới
        for z in self.new_features:
            if self.num_landmarks >= self.max_landmarks:
                self.remove_landmark(np.argmin(self.landmark_score))
            self.add_landmark(z)

        # 5. publish results
        if len(features) >1:
            self.publish_pose(scan.header.stamp)
            self.publish_map(scan.header.stamp)
        self.get_logger().info(f"EKF-SLAM: num_landmarks={self.num_landmarks}, pose=({self.x[0,0]:.4f}, {self.x[1,0]:.4f}, {self.x[2,0]:.4f})")


    # =========================
    #1. PREDICTION STEP
    # delta_x: dx, dy, dtheta ==> robot motion frame
    # =========================
    def predict(self, delta_x):
        dx, dy, dtheta = delta_x
        x, y, theta = self.x[0:3, 0]

        # --- Motion model (xoay delta) ---
        x_pred = x + np.cos(theta) * dx - np.sin(theta) * dy
        y_pred = y + np.sin(theta) * dx + np.cos(theta) * dy
        theta_pred = normalize_angle(theta + dtheta)

        # self.get_logger().info(f"Predict: dx={x_pred:.4f}, dy={y_pred:.4f}, dtheta={theta_pred:.4f}")  
        self.x[0:3, 0] = [x_pred, y_pred, theta_pred]

        # --- Jacobian wrt state ---
        Fx = np.eye(len(self.x))
        Fx[0, 2] = -np.sin(theta_pred) * dx - np.cos(theta_pred) * dy
        Fx[1, 2] =  np.cos(theta_pred) * dx - np.sin(theta_pred) * dy

        # --- Motion noise ---
        Q_full = np.zeros_like(self.P)
        Q_model = np.diag([
            self.a1*(dx*dx + dy*dy),
            self.a1*(dx*dx + dy*dy),
            self.a2*(dtheta*dtheta)
        ])

        # Q_odom = np.diag([
        #     self.last_odom.pose.covariance[0],
        #     self.last_odom.pose.covariance[7],
        #     self.last_odom.pose.covariance[35]
        # ])

        # Q = Q_model #+ self.beta * Q_odom

        Q_full[0:3, 0:3] = Q_model

        self.P = Fx @ self.P @ Fx.T + Q_full

    # =========================
    # 2. UPDATE STEP
    # =========================
    def update(self, z, landmark_id):
        z = np.asarray(z).reshape(2, 1)

        lm = 3 + 2 * landmark_id
        mx, my = self.x[lm:lm+2, 0]

        xr, yr, theta = self.x[0:3, 0]

        dx = mx - xr
        dy = my - yr
        q = dx*dx + dy*dy

        if q < 1e-6:
            return

        sqrt_q = math.sqrt(q)

        z_hat = np.array([
            [sqrt_q],
            [normalize_angle(math.atan2(dy, dx) - theta)]
        ])

        H = np.zeros((2, len(self.x)))

        H[0,0] = -dx / sqrt_q
        H[0,1] = -dy / sqrt_q
        H[1,0] =  dy / q
        H[1,1] = -dx / q
        H[1,2] = -1

        H[0,lm]   =  dx / sqrt_q
        H[0,lm+1] =  dy / sqrt_q
        H[1,lm]   = -dy / q
        H[1,lm+1] =  dx / q

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        y = z - z_hat
        y[1,0] = normalize_angle(y[1,0])

        self.x = self.x + K @ y
        self.x[2,0] = normalize_angle(self.x[2,0])

        I = np.eye(len(self.x))
        self.P = (I - K@H) @ self.P @ (I - K@H).T + K @ self.R @ K.T

    # =========================
    #3. ADD NEW LANDMARK
    # =========================
    def add_landmark(self, z):
        r, b = z
        xr, yr, theta = self.x[0:3, 0]

        phi = theta + b
        mx = xr + r * math.cos(phi)
        my = yr + r * math.sin(phi)

        # append state
        self.x = np.vstack((self.x, [[mx], [my]]))

        n = len(self.x)
        P_new = np.zeros((n, n))
        P_new[:-2, :-2] = self.P

        # Jacobians
        Gx = np.array([
            [1, 0, -r * math.sin(phi)],
            [0, 1,  r * math.cos(phi)]
        ])

        Gz = np.array([
            [math.cos(phi), -r * math.sin(phi)],
            [math.sin(phi),  r * math.cos(phi)]
        ])

        P_rr = self.P[0:3, 0:3]

        # landmark covariance
        P_mm = Gx @ P_rr @ Gx.T + Gz @ self.R @ Gz.T

        # cross-covariance
        P_xr = self.P[:, 0:3]         # cov(state, robot)
        P_rm = P_xr @ Gx.T


        P_new[:-2, -2:] = P_rm
        P_new[-2:, :-2] = P_rm.T
        P_new[-2:, -2:] = P_mm

        self.P = P_new
        self.num_landmarks += 1
        self.landmark_score.append(2.0)
        self.lm_observed = np.append(self.lm_observed, False)


    def remove_landmark(self, lm_id):
        idx = 3 + 2 * lm_id

        # remove state
        self.x = np.delete(self.x, [idx, idx+1], axis=0)

        # remove covariance
        self.P = np.delete(self.P, [idx, idx+1], axis=0)
        self.P = np.delete(self.P, [idx, idx+1], axis=1)

        # remove metadata
        self.landmark_score.pop(lm_id)
        self.num_landmarks -= 1
        self.lm_observed = np.delete(self.lm_observed, lm_id)


    # =========================
    # 4. FEATURE EXTRACTION FROM LASER SCAN
    # =========================
    def extract_features_from_scan(self, scan):
        curv_pts = self.extract_curvature_points(
            scan,
            k=5,
            curvature_threshold=0.2,
            range_min=0.5,
            range_max=10.0
        )

        if len(curv_pts) < 1:
            return []

    
        clusters = self.cluster_features(curv_pts,
                     angle_thresh=0.02,
                     range_thresh=0.5)

        features = [self.cluster_to_feature(c) for c in clusters]
        return features

    def extract_curvature_points(self, scan,
                                k=5,
                                curvature_threshold=0.15,
                                range_min=0.5,
                                range_max=10.0):
        ranges = np.array(scan.ranges)
        angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = angles[valid]

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        curv_points = []

        for i in range(k, len(xs) - k):
            if ranges[i] < range_min or ranges[i] > range_max:
                continue
            neighbors = np.stack([
                xs[i-k:i+k+1],
                ys[i-k:i+k+1]
            ], axis=1)

            curv = self.compute_curvature(neighbors)

            if curv > curvature_threshold:
                curv_points.append((ranges[i], angles[i]))

        return curv_points

    
    def compute_curvature(self, points):
        """
        points: Nx2 array (x, y)
        return curvature in [0, 1]
        """
        mean = np.mean(points, axis=0)
        cov = np.cov((points - mean).T)

        eigvals, _ = np.linalg.eig(cov)
        eigvals = np.sort(eigvals)

        if eigvals.sum() < 1e-6:
            return 0.0

        curvature = eigvals[0] / eigvals.sum()
        return curvature


    def cluster_features(self, points,
                     angle_thresh=0.03,
                     range_thresh=0.5):

        angle_clusters = self.cluster_by_angle(points, angle_thresh)
        final_clusters = []

        for cl in angle_clusters:
            min_size = self.adaptive_min_cluster_size(
                np.mean([p[0] for p in cl]) if cl else 0
            )
            if len(cl) >= min_size:
                cl = self.filter_converged_points(cl, range_thresh)
                final_clusters.append(cl)

        return final_clusters


    def cluster_by_angle(self, points, angle_thresh=0.03):
        clusters = []
        cluster = [points[0]]

        for i in range(1, len(points)):
            db = abs(points[i][1] - points[i-1][1])

            if db < angle_thresh:
                cluster.append(points[i])
            else:
                clusters.append(cluster)
                cluster = [points[i]]

        clusters.append(cluster)
        return clusters
    

    def filter_converged_points(self, cluster, range_thresh=0.5):
        ranges = [p[0] for p in cluster]
        r_min = np.min(ranges)

        filtered = [
            p for p in cluster
            if abs(p[0] - r_min) < range_thresh
        ]
        return filtered


    def cluster_to_feature(self, cluster):
        rs = np.array([p[0] for p in cluster])
        bs = np.array([p[1] for p in cluster])

        r = np.mean(rs)
        b = math.atan2(np.mean(np.sin(bs)), np.mean(np.cos(bs)))

        return (r, b)


    def adaptive_min_cluster_size(self, r):
        if r < 1.0: return 3
        # if r < 3.0: return 2
        return 1


    # =========================
    # 5. DATA ASSOCIATION
    # =========================
    def association(self, features, chi2_threshold=1.5):
        """
        features: list of np.array([r, b])
        """
        self.z = []
        self.z_lm_ids = []
        self.new_features = []

        # --- NEW: đánh dấu landmark được quan sát ---
        self.lm_observed = np.zeros(self.num_landmarks, dtype=bool)

        # : tính tất cả association candidate
        pairs = []  # (d2, feat_id, lm_id)

        for i, z in enumerate(features):
            for lm_id in range(self.num_landmarks):
                d2 = self.compute_mahalanobis(z, lm_id)
                if d2 < chi2_threshold:
                    pairs.append((d2, i, lm_id))
                

        #: chọn one-to-one (GNN)
        pairs.sort(key=lambda x: x[0])

        used_feat = set()
        used_lm   = set()
        associations = {}

        for d2, i, lm_id in pairs:
            if i not in used_feat and lm_id not in used_lm:
                associations[i] = lm_id
                used_feat.add(i)
                used_lm.add(lm_id)

        #xây self.z + đánh dấu landmark
        for i, z in enumerate(features):
            if i in associations:
                lm_id = associations[i]
                self.z.append(z)
                self.z_lm_ids.append(lm_id)

                # --- NEW ---
                self.lm_observed[lm_id] = True

                self.landmark_score[lm_id] += 2.0
            else:
                self.new_features.append(z)


    def compute_mahalanobis(self, z, lm_id):
        z = np.asarray(z).reshape(2, 1)

        lm_index = 3 + 2 * lm_id
        mx, my = self.x[lm_index:lm_index+2, 0]

        xr, yr, theta = self.x[0, 0], self.x[1, 0], self.x[2, 0]

        dx = mx - xr
        dy = my - yr
        q = dx*dx + dy*dy

        if q < 1e-6:
            return float("inf")

        sqrt_q = math.sqrt(q)

        z_hat = np.array([
            [sqrt_q],
            [normalize_angle(math.atan2(dy, dx) - theta)]
        ])

        H = np.zeros((2, len(self.x)))

        H[0,0] = -dx / sqrt_q
        H[0,1] = -dy / sqrt_q
        H[1,0] =  dy / q
        H[1,1] = -dx / q
        H[1,2] = -1

        H[0,lm_index]   =  dx / sqrt_q
        H[0,lm_index+1] =  dy / sqrt_q
        H[1,lm_index]   = -dy / q
        H[1,lm_index+1] =  dx / q

        S = H @ self.P @ H.T + self.R

        y = z - z_hat
        y[1,0] = normalize_angle(y[1,0])

        return float(y.T @ np.linalg.inv(S) @ y)



def main(args=None):
    rclpy.init(args=args)

    ekf_slam_node = EKFSLAM()

    try:
        rclpy.spin(ekf_slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  
