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
from scipy.linalg import cholesky
from scipy.linalg import block_diag
from message_filters import Subscriber, ApproximateTimeSynchronizer
from concurrent.futures import ThreadPoolExecutor

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

class UKFSLAM(Node):
    def __init__(self):
        super().__init__("ekf_slam_node")
    
        # State vector [xr, yr, theta, m1x, m1y, m2x, m2y, ...]
        self.x = np.ones((3,1)) * 1e-6
        # Covariance matrix
        self.P = np.eye(3) * 1e-3
        # Noise
        self.Q = np.eye(3) * 1e-3  # motion noise
        self.R = np.diag([0.0025, 0.0036])        # measurement noise
        # lamarks
        self.num_landmarks = 0
        self.max_landmarks =150     # giới hạn số landmark
        self.landmark_score = []     # độ tin cậy

        # measurements in current step
        self.z = []
        self.R_z = []
        self.z_lm_ids = []
        self.new_features = []

        #UKF
        self.alpha, self.kappa, self.beta = 0.01, 0.0, 2.0

        #time parameter
        self.last_odom = None      #[x, y, theta]
        self.last_vel = None        #[v, w]
        self.last_odom_cov = None        #3x3
        self.last_vel_cov = None         #2x2
        self.last_predict_time = None       #last predict

        self.scan_thread = ThreadPoolExecutor(max_workers=4)
        self.extract_thread = ThreadPoolExecutor(max_workers=4)
        #Ros 2 init
        self.ros_init()

    def declare_param(self):
        self.declare_parameter('scan_topic', '/robot1/scan')
        self.declare_parameter('odom_topic', '/odometry/data')

    def load_parameters(self):
        # Topic names
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value

    def ros_init(self):
        # Declare Topic Name Parameters
        self.declare_param()
        # load parameter
        self.load_parameters()
        # Subcriptions
        # self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        # self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
        self.odom_sub = Subscriber(self, Odometry, self.odom_topic)
        self.scan_sub = Subscriber(self, LaserScan, self.scan_topic)
        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.scan_sub],
            queue_size=20,
            slop=0.03  # sai số thời gian cho phép (50ms)
        )

        self.ts.registerCallback(self.sync_cb)
        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ekf_slam/pose", qos2)
        self.map_pub = self.create_publisher(MarkerArray, "/ekf_slam/map", 1)

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

    def sync_cb(self, odom_msg, scan_msg):
       self.odom_cb(odom_msg)
       self.scan_cb(scan_msg)

    def odom_cb(self, msg: Odometry):
        # data pose_odom ---
        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        # v = msg.twist.twist.linear.x
        # w = msg.twist.twist.angular.z

        # odom_covriance 3x3 [x, y, yaw]
        # c = msg.pose.covariance
        # curr_odom_cov = np.array([
        #     [c[0],  c[1],  c[5]],
        #     [c[6],  c[7],  c[11]],
        #     [c[30], c[31], c[35]]
        # ])
        
        # control corvariance 2x2 [v, w]
        # cv = msg.twist.covariance
        # curr_vel_cov = np.array([
        #     [cv[0],  cv[5]],
        #     [cv[30], cv[35]]
        # ])
        
        curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # kiem tra khoi tao
        if self.last_predict_time is None:
            self.last_odom = [curr_x, curr_y, curr_yaw]
            # self.last_odom_cov = curr_odom_cov
            # self.last_vel = [v, w]
            # self.last_vel_cov = curr_vel_cov
            self.last_predict_time = curr_time
            return

        # compute timestamp
        # dt = curr_time - self.last_predict_time
        
        # Delta Pose 
        dx = curr_x - self.last_odom[0]
        dy = curr_y - self.last_odom[1]
        raw_dtheta = curr_yaw - self.last_odom[2]
        dtheta = math.atan2(math.sin(raw_dtheta), math.cos(raw_dtheta))

        # --- robot_motion ---
        # R = np.array([
        #     [math.cos(self.last_odom[2]), -math.sin(self.last_odom[2]), 0],
        #     [math.sin(self.last_odom[2]),  math.cos(self.last_odom[2]), 0],
        #     [0,      0,     1]
        # ])
        # delta_robot = R.T @ np.array([[dx], [dy], [dtheta]])
        dx_robot =  math.cos(self.last_odom[2]) * dx + math.sin(self.last_odom[2]) * dy
        dy_robot = -math.sin(self.last_odom[2]) * dx + math.cos(self.last_odom[2]) * dy

        # if abs(dx_robot) < 0.0005 and abs(dy_robot) < 0.0005 and abs(dtheta) < 0.0005:
        #     dx_robot, dy_robot, dtheta = np.array([[1e-15], [1e-15], [1e-15]])

        # Q_incremental: 
        dist = math.sqrt(dx_robot**2 + dy_robot**2)
        Q_robot = np.diag([
            0.005 * dist + 1e-12,        # Nhiễu x
            0.005 * dist + 1e-12,        # Nhiễu y
            0.005* math.fabs(dtheta**2) + 1e-12  # Nhiễu theta
        ])

        # # Thực hiện phép biến đổi (Propagation)
        # Q_robot = R.T @ curr_odom_cov @ R
 
        self.predict_state((dx_robot, dy_robot, dtheta), Q_robot)
        # self.publish_pose(msg.header.stamp)
        # 
        self.last_odom = [curr_x, curr_y, curr_yaw]
        # self.last_odom_cov = curr_odom_cov
        self.last_predict_time = curr_time
        # self.last_vel = [v, w]
        # self.last_vel_cov = curr_vel_cov


    def scan_cb(self, scan: LaserScan):
            # 1. Trích xuất đặc trưng
            features = self.extract_features_from_scan(scan)
            if len(features) > 0:
        # --- 1. Tham số Unscented Transform (UT) ---
                # w_m, w_c, sigmas = self.generate_sigma_points(self.x, self.P)

                # --- 3. Measurement Model (Dự đoán z cho các landmark quan sát được) ---
                # z_hat, S, Pxz = self.predict_all_measurements(sigmas, w_m, w_c)
                P_zz = np.zeros((2 * self.num_landmarks, 2 * self.num_landmarks))
                z_hat = np.zeros(2 * self.num_landmarks)
                H = np.zeros((2 * self.num_landmarks, self.x.shape[0]))
                for lm_id in range(self.num_landmarks):
                    # Sửa dòng lỗi trong scan_cb:
                    z_ret, P_ret, _, H_ret = self.predict_measurement(lm_id)

                    z_hat[2*lm_id : 2*lm_id+2] = z_ret.flatten()  # Chuyển (2,1) thành (2,)
                    P_zz[2*lm_id : 2*lm_id+2, 2*lm_id : 2*lm_id+2] = P_ret
                    H[2*lm_id : 2*lm_id+2, :] = H_ret

                # 2. Data association (Hàm này sẽ lấp đầy self.z và self.new_features)
                self.association(features, z_hat, P_zz)

                # 3. UKF BATCH UPDATE
                batch_obs = []
                for i in range(len(self.z)):
                    r, b = self.z[i]
                    lm_id = self.z_lm_ids[i]
                    batch_obs.append((r, b, lm_id))

                if batch_obs:
                    self.update(z_hat, P_zz, H, batch_obs, self.R_z)

                # 4. Thêm landmark mới (Sử dụng dữ liệu từ association)
                for z in self.new_features:
                    if self.num_landmarks < self.max_landmarks:
                        self.add_landmark(z)
                    else:
                        # Nếu map đầy, thay thế landmark tệ nhất
                        low_score_id = np.argmin(self.landmark_score)
                        self.remove_landmark(low_score_id)
                        self.add_landmark(z)

                # Đảm bảo góc luôn chuẩn hóa sau khi update
                self.x[2, 0] = normalize_angle(self.x[2, 0])

            # 5. Publish & Log
            self.publish_pose(scan.header.stamp)
            self.publish_map(scan.header.stamp)
            self.get_logger().info(f"EKF-SLAM: num_landmarks={self.num_landmarks}, pose=({self.x[0,0]:.4f}, {self.x[1,0]:.4f}, {self.x[2,0]:.4f})")
            # self.get_logger().info(...)

    
    #=================
    def generate_sigma_points(self, x, P, a):
        n = x.shape[0] 
        # Tham số cho Unscented Transform ---
        lambd = a**2 * (n + self.kappa) - n
        # Tính trọng số (Weights)
        w_m = np.zeros(2 * n + 1)
        w_c = np.zeros(2 * n + 1)
        w_m[0] = lambd / (n + lambd)
        w_c[0] = lambd / (n + lambd) + (1 - self.alpha**2 + self.beta)
        for i in range(1, 2 * n + 1):
            w_m[i] = w_c[i] = 1 / (2 * (n + lambd))

        #-------- Tạo Sigma Points ---------
        # Tính căn bậc hai của ma trận (Matrix Square Root)
        try:
            P = 0.5 * (P + P.T) 
            idx_diag = np.diag_indices_from(P)
            P[idx_diag] = np.maximum(P[idx_diag], 1e-9)
            U = cholesky((n + lambd) * P)
        except np.linalg.LinAlgError:
            self.get_logger().warn("Cholesky decomposition failed, adding jitter to P")
            self.get_logger().debug(f"P before jitter:\n{P}")
            return 

        sigma_points = np.zeros((2 * n + 1, n))
        sigma_points[0] = x.flatten()
        for k in range(n):
            sigma_points[k + 1] = x.flatten() + U[k]
            sigma_points[k + n + 1] = x.flatten() - U[k]

        return w_m, w_c, sigma_points
    
    
    # =========================
    #1. PREDICTION STEP
    # delta_x: dx, dy, dtheta ==> robot motion frame
    # =========================
    def predict_state(self, delta_x, Q):
        x = np.zeros((6, 1))
        x[:3,0] = self.x[:3, 0]          # [xr, yr, theta] + noise
        x[3:, 0] = np.array(delta_x).flatten()
        P = np.zeros((6, 6))
        P[:3, :3] = self.P[:3, :3]       # Robot covariance + noise
        P[3:, 3:] = Q

        n = x.shape[0]

        # generate sigma points
        w_m, w_c, sigma_points = self.generate_sigma_points(x, P, 0.1)
        # -----------Dự báo từng Sigma Point qua Motion Model---------
        sigmas_f = np.copy(sigma_points) 
        pts = sigma_points[:, 2] 
        cos_pts = np.cos(pts)
        sin_pts = np.sin(pts)

        # Cập nhật x, y, theta 
        sigmas_f[:, 0] = sigma_points[:, 0] + cos_pts * sigma_points[:, 3] - sin_pts * sigma_points[:, 4]
        sigmas_f[:, 1] = sigma_points[:, 1] + sin_pts * sigma_points[:, 3] + cos_pts * sigma_points[:, 4]
        new_thetas = pts + sigma_points[:, 5]
        # sigmas_f[:, 2] = np.arctan2(np.sin(new_thetas), np.cos(new_thetas))
        sigmas_f[:, 2] = (new_thetas + np.pi) % (2.0 * np.pi) - np.pi

        # -------- Hợp nhất (Recover Mean and Covariance) --------
        # Tính State mới (Weighted mean)
        x_mean = np.sum(w_m[:, None] * sigma_points, axis=0)
        s = np.sum(w_m * np.sin(sigma_points[:,2]))
        c = np.sum(w_m * np.cos(sigma_points[:,2]))
        x_mean[2] = np.arctan2(s, c)
        y = np.sum(w_m[:, None] * sigmas_f, axis=0).reshape(-1, 1)
        sin_sum = np.sum(w_m * np.sin(sigmas_f[:, 2]))
        cos_sum = np.sum(w_m * np.cos(sigmas_f[:, 2]))
        y[2, 0] = math.atan2(sin_sum, cos_sum)
        
        # Tính Covariance mới (Weighted covariance)
        P_yx = np.zeros((6, 6)) 
        P_yy = np.zeros((6, 6)) 
        P_xx = np.zeros((6, 6))
        for i in range(2*n + 1):
            dx = (sigma_points[i] - x_mean.flatten()).reshape(-1,1)
            dx[2,0] = normalize_angle(dx[2,0])
            dy = (sigmas_f[i] - y.flatten()).reshape(-1,1)
            dy[2,0] = normalize_angle(dy[2,0])

            P_yx += w_c[i] * (dy @ dx.T)
            P_yy += w_c[i] * (dy @ dy.T)
            P_xx += w_c[i] * (dx @ dx.T)

        A = P_yx @ np.linalg.solve(P_xx, np.eye(P_xx.shape[0]))
        Phi_R = A[:3, :3]  

        P_RL_old = self.P[:3, 3:]   # robot-landmark
        P_RL_new = Phi_R @ P_RL_old
        self.P[:3, :3] = P_yy[:3, :3] 
        self.P[:3, 3:] = P_RL_new
        self.P[3:, :3] = P_RL_new.T
        self.P[3:, 3:] = self.P[3:, 3:] + 1e-5*np.eye(self.x.shape[0]-3)

        self.x[:3, 0] = y[:3, 0]  

    
    def predict_measurement(self, landmark_id):
        x = np.zeros((5, 1))
        x[:3,0] = self.x[:3, 0]          # [xr, yr, theta] 
        x[3:, 0] = self.x[3 + 2*landmark_id : 3 + 2*landmark_id + 2, 0]    # [mx, my]
        P = np.zeros((5, 5))
        P[:3, :3] = self.P[:3, :3]       
        P[3:, 3:] = self.P[3 + 2*landmark_id : 3 + 2*landmark_id + 2,
                           3 + 2*landmark_id : 3 + 2*landmark_id + 2] + np.eye(2) * 1e-6  # Landmark covariance
        P[:3, 3:] = self.P[:3, 3 + 2*landmark_id : 3 + 2*landmark_id + 2]   # Robot-Landmark covariance
        P[3:, :3] = self.P[3 + 2*landmark_id : 3 + 2*landmark_id + 2, :3]   # Landmark-Robot covariance

        n = x.shape[0]

        # generate sigma points
        w_m, w_c, sigma_points = self.generate_sigma_points(x, P, 1.0)
        # -----------Dự báo từng Sigma Point qua Motion Model---------
        Z_sigmas = np.zeros((2*n + 1, 2))   
        z_pred = np.zeros((2, 1))

        for i in range(2*n+1):
            xr, yr, theta = sigma_points[i, 0:3]
            mx, my = sigma_points[i, 3:5]
                
            dx, dy = mx - xr, my - yr
            dist = np.sqrt(dx**2 + dy**2)
            bearing = normalize_angle(np.arctan2(dy, dx) - theta)
                
            Z_sigmas[i, 0] = dist
            Z_sigmas[i, 1] = bearing

        # -------- Hợp nhất (Recover Mean and Covariance) --------
        # Tính State mới (Weighted mean)
        x_mean = np.sum(w_m[:, None] * sigma_points, axis=0)
        s = np.sum(w_m * np.sin(sigma_points[:,2]))
        c = np.sum(w_m * np.cos(sigma_points[:,2]))
        x_mean[2] = np.arctan2(s, c)
        z_pred[0,0] = np.sum(w_m * Z_sigmas[:, 0])  
        s_mean = np.sum(w_m * np.sin(Z_sigmas[:, 1]))
        c_mean = np.sum(w_m * np.cos(Z_sigmas[:, 1]))
        z_pred[1,0] = np.arctan2(s_mean, c_mean)

        # Tính Covariance mới (Weighted covariance)
        P_zx = np.zeros((2, 5)) 
        P_zz = np.zeros((2, 2)) 
        P_xx = np.zeros((5, 5))
        for i in range(2*n + 1):
            dx = (sigma_points[i] - x_mean.flatten()).reshape(-1,1)
            dx[2,0] = normalize_angle(dx[2,0])
            dz = (Z_sigmas[i] - z_pred.flatten()).reshape(-1,1)
            dz[1,0] = normalize_angle(dz[1,0])

            P_zx += w_c[i] * (dz @ dx.T)    # (2x1)(1x5) → (2x5)
            P_zz += w_c[i] * (dz @ dz.T)    # 2x2
            P_xx += w_c[i] * (dx @ dx.T)    # 5x5

        A = np.linalg.solve(P_xx.T, P_zx.T).T
        H = np.zeros((2, 3 +2*self.num_landmarks))
        H[:, :3] = A[:, :3]
        H[:, 2*landmark_id : 2*landmark_id + 2] = A[:, 3:]

        return z_pred, P_zz, P_zx, H
    
        # =========================
        # 2. UPDATE STEP
        # =========================
    def update(self, z_pred, P_zz, H, observations, R):
        """
        observations: Danh sách các tuple [(r, b, lm_id), ...] từ kết quả association
        Z_pred_full: Vector (2*M,) dự báo cho tất cả landmark
        S_full: Ma trận (2*M, 2*M) hiệp phương sai dự báo toàn phần
        Pxz_full: Ma trận (n, 2*M) hiệp phương sai chéo
        """
        if not observations:
            return

        m = len(observations)      # Số lượng landmark khớp được
        n = self.x.shape[0]        # 3 + 2*M
        z_dim = 2 * m
        
        # --- 1. Xây dựng Index quan sát (Indices Mapping) ---
        # Chúng ta cần nhặt ra các hàng/cột tương ứng với lm_id từ ma trận Full
        matched_indices = []
        z_actual = np.zeros((z_dim, 1))
        z_hat = np.zeros((z_dim, 1))
        H_f = np.zeros((z_dim, n))
        for i, (r, b, lm_id) in enumerate(observations):
            # Tọa độ đo đạc thực tế
            z_actual[2*i] = r
            z_actual[2*i+1] = b
            
            # Trích xuất dự báo tương ứng từ Z_pred_full
            z_hat[2*i] = z_pred[2*lm_id]
            z_hat[2*i+1] = z_pred[2*lm_id+1]
            
            H_f[2*i : 2*i+2, :] = H[2*lm_id : 2*lm_id+2, :]
            # Lưu lại vị trí các cột trong ma trận S_full và Pxz_full
            matched_indices.extend([2*lm_id, 2*lm_id+1])

        # --- 2. Trích xuất các ma trận con (Slicing) ---
        # S_match kích thước (2m x 2m)
        # S = P_zz[np.ix_(matched_indices, matched_indices)] + block_diag(*self.R_z)
        R_batch = block_diag(*R)
        PHt = self.P @ H_f.T
        S = H_f @ PHt + R_batch

        # --- 4. Tính toán Kalman Gain và Cập nhật ---
        K = np.linalg.solve(S.T, PHt.T).T

        # Innovation y
        y = z_actual - z_hat
        y[1::2, 0] = np.array([normalize_angle(angle) for angle in y[1::2, 0]])
                
        # Cập nhật State x
        self.x = self.x + K @ y
        self.x[2, 0] = normalize_angle(self.x[2, 0])
            
        I = np.eye(n)
        IKH = I - K @ H_f
        self.P = IKH @ self.P @ IKH.T + K @ R_batch @ K.T
    
    # Force symmetry (Chống trôi số học - Nguyên nhân gây nhảy landmark)
        self.P = 0.5 * (self.P + self.P.T)
            

     # =========================
    #3. ADD NEW LANDMARK
    # =========================
    def add_landmark(self, z_feat):
        """
        z_feat: Tuple (z_mean, R_obs) từ cluster_to_feature
        z_mean: [r, b]
        R_obs: Ma trận hiệp phương sai 2x2 của cụm điểm đo được
        """
        z_mean, R_obs = z_feat
        r, b = z_mean
        
        # 1. Trích xuất trạng thái robot hiện tại
        xr, yr, theta = self.x[0:3, 0]

        # 2. Tính vị trí tuyệt đối của Landmark (Toạ độ Cartesian)
        phi = normalize_angle(theta + b)
        mx = xr + r * np.cos(phi)
        my = yr + r * np.sin(phi)

        # 3. Mở rộng Vector trạng thái x
        self.x = np.vstack((self.x, [[mx], [my]]))

       # --- 4. MỞ RỘNG MA TRẬN P VỚI TƯƠNG QUAN ---
        n_old = self.P.shape[0]
        
        # Jacobian của hàm chuyển đổi (Polar -> Cartesian) theo Robot [xr, yr, theta]
        Gr = np.array([
            [1, 0, -r * np.sin(phi)],
            [0, 1,  r * np.cos(phi)]
        ])
        
        # Jacobian của hàm chuyển đổi theo phép đo [r, b]
        Gz = np.array([
            [np.cos(phi), -r * np.sin(phi)],
            [np.sin(phi),  r * np.cos(phi)]
        ])

        # A. Tính tương quan giữa Map hiện tại và Landmark mới
        # P_new_column = P_old * Gr.T
        # Kích thước: (n_old x 3) * (3 x 2) = (n_old x 2)
        P_robot_map = self.P[:, :3] 
        P_cross = P_robot_map @ Gr.T

        # B. Tính hiệp phương sai tự thân của Landmark mới (Uncertainty)
        # P_ll = Gr * P_robot * Gr.T + Gz * R_obs * Gz.T
        P_robot_only = self.P[0:3, 0:3]
        P_ll = Gr @ P_robot_only @ Gr.T + Gz @ R_obs @ Gz.T

        # C. Ghép vào ma trận P mới
        P_new = np.zeros((n_old + 2, n_old + 2))
        P_new[:n_old, :n_old] = self.P           # Map cũ
        P_new[:n_old, n_old:] = P_cross         # Tương quan (Cột phải)
        P_new[n_old:, :n_old] = P_cross.T       # Tương quan (Hàng dưới)
        P_new[n_old:, n_old:] = P_ll            # Landmark mới

        self.P = P_new

        # 5. Cập nhật các biến quản lý
        self.num_landmarks += 1
        self.landmark_score.append(2.0)
        
        if hasattr(self, 'lm_observed'):
            self.lm_observed = np.append(self.lm_observed, True)


    def remove_landmark(self, lm_id):
        """
        Xóa landmark khỏi state, covariance và metadata.
        """
        if lm_id >= self.num_landmarks:
            return

        idx = 3 + 2 * lm_id

        # 1. Xóa khỏi vector trạng thái x (axis=0 vì x là vector cột n x 1)
        self.x = np.delete(self.x, [idx, idx + 1], axis=0)

        # 2. Xóa khỏi ma trận hiệp phương sai P (cả hàng và cột)
        self.P = np.delete(self.P, [idx, idx + 1], axis=0)
        self.P = np.delete(self.P, [idx, idx + 1], axis=1)

        # 3. Cập nhật metadata
        self.landmark_score.pop(lm_id)
        self.num_landmarks -= 1
        
        # Xóa trạng thái quan sát (nếu có dùng mảng numpy)
        if hasattr(self, 'lm_observed'):
            self.lm_observed = np.delete(self.lm_observed, lm_id)


    # =========================
    # 4. FEATURE EXTRACTION FROM LASER SCAN
    # =========================
    def extract_features_from_scan(self, scan):
        ranges = np.array(scan.ranges)
        indices = np.arange(len(ranges))

        valid = np.isfinite(ranges)
        ranges = ranges[valid]
        angles = (scan.angle_min + indices * scan.angle_increment)[valid]
        valid_indices = indices[valid]
        
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)

        point_cloud = np.empty((len(xs), 5))
        point_cloud[:,0] = xs
        point_cloud[:,1] = ys
        point_cloud[:,2] = ranges
        point_cloud[:,3] = angles
        point_cloud[:,4] = valid_indices

        segment_clusters = self.segment_scan(point_cloud, 0.3, 3)

        # 2. Chạy trích xuất đặc trưng cho TỪNG cụm
        # clusters = []
        # for point_cluster in segment_clusters:
        #     features = self.extract_curvature_points(
        #         point_cluster,
        #         k=5, 
        #         curvature_threshold=0.2,
        #         range_min=0.5,
        #         range_max=10.0
        #     )

        #     if len(features) > 0:
        #         clusters.append(features)

        clusters = []
        futures = [
            self.extract_thread.submit(
                self.extract_curvature_points,
                point_cluster,
                5,          # k
                0.185,        # curvature_threshold
                0.5,        # range_min
                10.0        # range_max
            )
            for point_cluster in segment_clusters
        ]

        # lấy kết quả
        for future in futures:
            features = future.result()
            if features is not None and len(features) > 0:
                clusters.append(features)

        if len(clusters) == 0:
            return []

        curv_pts = np.vstack(clusters)

        if len(curv_pts) < 1:
            return []
        
        clusters = self.cluster_features(curv_pts,
                     angle_thresh=0.02)

        measurements = [self.cluster_to_feature(c) for c in clusters]
        return measurements


    def segment_scan(self, point_cloud, threshold=0.2, min_points=3):
        """
        Tối ưu hóa bằng cách tìm tất cả điểm ngắt cùng lúc.
        """
        if len(point_cloud) < min_points:
            return []

        # 1. Tính toán khoảng cách bình phương giữa các điểm liên tiếp (Vectorized)
        diffs = np.diff(point_cloud[:, :2], axis=0)
        dist_sq_array = np.sum(diffs**2, axis=1)
        
        # 2. Tính toán độ nhảy index giữa các điểm liên tiếp
        idx_diff = np.diff(point_cloud[:, 4])
        
        # 3. Tìm các vị trí "ngắt" (vượt ngưỡng khoảng cách HOẶC nhảy index quá xa)
        break_indices = np.where((dist_sq_array > threshold**2) | (idx_diff > 5))[0] + 1
        
        # 4. Chia mảng thành các cụm bằng np.split
        clusters = np.split(point_cloud, break_indices)
        
        # 5. Lọc các cụm không đủ số lượng điểm (List comprehension nhanh hơn loop append)
        clusters = [c for c in clusters if len(c) >= min_points]
        
        # 6. Xử lý khép vòng 360 độ (Wrap-around)
        if len(clusters) > 1:
            first_pt = clusters[0][0, :2]
            last_pt = clusters[-1][-1, :2]
            if np.sum((first_pt - last_pt)**2) < threshold**2:
                # Gộp cụm cuối vào đầu và xóa cụm cuối
                clusters[0] = np.vstack((clusters[-1], clusters[0]))
                clusters.pop()
                
        return clusters
    

    def extract_curvature_points(self, point_cluster, 
                                k=5, 
                                curvature_threshold=0.13, 
                                range_min=0.5, 
                                range_max=10.0):
        """
        point_cluster: np.array shape (N, 5) -> [x, y, r, b, id]
        """
        n = len(point_cluster)
        if n < 2 * k + 1:
            return []

        # # 1. Tính toán độ cong cho vùng trung tâm (từ k đến n-k-1)
        # curv_indices = []
        # for i in range(k, n - k):
        #     if point_cluster[i, 2] < range_min or point_cluster[i, 2] > range_max:
        #         continue
                
        #     # neighbors = point_cluster[i-k : i+k+1, 0:2]
        #     curv = self.compute_curvature(point_cluster[i-k : i+k+1, 0:2])
        #     if curv > curvature_threshold:
        #         curv_indices.append(i)

        # if not curv_indices:
        #     return []

        # # 2. Xây dựng danh sách kết quả (Sử dụng slicing để tránh duplicate)
        # first_curv = curv_indices[0]
        # last_curv = curv_indices[-1]
        
        # final_results = []
        # # Phần bù đầu
        # if first_curv == k:
        #     final_results.extend(point_cluster[0:k])
            
        # # Phần thân (các điểm thực sự vượt ngưỡng)
        # final_results.extend(point_cluster[curv_indices])
            
        # # Phần bù cuối
        # if last_curv == n - k - 1:
        #     final_results.extend(point_cluster[n-k : n])

        # return final_results

        curv = self.fast_pca_curvature(point_cluster[:, :2], k)

        # valid_pts = point_cluster[k:-k]

        mask = (curv > curvature_threshold) & (point_cluster[:,2] > range_min) & (point_cluster[:,2] < range_max)

        return point_cluster[mask]
    
    
    def compute_curvature(self, points):
        # points: Nx2 array (x, y)
        if len(points) < 3:
            return 0.0

        # Tính hiệp phương sai thủ công cho ma trận 2x2 (Nhanh hơn gọi np.cov)
        centered = points - np.mean(points, axis=0)
        # cov = [[var_x, cov_xy], [cov_xy, var_y]]
        cov = (centered.T @ centered) / (len(points) - 1)

        # Tính trị riêng cho ma trận 2x2 bằng công thức nghiệm phương trình bậc 2
        # trace = lambda1 + lambda2, det = lambda1 * lambda2
        trace = cov[0, 0] + cov[1, 1]
        det = cov[0, 0] * cov[1, 1] - cov[0, 1]**2
        
        # Tính lambda_min (trị riêng nhỏ hơn)
        # lambda = (trace - sqrt(trace^2 - 4*det)) / 2
        discriminant = max(0, trace**2 - 4 * det)
        lambda_min = (trace - np.sqrt(discriminant)) / 2.0

        if trace < 1e-6:
            return 0.0

        return lambda_min / trace

    def fast_pca_curvature(self, pts, k=5):
        xs = pts[:,0]
        ys = pts[:,1]

        N = len(xs)
        w = 2*k+1

        # cumulative sums
        cx = np.cumsum(np.insert(xs, 0, 0))
        cy = np.cumsum(np.insert(ys, 0, 0))
        cxx = np.cumsum(np.insert(xs*xs, 0, 0))
        cyy = np.cumsum(np.insert(ys*ys, 0, 0))
        cxy = np.cumsum(np.insert(xs*ys, 0, 0))

        # sliding window sums
        sum_x  = cx[w:]  - cx[:-w]
        sum_y  = cy[w:]  - cy[:-w]
        sum_xx = cxx[w:] - cxx[:-w]
        sum_yy = cyy[w:] - cyy[:-w]
        sum_xy = cxy[w:] - cxy[:-w]

        # mean
        mx = sum_x / w
        my = sum_y / w

        # covariance elements
        var_x = sum_xx/w - mx*mx
        var_y = sum_yy/w - my*my
        cov_xy = sum_xy/w - mx*my

        # trace & determinant
        trace = var_x + var_y
        det = var_x * var_y - cov_xy**2

        # eigenvalue nhỏ (lambda_min)
        discr = np.maximum(0, trace**2 - 4*det)
        lambda_min = (trace - np.sqrt(discr)) / 2.0

        # curvature
        curvature = lambda_min / (trace + 1e-9)

        curvature_full = np.pad(curvature, (k, k), mode='edge')

        return curvature_full


    def cluster_features(self, points, angle_thresh=0.03):
        if len(points) == 0:
            return []

        # 1. Phân cụm thô theo góc
        angle_clusters = self.cluster_by_angle(points, angle_thresh)
        final_clusters = []

        # 2. Lọc và làm sạch từng cụm
        for cl in angle_clusters:
            # cl_array = np.array(cl)
            # Tính khoảng cách trung bình của cụm (cột r là index 2)
            mean_r = np.mean(cl[:, 2])
            
            min_size = self.adaptive_min_cluster_size(mean_r)
            if len(cl) >= min_size:
                # cl = self.filter_converged_points(cl_array, range_thresh=0.5)
                final_clusters.append(cl)

        return final_clusters
    

    def cluster_by_angle(self, points, angle_thresh=0.03):
        if len(points) == 0:
            return []

        pts = np.array(points)
        angles = pts[:, 3]

        # Tính delta góc vectorized
        dtheta = np.abs(np.diff(angles))

        # Nếu có wrap-around (góc nhảy π → -π)
        dtheta = np.minimum(dtheta, 2*np.pi - dtheta)

        # Tìm điểm split
        split_idx = np.where(dtheta > angle_thresh)[0] + 1

        # Split mảng
        clusters = np.split(pts, split_idx)

        return clusters

    def cluster_to_feature(self, cluster):
        """
        cluster: np.array shape (N, 5) -> [x, y, r, b, id]
        Kết hợp Hiệp phương sai tự thân của cụm và Nhiễu mặc định self.R
        """
        cluster = np.array(cluster)
        
        # 1. (Mean)
        mean_r = np.mean(cluster[:, 2])
        
        mean_b = np.arctan2(np.mean(np.sin(cluster[:, 3])), 
                            np.mean(np.cos(cluster[:, 3])))
        
        z_mean = np.array([mean_r, mean_b])

        # 2.(Covariance)
        if len(cluster) > 1:
            data = cluster[:, 2:4]
            mean = np.mean(data, axis=0)
            centered = data - mean
            z_cov_empirical = (centered.T @ centered) / (len(data) - 1)
        else:
            # Nếu chỉ có 1 điểm
            z_cov_empirical = np.zeros((2, 2))

        # 3.(Noise Floor)
        z_cov = z_cov_empirical + self.R

        return z_mean, z_cov

    def adaptive_min_cluster_size(self, r):
        if r < 1.0: return 4
        if r < 3.0: return 3
        if r < 2.0: return 2
        return 1


    # =========================
    # 5. DATA ASSOCIATION
    # =========================
    def association(self, features, Z_pred_full, S_full, chi2_threshold=0.55):
        """
        features: list of (z_obs, R_obs) từ extract_features_from_scan
        Z_pred_full: Vector (2*M,) dự báo [r1, b1, r2, b2...]
        S_full: Ma trận (2*M, 2*M) hiệp phương sai dự báo
        """
        self.z = []
        self.R_z = []
        self.z_lm_ids = []
        self.new_features = []
        self.lm_observed = np.zeros(self.num_landmarks, dtype=bool)

        if self.num_landmarks == 0:
            for z_obs, R_obs in features:
                self.new_features.append((z_obs, R_obs))
            return

        # # 1. Trích xuất các khối đường chéo S cho từng Landmark
        # # S_diag_blocks[lm_id] = ma trận 2x2
        # # S_diag_blocks = [
        # #     S_full[2*j : 2*j+2, 2*j : 2*j+2] for j in range(self.num_landmarks)
        # # ]

        # # 2. Tính toán tất cả ứng viên tiềm năng (Mahalanobis)
        # pairs = []  # (d2, feat_id, lm_id)

        # for i, (z_obs, R_obs) in enumerate(features):
        #     # --- reshape ---
        #     # z_obs = z_obs.reshape(1, 2)   # (1,2)
        #     # Z_pred = Z_pred_full.reshape(-1, 2)   # (M,2)

        #     # --- innovation ---
        #     v = Z_pred_full.reshape(-1, 2) - z_obs   # (M,2)
        #     v[:, 1] = (v[:, 1] + np.pi) % (2*np.pi) - np.pi

        #     # gating thô
        #     mask = (np.abs(v[:,0]) < 2.0) & (np.abs(v[:,1]) < np.pi/6)
        #     valid_ids = np.where(mask)[0]

        #     for lm_id in valid_ids:
        #         S_total = S_full[2*lm_id:2*lm_id+2, 2*lm_id:2*lm_id+2] + R_obs
        #         try:
        #             d2 = v[lm_id].T @ np.linalg.solve(S_total, v[lm_id])
        #             if d2 < chi2_threshold:
        #                 pairs.append((d2, i, lm_id))
        #         except np.linalg.LinAlgError:
        #             continue

        M = self.num_landmarks

        # --- reshape trước (tránh làm lại nhiều lần) ---
        Z_pred = Z_pred_full.reshape(M, 2)

        # --- lấy block S 2x2 cho từng landmark ---
        S_blocks = S_full.reshape(M, 2, M, 2).transpose(0,2,1,3)
        S_blocks = S_blocks[np.arange(M), np.arange(M)]   # (M,2,2)

        pairs = []

        for i, (z_obs, R_obs) in enumerate(features):

            # --- innovation vectorized ---
            v = Z_pred - z_obs   # (M,2)
            v[:,1] = np.arctan2(np.sin(v[:,1]), np.cos(v[:,1]))

            # --- gating thô ---
            mask = (np.abs(v[:,0]) < 2.0) & (np.abs(v[:,1]) < np.pi/6)
            valid_ids = np.where(mask)[0]

            if len(valid_ids) == 0:
                continue

            v_valid = v[valid_ids]                      # (k,2)
            S_valid = S_blocks[valid_ids] + R_obs       # (k,2,2)

            # --- tính Mahalanobis vectorized ---
            # try:
            #     S_inv = np.linalg.inv(S_valid)   # (k,2,2)
            # except np.linalg.LinAlgError:
            #     continue

            d2 = np.einsum('ij,ij->i',
               v_valid,
               np.linalg.solve(S_valid, v_valid[:,:,None]).squeeze(-1))

            # --- filter chi2 ---
            good = d2 < chi2_threshold

            for idx, lm_id in enumerate(valid_ids[good]):
                pairs.append((d2[good][idx], i, lm_id))

        # 3. Chọn cặp khớp One-to-One (GNN)
        pairs.sort(key=lambda x: x[0])

        used_feat = set()
        used_lm   = set()
        associations = {}

        for d2, i, lm_id in pairs:
            if i not in used_feat and lm_id not in used_lm:
                associations[i] = lm_id
                used_feat.add(i)
                used_lm.add(lm_id)

        # 4. Phân loại Feature thành Landmark cũ hoặc Feature mới
        for i, (z_obs, R_obs) in enumerate(features):
            if i in associations:
                lm_id = associations[i]
                self.z.append(z_obs)
                self.R_z.append(R_obs)
                self.z_lm_ids.append(lm_id)
                
                self.lm_observed[lm_id] = True
                self.landmark_score[lm_id] += 2.0
            else:
                # Trả về cả z và R để khởi tạo landmark mới chính xác hơn
                self.new_features.append((z_obs, R_obs))



def main(args=None):
    rclpy.init(args=args)

    ekf_slam_node = UKFSLAM()

    try:
        rclpy.spin(ekf_slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        ekf_slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()  