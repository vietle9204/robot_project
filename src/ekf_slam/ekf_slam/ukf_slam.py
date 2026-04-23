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
        self.x = np.zeros((3,1))
        # Covariance matrix
        self.P = np.eye(3) * 1e-6
        # Noise
        self.Q = np.eye(3) * 1e-2  # motion noise
        self.R = np.diag([0.0036, 0.0049])        # measurement noise
        # lamarks
        self.num_landmarks = 0
        self.max_landmarks = 150    # giới hạn số landmark
        self.landmark_score = []     # độ tin cậy

        # measurements in current step
        self.z = []
        self.R_z = []
        self.z_lm_ids = []
        self.new_features = []

        #UKF
        self.alpha, self.kappa, self.beta = 0.01, 0.0, 2.0

        self.w_m, self.w_c, self.sigma = None, None, None

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
        self.odom_sub = Subscriber(self, Odometry, self.odom_topic, qos_profile=qos2)
        self.scan_sub = Subscriber(self, LaserScan, self.scan_topic, qos_profile=qos)
        self.ts = ApproximateTimeSynchronizer(
            [self.odom_sub, self.scan_sub],
            queue_size=30,
            slop=0.018  # sai số thời gian cho phép (30ms)
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
        dx_robot =  math.cos(self.last_odom[2]) * dx + math.sin(self.last_odom[2]) * dy
        dy_robot = -math.sin(self.last_odom[2]) * dx + math.cos(self.last_odom[2]) * dy
        # dy_robot = 0.0

        if abs(dx_robot) < 0.001 and abs(dy_robot) < 0.001 and abs(dtheta) < 0.001:
            dx_robot, dy_robot, dtheta = 1e-15, 1e-15, 1e-15
        
        # Q_incremental: 
        dist = math.sqrt(dx_robot**2 + dy_robot**2)
        Q_robot = np.diag([
            0.0036 * dist + 1e-15,        # Nhiễu x
            0.0036 * dist + 1e-15,        # Nhiễu y
            0.0016 * math.fabs(dtheta**2) + 1e-15   # Nhiễu theta
        ])
 
        self.predict((dx_robot, dy_robot, dtheta), Q_robot)
        # self.publish_pose(msg.header.stamp)
        # 
        self.last_odom = [curr_x, curr_y, curr_yaw]
        # self.last_odom_cov = curr_odom_cov
        self.last_predict_time = curr_time
        # self.last_vel = [v, w]
        # self.last_vel_cov = curr_vel_cov


    def scan_cb(self, scan: LaserScan):
        if self.sigma is None:
            return
        
        future_features = self.scan_thread.submit(
            self.extract_features_from_scan,
            scan
        )

        future_predict = self.scan_thread.submit(
            self.predict_all_measurements,
            self.sigma,
            self.w_m,
            self.w_c
        )

        features = future_features.result()
        pred_res = future_predict.result()
        if pred_res is None:
            return
        z_hat, S, Pxz = pred_res

        # 2. Data association (Hàm này sẽ lấp đầy self.z và self.new_features)
        self.association(features, z_hat, S)

        # 3. UKF BATCH UPDATE
        batch_obs = [(r, b, lm_id) for (r, b), lm_id in zip(self.z, self.z_lm_ids)]

        if batch_obs:
            self.update(batch_obs, z_hat, S, Pxz) 

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

        self.w_m, self.w_c, self.sigmas = None, None, None
    
    #=================
    def generate_sigma_points(self, x, P):
        n = x.shape[0] 
        # Tham số cho Unscented Transform ---
        lambd = self.alpha**2 * (n + self.kappa) - n
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
            U = cholesky(P)
        except np.linalg.LinAlgError:
            return 

        x_flat = x.flatten()
        scale = np.sqrt(n + lambd)

        sigma_points = np.zeros((2*n + 1, n))
        sigma_points[0] = x_flat

        sigma_points[1:n+1]     = x_flat + scale * U.T
        sigma_points[n+1:2*n+1] = x_flat - scale * U.T

        return w_m, w_c, sigma_points
    
    def predict_all_measurements(self, sigmas, w_m, w_c):
        """
        Dự báo đo lường cho TOÀN BỘ Landmark trong Map từ tập Sigma Points.
        n: số lượng trạng thái (3 + 2*M)
        m: số lượng Landmark hiện có
        """
        num_sigmas = sigmas.shape[0]        #S
        num_lms = self.num_landmarks        #M
        z_dim_full = 2 * num_lms
        
        # 1. Khởi tạo ma trận chứa các Sigma Points trong không gian đo lường
        # Shape: (2*n + 1, 2*m)

        xr = sigmas[:, 0]      # (S,)
        yr = sigmas[:, 1]      # (S,)
        theta = sigmas[:, 2]   # (S,)

        lm = sigmas[:, 3:].reshape(num_sigmas, num_lms, 2)   #(S, 2*M) --> (S, M, 2) 
        mx = lm[:, :, 0]                       # (S, M)
        my = lm[:, :, 1]                       # (S, M)
        
        dx = mx - xr[:, None]                  # (S, M)
        dy = my - yr[:, None]  

        dist = np.sqrt(dx**2 + dy**2)          # (S, M)
        bearing = np.arctan2(dy, dx) - theta[:, None]

        Z_sigmas_full = np.zeros((num_sigmas, z_dim_full))
        Z_sigmas_full[:, 0::2] = dist
        Z_sigmas_full[:, 1::2] = bearing

        # 2. Tính Kỳ vọng dự báo (Z_pred_full)
        dist_mean = np.sum(w_m[:, None] * dist, axis=0)
        # --- mean góc ---
        sin_mean = np.sum(w_m[:, None] * np.sin(bearing), axis=0)
        cos_mean = np.sum(w_m[:, None] * np.cos(bearing), axis=0)
        bearing_mean = np.arctan2(sin_mean, cos_mean)

        # --- gộp lại ---
        Z_pred_full = np.empty(z_dim_full)
        Z_pred_full[0::2] = dist_mean
        Z_pred_full[1::2] = bearing_mean

        # 3. Tính Hiệp phương sai S_full và Pxz_full 
        # Tiền tính toán dx (state error) cho Pxz
        X_diff = sigmas - self.x.reshape(1,-1)
        X_diff[:, 2] = (X_diff[:, 2] + np.pi) % (2*np.pi) - np.pi  #normalization

        dZ = Z_sigmas_full - Z_pred_full   # (S, 2M)
        dZ[:, 1::2] = (dZ[:, 1::2] + np.pi) % (2*np.pi) - np.pi   #normalize

        Wc = w_c.reshape(-1, 1)
        S_full = (Wc * dZ).T @ dZ
        Pxz_full = (Wc * X_diff).T @ dZ

        return Z_pred_full, S_full, Pxz_full
    
    # =========================
    #1. PREDICTION STEP
    # delta_x: dx, dy, dtheta ==> robot motion frame
    # =========================
    def predict(self, delta_x, Q):
        dx, dy, dtheta = delta_x
        n = self.x.shape[0]              # 3+2n
        # Thêm nhiễu Q vào P trước khi tạo sigma points 
        # Lấy góc hiện tại của robot trong Map
        theta = self.x[2, 0]
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        # Ma trận quay R (3x3 cho x, y, theta)
        R = np.array([
            [cos_t, -sin_t, 0],
            [sin_t,  cos_t, 0],
            [0,      0,     1]
        ])
        # Xoay Q từ Robot Frame sang Global Frame
        Q_global = R @ Q @ R.T
        # Sau đó mới gán vào Q_model lớn
        Q_model = np.zeros((n, n))
        Q_model[:3, :3] = Q_global

        # generate sigma points
        w_m, w_c, sigma_points = self.generate_sigma_points(self.x, self.P)
        # -----------Dự báo từng Sigma Point qua Motion Model---------
        sigmas_f = np.copy(sigma_points) # Copy tọa độ landmark
        pts = sigma_points[:, 2] # Lấy cột theta của tất cả sigma points
        cos_pts = np.cos(pts)
        sin_pts = np.sin(pts)

        # Cập nhật x, y, theta 
        sigmas_f[:, 0] = sigma_points[:, 0] + cos_pts * dx - sin_pts * dy
        sigmas_f[:, 1] = sigma_points[:, 1] + sin_pts * dx + cos_pts * dy
        new_thetas = pts + dtheta
        sigmas_f[:, 2] = np.arctan2(np.sin(new_thetas), np.cos(new_thetas))

        # -------- Hợp nhất (Recover Mean and Covariance) --------
        # Tính State mới (Weighted mean)
        self.x = np.sum(w_m[:, None] * sigmas_f, axis=0).reshape(-1, 1)
        sin_sum = np.sum(w_m * np.sin(sigmas_f[:, 2]))
        cos_sum = np.sum(w_m * np.cos(sigmas_f[:, 2]))
        self.x[2, 0] = math.atan2(sin_sum, cos_sum)
    
        # Tính Covariance mới (Weighted covariance)
        # P_new = np.zeros_like(self.P)
        diff = sigmas_f - self.x.T 
        diff[:, 2] = (diff[:, 2] + np.pi) % (2 * np.pi) - np.pi
        self.P = (diff.T * w_c) @ diff + Q_model

        self.w_m, self.w_c, self.sigma = w_m, w_c, sigmas_f

        # =========================
        # 2. UPDATE STEP
        # =========================
    def update(self, observations, Z_pred_full, S_full, Pxz_full):
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
        
        obs = np.array(observations)     # (m,3)
        r = obs[:, 0]
        b = obs[:, 1]
        lm_id = obs[:, 2].astype(int)

        # --- z_actual ---
        z_actual = np.empty((2*m, 1))
        z_actual[0::2, 0] = r
        z_actual[1::2, 0] = b

        # --- z_hat ---
        z_hat = np.empty((2*m, 1))
        z_hat[0::2, 0] = Z_pred_full[2*lm_id]
        z_hat[1::2, 0] = Z_pred_full[2*lm_id + 1]

        # --- matched_indices ---
        matched_indices = np.empty(2*m, dtype=int)
        matched_indices[0::2] = 2*lm_id
        matched_indices[1::2] = 2*lm_id + 1

        # --- 2. Trích xuất các ma trận con (Slicing) ---
        # S_match kích thước (2m x 2m)
        S = S_full[np.ix_(matched_indices, matched_indices)]
        # Pxz kích thước (n x 2m)
        Pxz = Pxz_full[:, matched_indices]

        # --- 3. Cộng nhiễu đo lường R ---
        R = np.kron(np.eye(m), self.R_z[0])
        S = S + R

        # --- 4. Tính toán Kalman Gain và Cập nhật ---
        try:
            # Đảm bảo tính đối xứng để Cholesky không lỗi
            S = 0.5 * (S + S.T) + 1e-9 * np.eye(z_dim)
            L_s = np.linalg.cholesky(S)
            
            # K = Pxz * S^-1 (Giải hệ phương trình L*L.T * K.T = Pxz.T)
            K = np.linalg.solve(L_s.T, np.linalg.solve(L_s, Pxz.T)).T
            
            # Innovation y
            y = z_actual - z_hat
            y[1::2, 0] = (y[1::2, 0] + np.pi) % (2*np.pi) - np.pi
                
            # Cập nhật State x
            self.x = self.x + K @ y
            self.x[2, 0] = normalize_angle(self.x[2, 0])
            
            # Cập nhật Covariance 
            self.P = self.P - K @ S @ K.T
            
        except np.linalg.LinAlgError:
            print("UKF Update: Cholesky failed, skipping update.")
            return

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
        P_ll = Gr @ P_robot_only @ Gr.T + Gz @ R_obs @ Gz.T + 9.0*1e-4*np.eye(2)

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

        segment_clusters = self.segment_scan(point_cloud, 0.2, 9)

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
                     angle_thresh=0.03)

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
        if r < 5.0: return 2
        return 1


    # =========================
    # 5. DATA ASSOCIATION
    # =========================
    def association(self, features, Z_pred_full, S_full, chi2_threshold=5.99):
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










# #!/usr/bin/env python3
# from pyexpat import features
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu, MagneticField, PointCloud2, LaserScan
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
# from visualization_msgs.msg import Marker, MarkerArray
# from scipy.linalg import cholesky
# from scipy.linalg import block_diag
# from message_filters import Subscriber, ApproximateTimeSynchronizer

# qos = QoSProfile(
#     reliability=ReliabilityPolicy.BEST_EFFORT,
#     durability=DurabilityPolicy.VOLATILE,
#     depth=10
# )
# qos2 = QoSProfile(
#     reliability=ReliabilityPolicy.RELIABLE,
#     durability=DurabilityPolicy.VOLATILE,
#     depth=10
# )

# def normalize_angle(a):
#     return (a + math.pi) % (2.0 * math.pi) - math.pi

# def yaw_from_quaternion(q):
#     return math.atan2(
#         2.0 * (q.w * q.z + q.x * q.y),
#         1.0 - 2.0 * (q.y*q.y + q.z*q.z)
#     )

# def quaternion_from_euler(roll, pitch, yaw):
#     """
#     Convert Euler angles (roll, pitch, yaw) to quaternion.
#     All angles in radians.
#     Return: (qx, qy, qz, qw)
#     """
#     cy = math.cos(yaw * 0.5)
#     sy = math.sin(yaw * 0.5)
#     cp = math.cos(pitch * 0.5)
#     sp = math.sin(pitch * 0.5)
#     cr = math.cos(roll * 0.5)
#     sr = math.sin(roll * 0.5)

#     qw = cr * cp * cy + sr * sp * sy
#     qx = sr * cp * cy - cr * sp * sy
#     qy = cr * sp * cy + sr * cp * sy
#     qz = cr * cp * sy - sr * sp * cy

#     return qx, qy, qz, qw

# class UKFSLAM(Node):
#     def __init__(self):
#         super().__init__("ekf_slam_node")
    
#         # State vector [xr, yr, theta, m1x, m1y, m2x, m2y, ...]
#         self.x = np.ones((3,1)) * 1e-6
#         # Covariance matrix
#         self.P = np.eye(3) * 1e-3
#         # Noise
#         self.Q = np.eye(3) * 1e-2  # motion noise
#         self.R = np.diag([0.05, 0.055])        # measurement noise
#         # lamarks
#         self.num_landmarks = 0
#         self.max_landmarks = 150    # giới hạn số landmark
#         self.landmark_score = []     # độ tin cậy

#         # measurements in current step
#         self.z = []
#         self.R_z = []
#         self.z_lm_ids = []
#         self.new_features = []

#         #UKF
#         self.alpha, self.kappa, self.beta = 0.005, 0.0, 2.0

#         self.w_m, self.w_c, self.sigma = None, None, None

#         #time parameter
#         self.last_odom = None      #[x, y, theta]
#         self.last_vel = None        #[v, w]
#         self.last_odom_cov = None        #3x3
#         self.last_vel_cov = None         #2x2
#         self.last_predict_time = None       #last predict

#         #Ros 2 init
#         self.ros_init()

#     def declare_param(self):
#         self.declare_parameter('scan_topic', '/robot1/scan')
#         self.declare_parameter('odom_topic', '/odometry/data')

#     def load_parameters(self):
#         # Topic names
#         self.scan_topic = self.get_parameter('scan_topic').value
#         self.odom_topic = self.get_parameter('odom_topic').value

#     def ros_init(self):
#         # Declare Topic Name Parameters
#         self.declare_param()
#         # load parameter
#         self.load_parameters()
#         # Subcriptions
#         # self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
#         # self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, qos)
#         self.odom_sub = Subscriber(self, Odometry, self.odom_topic, qos_profile=qos2)
#         self.scan_sub = Subscriber(self, LaserScan, self.scan_topic, qos_profile=qos)
#         self.ts = ApproximateTimeSynchronizer(
#             [self.odom_sub, self.scan_sub],
#             queue_size=30,
#             slop=0.03  # sai số thời gian cho phép (30ms)
#         )

#         self.ts.registerCallback(self.sync_cb)
#         # Publishers
#         self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/ekf_slam/pose", qos2)
#         self.map_pub = self.create_publisher(MarkerArray, "/ekf_slam/map", 1)

#     def publish_pose(self, stamp):
#         msg = PoseWithCovarianceStamped()
#         msg.header.stamp = stamp
#         msg.header.frame_id = "map"   # EKF-SLAM 

#         # --- Pose ---
#         msg.pose.pose.position.x = float(self.x[0, 0])
#         msg.pose.pose.position.y = float(self.x[1, 0])
#         msg.pose.pose.position.z = 0.0

#         q = quaternion_from_euler(0.0, 0.0, self.x[2, 0])
#         msg.pose.pose.orientation.x = q[0]
#         msg.pose.pose.orientation.y = q[1]
#         msg.pose.pose.orientation.z = q[2]
#         msg.pose.pose.orientation.w = q[3]

#         # --- Covariance (6x6) ---
#         cov = np.zeros((6, 6))
#         cov[0, 0] = self.P[0, 0]   # x
#         cov[1, 1] = self.P[1, 1]   # y
#         cov[5, 5] = self.P[2, 2]   # yaw

#         msg.pose.covariance = cov.flatten().tolist()

#         self.pose_pub.publish(msg)    

#     def publish_map(self, stamp):
#         if self.num_landmarks == 0:
#             return

#         marker_array = MarkerArray()

#         for lm_id in range(self.num_landmarks):
#             if lm_id >= len(self.lm_observed):
#                 continue

#             idx = 3 + 2 * lm_id
#             mx = float(self.x[idx, 0])
#             my = float(self.x[idx + 1, 0])

#             m = Marker()
#             m.header.frame_id = "map"
#             m.header.stamp = stamp
#             m.ns = "landmarks"
#             m.id = lm_id
#             m.type = Marker.SPHERE
#             m.action = Marker.ADD

#             m.pose.position.x = mx 

#             m.pose.position.y = my
#             m.pose.position.z = 0.0
#             m.pose.orientation.w = 1.0

#             m.scale.x = m.scale.y = m.scale.z = 0.15

#             m.color.a = 1.0
#             if self.lm_observed[lm_id]:
#                 m.color.g = 1.0
#             else:
#                 m.color.r = 1.0

#             marker_array.markers.append(m)

#         self.map_pub.publish(marker_array)

#     def sync_cb(self, odom_msg, scan_msg):
#        self.odom_cb(odom_msg)
#        self.scan_cb(scan_msg)

#     def odom_cb(self, msg: Odometry):
#         # data pose_odom ---
#         curr_x = msg.pose.pose.position.x
#         curr_y = msg.pose.pose.position.y
#         curr_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
#         # v = msg.twist.twist.linear.x
#         # w = msg.twist.twist.angular.z

#         # odom_covriance 3x3 [x, y, yaw]

#         # c = msg.pose.covariance
#         # curr_odom_cov = np.array([
#         #     [c[0],  c[1],  c[5]],
#         #     [c[6],  c[7],  c[11]],
#         #     [c[30], c[31], c[35]]
#         # ])
        
#         # control corvariance 2x2 [v, w]
#         # cv = msg.twist.covariance
#         # curr_vel_cov = np.array([
#         #     [cv[0],  cv[5]],
#         #     [cv[30], cv[35]]
#         # ])
        
#         curr_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

#         # kiem tra khoi tao
#         if self.last_predict_time is None:
#             self.last_odom = [curr_x, curr_y, curr_yaw]
#             # self.last_odom_cov = curr_odom_cov
#             # self.last_vel = [v, w]
#             # self.last_vel_cov = curr_vel_cov
#             self.last_predict_time = curr_time
#             return

#         # compute timestamp
#         # dt = curr_time - self.last_predict_time
        
#         # Delta Pose 
#         dx = curr_x - self.last_odom[0]
#         dy = curr_y - self.last_odom[1]
#         raw_dtheta = curr_yaw - self.last_odom[2]
#         dtheta = math.atan2(math.sin(raw_dtheta), math.cos(raw_dtheta))

#         # --- robot_motion ---
#         dx_robot =  math.cos(self.last_odom[2]) * dx + math.sin(self.last_odom[2]) * dy
#         dy_robot = -math.sin(self.last_odom[2]) * dx + math.cos(self.last_odom[2]) * dy

#         if abs(dx_robot) < 0.002 and abs(dy_robot) < 0.001 and abs(dtheta) < 0.001:
#             dx_robot, dy_robot, dtheta = 1e-15, 1e-15, 1e-15
        
#         # Q_incremental: 
#         dist = math.sqrt(dx_robot**2 + dy_robot**2)
#         Q_robot = np.diag([
#             0.015 * dist + 1e-12,        # Nhiễu x
#             0.015 * dist + 1e-12,        # Nhiễu y
#             0.015 * math.fabs(dtheta**2) + 1e-12   # Nhiễu theta
#         ])
 
#         self.predict((dx_robot, dy_robot, dtheta), Q_robot)
#         # self.publish_pose(msg.header.stamp)
#         # 
#         self.last_odom = [curr_x, curr_y, curr_yaw]
#         # self.last_odom_cov = curr_odom_cov
#         self.last_predict_time = curr_time
#         # self.last_vel = [v, w]
#         # self.last_vel_cov = curr_vel_cov


#     def scan_cb(self, scan: LaserScan):
#         # 1. Trích xuất đặc trưng
#         features = self.extract_features_from_scan(scan)
#         if len(features) > 0:
#         # --- 1. Tham số Unscented Transform (UT) ---
#             if self.sigma is not None:
#                 w_m, w_c, sigmas = self.w_m, self.w_c, self.sigma
#             else:
#                 # return
#                 w_m, w_c, sigmas = self.generate_sigma_points(self.x, self.P)

#                 # --- 3. Measurement Model (Dự đoán z cho các landmark quan sát được) ---
#             z_hat, S, Pxz = self.predict_all_measurements(sigmas, w_m, w_c)
#             # 2. Data association (Hàm này sẽ lấp đầy self.z và self.new_features)
#             self.association(features, z_hat, S)

#             # 3. UKF BATCH UPDATE
#             batch_obs = [(r, b, lm_id) for (r, b), lm_id in zip(self.z, self.z_lm_ids)]

#             if batch_obs:
#                 self.update(batch_obs, z_hat, S, Pxz) 

#             # 4. Thêm landmark mới (Sử dụng dữ liệu từ association)
#             for z in self.new_features:
#                 if self.num_landmarks < self.max_landmarks:
#                     self.add_landmark(z)
#                 else:
#                 # Nếu map đầy, thay thế landmark tệ nhất
#                     low_score_id = np.argmin(self.landmark_score)
#                     self.remove_landmark(low_score_id)
#                     self.add_landmark(z)

#             # Đảm bảo góc luôn chuẩn hóa sau khi update
#             self.x[2, 0] = normalize_angle(self.x[2, 0])

#         # 5. Publish & Log
#         self.publish_pose(scan.header.stamp)
#         self.publish_map(scan.header.stamp)
#         self.get_logger().info(f"EKF-SLAM: num_landmarks={self.num_landmarks}, pose=({self.x[0,0]:.4f}, {self.x[1,0]:.4f}, {self.x[2,0]:.4f})")
#             # self.get_logger().info(...)

#         w_m, w_c, sigmas = None, None, None
    
#     #=================
#     def generate_sigma_points(self, x, P):
#         n = x.shape[0] 
#         # Tham số cho Unscented Transform ---
#         lambd = self.alpha**2 * (n + self.kappa) - n
#         # Tính trọng số (Weights)
#         w_m = np.zeros(2 * n + 1)
#         w_c = np.zeros(2 * n + 1)
#         w_m[0] = lambd / (n + lambd)
#         w_c[0] = lambd / (n + lambd) + (1 - self.alpha**2 + self.beta)
#         for i in range(1, 2 * n + 1):
#             w_m[i] = w_c[i] = 1 / (2 * (n + lambd))

#         #-------- Tạo Sigma Points ---------
#         # Tính căn bậc hai của ma trận (Matrix Square Root)
#         try:
#             P = 0.5 * (P + P.T) 
#             idx_diag = np.diag_indices_from(P)
#             P[idx_diag] = np.maximum(P[idx_diag], 1e-9)
#             U = cholesky(P)
#         except np.linalg.LinAlgError:
#             return 

#         x_flat = x.flatten()
#         scale = np.sqrt(n + lambd)

#         sigma_points = np.zeros((2*n + 1, n))
#         sigma_points[0] = x_flat

#         sigma_points[1:n+1]     = x_flat + scale * U.T
#         sigma_points[n+1:2*n+1] = x_flat - scale * U.T

#         return w_m, w_c, sigma_points
    
#     def predict_all_measurements(self, sigmas, w_m, w_c):
#         """
#         Dự báo đo lường cho TOÀN BỘ Landmark trong Map từ tập Sigma Points.
#         n: số lượng trạng thái (3 + 2*M)
#         m: số lượng Landmark hiện có
#         """
#         num_sigmas = sigmas.shape[0]        #S
#         num_lms = self.num_landmarks        #M
#         z_dim_full = 2 * num_lms
        
#         # 1. Khởi tạo ma trận chứa các Sigma Points trong không gian đo lường
#         # Shape: (2*n + 1, 2*m)

#         xr = sigmas[:, 0]      # (S,)
#         yr = sigmas[:, 1]      # (S,)
#         theta = sigmas[:, 2]   # (S,)

#         lm = sigmas[:, 3:].reshape(num_sigmas, num_lms, 2)   #(S, 2*M) --> (S, M, 2) 
#         mx = lm[:, :, 0]                       # (S, M)
#         my = lm[:, :, 1]                       # (S, M)
        
#         dx = mx - xr[:, None]                  # (S, M)
#         dy = my - yr[:, None]  

#         dist = np.sqrt(dx**2 + dy**2)          # (S, M)
#         bearing = np.arctan2(dy, dx) - theta[:, None]

#         Z_sigmas_full = np.zeros((num_sigmas, z_dim_full))
#         Z_sigmas_full[:, 0::2] = dist
#         Z_sigmas_full[:, 1::2] = bearing

#         # 2. Tính Kỳ vọng dự báo (Z_pred_full)
#         dist_mean = np.sum(w_m[:, None] * dist, axis=0)
#         # --- mean góc ---
#         sin_mean = np.sum(w_m[:, None] * np.sin(bearing), axis=0)
#         cos_mean = np.sum(w_m[:, None] * np.cos(bearing), axis=0)
#         bearing_mean = np.arctan2(sin_mean, cos_mean)

#         # --- gộp lại ---
#         Z_pred_full = np.empty(z_dim_full)
#         Z_pred_full[0::2] = dist_mean
#         Z_pred_full[1::2] = bearing_mean

#         # 3. Tính Hiệp phương sai S_full và Pxz_full 
#         # Tiền tính toán dx (state error) cho Pxz
#         X_diff = sigmas - self.x.reshape(1,-1)
#         X_diff[:, 2] = (X_diff[:, 2] + np.pi) % (2*np.pi) - np.pi  #normalization

#         dZ = Z_sigmas_full - Z_pred_full   # (S, 2M)
#         dZ[:, 1::2] = (dZ[:, 1::2] + np.pi) % (2*np.pi) - np.pi   #normalize

#         Wc = w_c.reshape(-1, 1)
#         S_full = (Wc * dZ).T @ dZ
#         Pxz_full = (Wc * X_diff).T @ dZ

#         return Z_pred_full, S_full, Pxz_full
    
#     # =========================
#     #1. PREDICTION STEP
#     # delta_x: dx, dy, dtheta ==> robot motion frame
#     # =========================
#     def predict(self, delta_x, Q):
#         dx, dy, dtheta = delta_x
#         n = self.x.shape[0]              # 3+2n
        
#         # generate sigma points
#         w_m, w_c, sigma_points = self.generate_sigma_points(self.x, self.P)
#         # -----------Dự báo từng Sigma Point qua Motion Model---------
#         sigmas_f = np.copy(sigma_points) # Copy tọa độ landmark
#         pts = sigma_points[:, 2] # Lấy cột theta của tất cả sigma points
#         cos_pts = np.cos(pts)
#         sin_pts = np.sin(pts)

#         # Cập nhật x, y, theta 
#         sigmas_f[:, 0] = sigma_points[:, 0] + cos_pts * dx - sin_pts * dy
#         sigmas_f[:, 1] = sigma_points[:, 1] + sin_pts * dx + cos_pts * dy
#         new_thetas = pts + dtheta
#         sigmas_f[:, 2] = np.arctan2(np.sin(new_thetas), np.cos(new_thetas))

#         # -------- Hợp nhất (Recover Mean and Covariance) --------
#         # Tính State mới (Weighted mean)
#         # Tính Mean cho x, y và các landmarks bình thường
#         self.x = np.sum(w_m[:, None] * sigmas_f, axis=0).reshape(-1, 1)

#         # Tính Mean riêng cho Theta bằng hàm lượng giác để tránh sai số hướng
#         sin_sum = np.sum(w_m * np.sin(sigmas_f[:, 2]))
#         cos_sum = np.sum(w_m * np.cos(sigmas_f[:, 2]))
#         self.x[2, 0] = math.atan2(sin_sum, cos_sum)

#         # Tính Covariance mới (Weighted covariance)
#         P_new = np.zeros_like(self.P)
#         for i in range(2 * n + 1):
#             diff = (sigmas_f[i] - self.x.flatten()).reshape(-1, 1)
#             diff[2, 0] = normalize_angle(diff[2, 0])
#             P_new += w_c[i] * (diff @ diff.T)

#         # Thêm nhiễu Q vào P trước khi tạo sigma points 
#         # Lấy góc hiện tại của robot trong Map
#         theta = self.x[2, 0]
#         cos_t = math.cos(theta)
#         sin_t = math.sin(theta)
#         # Ma trận quay R (3x3 cho x, y, theta)
#         R = np.array([
#             [cos_t, -sin_t, 0],
#             [sin_t,  cos_t, 0],
#             [0,      0,     1]
#         ])
#         # Xoay Q từ Robot Frame sang Global Frame
#         Q_global = R @ Q @ R.T

#         # Sau đó mới gán vào Q_model lớn
#         Q_model = np.zeros((n, n))
#         Q_model[:3, :3] = Q_global
        
#         self.P = P_new + Q_model # Cộng nhiễu hệ thống

#         self.w_m, self.w_c, self.sigma = w_m, w_c, sigmas_f

#         # =========================
#         # 2. UPDATE STEP
#         # =========================
#     def update(self, observations, Z_pred_full, S_full, Pxz_full):
#         """
#         observations: Danh sách các tuple [(r, b, lm_id), ...] từ kết quả association
#         Z_pred_full: Vector (2*M,) dự báo cho tất cả landmark
#         S_full: Ma trận (2*M, 2*M) hiệp phương sai dự báo toàn phần
#         Pxz_full: Ma trận (n, 2*M) hiệp phương sai chéo
#         """
#         if not observations:
#             return

#         m = len(observations)      # Số lượng landmark khớp được
#         n = self.x.shape[0]        # 3 + 2*M
#         z_dim = 2 * m
        
#         # --- 1. Xây dựng Index quan sát (Indices Mapping) ---
#         # Chúng ta cần nhặt ra các hàng/cột tương ứng với lm_id từ ma trận Full
#         matched_indices = []
#         z_actual = np.zeros((z_dim, 1))
#         z_hat = np.zeros((z_dim, 1))
        
#         obs = np.array(observations)     # (m,3)
#         r = obs[:, 0]
#         b = obs[:, 1]
#         lm_id = obs[:, 2].astype(int)

#         # --- z_actual ---
#         z_actual = np.empty((2*m, 1))
#         z_actual[0::2, 0] = r
#         z_actual[1::2, 0] = b

#         # --- z_hat ---
#         z_hat = np.empty((2*m, 1))
#         z_hat[0::2, 0] = Z_pred_full[2*lm_id]
#         z_hat[1::2, 0] = Z_pred_full[2*lm_id + 1]

#         # --- matched_indices ---
#         matched_indices = np.empty(2*m, dtype=int)
#         matched_indices[0::2] = 2*lm_id
#         matched_indices[1::2] = 2*lm_id + 1

#         # --- 2. Trích xuất các ma trận con (Slicing) ---
#         # S_match kích thước (2m x 2m)
#         S = S_full[np.ix_(matched_indices, matched_indices)]
#         # Pxz kích thước (n x 2m)
#         Pxz = Pxz_full[:, matched_indices]

#         # --- 3. Cộng nhiễu đo lường R ---
#         R = np.kron(np.eye(m), self.R_z[0])
#         S = S + R

#         # --- 4. Tính toán Kalman Gain và Cập nhật ---
#         try:
#             # Đảm bảo tính đối xứng để Cholesky không lỗi
#             S = 0.5 * (S + S.T) + 1e-9 * np.eye(z_dim)
#             L_s = np.linalg.cholesky(S)
            
#             # K = Pxz * S^-1 (Giải hệ phương trình L*L.T * K.T = Pxz.T)
#             K = np.linalg.solve(L_s.T, np.linalg.solve(L_s, Pxz.T)).T
            
#             # Innovation y
#             y = z_actual - z_hat
#             y[1::2, 0] = (y[1::2, 0] + np.pi) % (2*np.pi) - np.pi
                
#             # Cập nhật State x
#             self.x = self.x + K @ y
#             self.x[2, 0] = normalize_angle(self.x[2, 0])
            
#             # Cập nhật Covariance 
#             self.P = self.P - K @ S @ K.T
            
#         except np.linalg.LinAlgError:
#             print("UKF Update: Cholesky failed, skipping update.")
#             return

#     # =========================
#     #3. ADD NEW LANDMARK
#     # =========================
#     def add_landmark(self, z_feat):
#         """
#         z_feat: Tuple (z_mean, R_obs) từ cluster_to_feature
#         z_mean: [r, b]
#         R_obs: Ma trận hiệp phương sai 2x2 của cụm điểm đo được
#         """
#         z_mean, R_obs = z_feat
#         r, b = z_mean
        
#         # 1. Trích xuất trạng thái robot hiện tại
#         xr, yr, theta = self.x[0:3, 0]

#         # 2. Tính vị trí tuyệt đối của Landmark (Toạ độ Cartesian)
#         phi = normalize_angle(theta + b)
#         mx = xr + r * np.cos(phi)
#         my = yr + r * np.sin(phi)

#         # 3. Mở rộng Vector trạng thái x
#         self.x = np.vstack((self.x, [[mx], [my]]))

#        # --- 4. MỞ RỘNG MA TRẬN P VỚI TƯƠNG QUAN ---
#         n_old = self.P.shape[0]
        
#         # Jacobian của hàm chuyển đổi (Polar -> Cartesian) theo Robot [xr, yr, theta]
#         Gr = np.array([
#             [1, 0, -r * np.sin(phi)],
#             [0, 1,  r * np.cos(phi)]
#         ])
        
#         # Jacobian của hàm chuyển đổi theo phép đo [r, b]
#         Gz = np.array([
#             [np.cos(phi), -r * np.sin(phi)],
#             [np.sin(phi),  r * np.cos(phi)]
#         ])

#         # A. Tính tương quan giữa Map hiện tại và Landmark mới
#         # P_new_column = P_old * Gr.T
#         # Kích thước: (n_old x 3) * (3 x 2) = (n_old x 2)
#         P_robot_map = self.P[:, :3] 
#         P_cross = P_robot_map @ Gr.T

#         # B. Tính hiệp phương sai tự thân của Landmark mới (Uncertainty)
#         # P_ll = Gr * P_robot * Gr.T + Gz * R_obs * Gz.T
#         P_robot_only = self.P[0:3, 0:3]
#         P_ll = Gr @ P_robot_only @ Gr.T + Gz @ R_obs @ Gz.T

#         # C. Ghép vào ma trận P mới
#         P_new = np.zeros((n_old + 2, n_old + 2))
#         P_new[:n_old, :n_old] = self.P           # Map cũ
#         P_new[:n_old, n_old:] = P_cross         # Tương quan (Cột phải)
#         P_new[n_old:, :n_old] = P_cross.T       # Tương quan (Hàng dưới)
#         P_new[n_old:, n_old:] = P_ll            # Landmark mới

#         self.P = P_new

#         # 5. Cập nhật các biến quản lý
#         self.num_landmarks += 1
#         self.landmark_score.append(2.0)
        
#         if hasattr(self, 'lm_observed'):
#             self.lm_observed = np.append(self.lm_observed, True)


#     def remove_landmark(self, lm_id):
#         """
#         Xóa landmark khỏi state, covariance và metadata.
#         """
#         if lm_id >= self.num_landmarks:
#             return

#         idx = 3 + 2 * lm_id

#         # 1. Xóa khỏi vector trạng thái x (axis=0 vì x là vector cột n x 1)
#         self.x = np.delete(self.x, [idx, idx + 1], axis=0)

#         # 2. Xóa khỏi ma trận hiệp phương sai P (cả hàng và cột)
#         self.P = np.delete(self.P, [idx, idx + 1], axis=0)
#         self.P = np.delete(self.P, [idx, idx + 1], axis=1)

#         # 3. Cập nhật metadata
#         self.landmark_score.pop(lm_id)
#         self.num_landmarks -= 1
        
#         # Xóa trạng thái quan sát (nếu có dùng mảng numpy)
#         if hasattr(self, 'lm_observed'):
#             self.lm_observed = np.delete(self.lm_observed, lm_id)


#     # =========================
#     # 4. FEATURE EXTRACTION FROM LASER SCAN
#     # =========================
#     def extract_features_from_scan(self, scan):
#         ranges = np.array(scan.ranges)
#         indices = np.arange(len(ranges))

#         valid = np.isfinite(ranges)
#         ranges = ranges[valid]
#         angles = (scan.angle_min + indices * scan.angle_increment)[valid]
#         valid_indices = indices[valid]
        
#         xs = ranges * np.cos(angles)
#         ys = ranges * np.sin(angles)

#         point_cloud = np.column_stack((xs, ys, ranges, angles, valid_indices))

#         segment_clusters = self.segment_scan(point_cloud, 0.3, 3)

#         # 2. Chạy trích xuất đặc trưng cho TỪNG cụm
#         curv_pts = []
#         for point_cluster in segment_clusters:
#             # point_cluster là mảng (N, 5) chứa [x, y, r, b, id]
            
#             features = self.extract_curvature_points(
#                 point_cluster,
#                 k=5, 
#                 curvature_threshold=0.185,
#                 range_min=0.5,
#                 range_max=10.0
#             )
            
#             if len(features) > 0:
#                 curv_pts.extend(features)

#         if len(curv_pts) < 1:
#             return []
        
#         clusters = self.cluster_features(curv_pts,
#                      angle_thresh=0.02)

#         measurements = [self.cluster_to_feature(c) for c in clusters]
#         return measurements


#     def segment_scan(self, point_cloud, threshold=0.2, min_points=3):
#         """
#         points_cloud: np.array shape (N, 5) chứa [x, y, r, b, id]
#         """
#         if len(point_cloud) < 2: return []

#         # Tính khoảng cách Euclidean từ cột x (0) và y (1)
#         diffs = np.diff(point_cloud[:, :2], axis=0)
#         dist_sq_array = np.sum(diffs**2, axis=1)
        
#         thresh_sq = threshold**2
#         clusters = []
#         current_cluster = [point_cloud[0]]

#         for i in range(len(dist_sq_array)):
#             # Kiểm tra thêm: Nếu index nhảy bậc quá xa, ta chủ động tách cụm
#             idx_diff = point_cloud[i+1, 4] - point_cloud[i, 4]
            
#             if dist_sq_array[i] < thresh_sq and idx_diff < 5: # 5 là ngưỡng nhảy index tùy chọn
#                 current_cluster.append(point_cloud[i+1])
#             else:
#                 if len(current_cluster) >= min_points:
#                     clusters.append(np.array(current_cluster))
#                 current_cluster = [point_cloud[i+1]]
        
#         # Cụm cuối
#         if len(current_cluster) >= min_points:
#             clusters.append(np.array(current_cluster))

#         # 3. Xử lý khép vòng 360 độ
#         if len(clusters) > 1:
#             first_pt = clusters[0][0]   
#             last_pt = clusters[-1][-1]   
#             dist_wrap_sq = np.sum((first_pt[:2] - last_pt[:2])**2)
#             if dist_wrap_sq < thresh_sq:
#                 clusters[0] = np.vstack((clusters[-1], clusters[0]))
#                 clusters.pop()

#         return clusters
    

#     def extract_curvature_points(self, point_cluster, 
#                                 k=5, 
#                                 curvature_threshold=0.13, 
#                                 range_min=0.5, 
#                                 range_max=10.0):
#         """
#         point_cluster: np.array shape (N, 5) -> [x, y, r, b, id]
#         """
#         n = len(point_cluster)
#         if n < 2 * k + 1:
#             return []

#         # 1. Tính toán độ cong cho vùng trung tâm (từ k đến n-k-1)
#         curv_indices = []
#         for i in range(k, n - k):
#             if point_cluster[i, 2] < range_min or point_cluster[i, 2] > range_max:
#                 continue
                
#             neighbors = point_cluster[i-k : i+k+1, 0:2]
#             curv = self.compute_curvature(neighbors)
#             if curv > curvature_threshold:
#                 curv_indices.append(i)

#         if not curv_indices:
#             return []

#         # 2. Xây dựng danh sách kết quả (Sử dụng slicing để tránh duplicate)
#         first_curv = curv_indices[0]
#         last_curv = curv_indices[-1]
        
#         final_results = []
#         # Phần bù đầu
#         if first_curv == k:
#             final_results.extend(point_cluster[0:k])
            
#         # Phần thân (các điểm thực sự vượt ngưỡng)
#         final_results.extend(point_cluster[curv_indices])
            
#         # Phần bù cuối
#         if last_curv == n - k - 1:
#             final_results.extend(point_cluster[n-k : n])

#         return final_results
    
    
#     def compute_curvature(self, points):
#         # points: Nx2 array (x, y)
#         if len(points) < 3:
#             return 0.0

#         # Tính hiệp phương sai thủ công cho ma trận 2x2 (Nhanh hơn gọi np.cov)
#         centered = points - np.mean(points, axis=0)
#         # cov = [[var_x, cov_xy], [cov_xy, var_y]]
#         cov = (centered.T @ centered) / (len(points) - 1)

#         # Tính trị riêng cho ma trận 2x2 bằng công thức nghiệm phương trình bậc 2
#         # trace = lambda1 + lambda2, det = lambda1 * lambda2
#         trace = cov[0, 0] + cov[1, 1]
#         det = cov[0, 0] * cov[1, 1] - cov[0, 1]**2
        
#         # Tính lambda_min (trị riêng nhỏ hơn)
#         # lambda = (trace - sqrt(trace^2 - 4*det)) / 2
#         discriminant = max(0, trace**2 - 4 * det)
#         lambda_min = (trace - np.sqrt(discriminant)) / 2.0

#         if trace < 1e-6:
#             return 0.0

#         return lambda_min / trace


#     def cluster_features(self, points, angle_thresh=0.03):
#         if not points or len(points) == 0:
#             return []

#         # 1. Phân cụm thô theo góc
#         angle_clusters = self.cluster_by_angle(points, angle_thresh)
#         final_clusters = []

#         # 2. Lọc và làm sạch từng cụm
#         for cl in angle_clusters:
#             cl_array = np.array(cl)
#             # Tính khoảng cách trung bình của cụm (cột r là index 2)
#             mean_r = np.mean(cl_array[:, 2])
            
#             min_size = self.adaptive_min_cluster_size(mean_r)
#             if len(cl) >= min_size:
#                 # cl = self.filter_converged_points(cl_array, range_thresh=0.5)
#                 final_clusters.append(cl_array)

#         return final_clusters
    

#     def cluster_by_angle(self, points, angle_thresh=0.03):
#         """Phân cụm dựa trên chênh lệch góc beta (index 3)"""
#         if not points: return []
        
#         # Đảm bảo điểm được sắp xếp theo góc để duyệt tuyến tính
#         # points = sorted(points, key=lambda p: p[3])
        
#         clusters = []
#         current_cluster = [points[0]]

#         for i in range(1, len(points)):
#             # Tính delta beta (index 3)
#             db = abs(points[i][3] - points[i-1][3])
            
#             # if db > np.pi:
#             #     db = abs(db - 2 * np.pi)

#             if db < angle_thresh:
#                 current_cluster.append(points[i])
#             else:
#                 clusters.append(current_cluster)
#                 current_cluster = [points[i]]

#         clusters.append(current_cluster)

#         return clusters
    

#     def filter_converged_points(self, cluster, range_thresh=0.5):
#         if len(cluster) == 0: return cluster
        
#         rs = cluster[:, 2]
#         r_mean = np.median(rs)

#         mask = np.abs(rs - r_mean) < range_thresh
#         return cluster[mask]


#     def cluster_to_feature(self, cluster):
#         """
#         cluster: np.array shape (N, 5) -> [x, y, r, b, id]
#         Kết hợp Hiệp phương sai tự thân của cụm và Nhiễu mặc định self.R
#         """
#         cluster = np.array(cluster)
        
#         # 1. (Mean)
#         mean_r = np.mean(cluster[:, 2])
        
#         mean_b = np.arctan2(np.mean(np.sin(cluster[:, 3])), 
#                             np.mean(np.cos(cluster[:, 3])))
        
#         z_mean = np.array([mean_r, mean_b])

#         # 2.(Covariance)
#         if len(cluster) > 1:
#             z_cov_empirical = np.cov(cluster[:, 2:4].T)
#         else:
#             # Nếu chỉ có 1 điểm
#             z_cov_empirical = np.zeros((2, 2))

#         # 3.(Noise Floor)
#         z_cov = z_cov_empirical + self.R

#         return z_mean, z_cov

#     def adaptive_min_cluster_size(self, r):
#         if r < 1.0: return 4
#         if r < 3.0: return 3
#         if r < 2.0: return 2
#         return 1


#     # =========================
#     # 5. DATA ASSOCIATION
#     # =========================
#     def association(self, features, Z_pred_full, S_full, chi2_threshold=0.1):
#         """
#         features: list of (z_obs, R_obs) từ extract_features_from_scan
#         Z_pred_full: Vector (2*M,) dự báo [r1, b1, r2, b2...]
#         S_full: Ma trận (2*M, 2*M) hiệp phương sai dự báo
#         """
#         self.z = []
#         self.R_z = []
#         self.z_lm_ids = []
#         self.new_features = []
#         self.lm_observed = np.zeros(self.num_landmarks, dtype=bool)

#         if self.num_landmarks == 0:
#             for z_obs, R_obs in features:
#                 self.new_features.append((z_obs, R_obs))
#             return

#         # 1. Trích xuất các khối đường chéo S cho từng Landmark
#         # S_diag_blocks[lm_id] = ma trận 2x2
#         S_diag_blocks = [
#             S_full[2*j : 2*j+2, 2*j : 2*j+2] for j in range(self.num_landmarks)
#         ]

#         # 2. Tính toán tất cả ứng viên tiềm năng (Mahalanobis)
#         pairs = []  # (d2, feat_id, lm_id)

#         for i, (z_obs, R_obs) in enumerate(features):
#             # --- reshape ---
#             # z_obs = z_obs.reshape(1, 2)   # (1,2)
#             # Z_pred = Z_pred_full.reshape(-1, 2)   # (M,2)

#             # --- innovation ---
#             v = Z_pred_full.reshape(-1, 2) - z_obs   # (M,2)
#             v[:, 1] = (v[:, 1] + np.pi) % (2*np.pi) - np.pi

#             # gating thô
#             mask = (np.abs(v[:,0]) < 3.0) & (np.abs(v[:,1]) < np.pi/6)
#             valid_ids = np.where(mask)[0]

#             for lm_id in valid_ids:
#                 S_total = S_diag_blocks[lm_id] + R_obs
#                 try:
#                     d2 = v[lm_id].T @ np.linalg.solve(S_total, v[lm_id])
#                     if d2 < chi2_threshold:
#                         pairs.append((d2, i, lm_id))
#                 except np.linalg.LinAlgError:
#                     continue

#         # 3. Chọn cặp khớp One-to-One (GNN)
#         pairs.sort(key=lambda x: x[0])

#         used_feat = set()
#         used_lm   = set()
#         associations = {}

#         for d2, i, lm_id in pairs:
#             if i not in used_feat and lm_id not in used_lm:
#                 associations[i] = lm_id
#                 used_feat.add(i)
#                 used_lm.add(lm_id)

#         # 4. Phân loại Feature thành Landmark cũ hoặc Feature mới
#         for i, (z_obs, R_obs) in enumerate(features):
#             if i in associations:
#                 lm_id = associations[i]
#                 self.z.append(z_obs)
#                 self.R_z.append(R_obs)
#                 self.z_lm_ids.append(lm_id)
                
#                 self.lm_observed[lm_id] = True
#                 self.landmark_score[lm_id] += 2.0
#             else:
#                 # Trả về cả z và R để khởi tạo landmark mới chính xác hơn
#                 self.new_features.append((z_obs, R_obs))


# def main(args=None):
#     rclpy.init(args=args)

#     ekf_slam_node = UKFSLAM()

#     try:
#         rclpy.spin(ekf_slam_node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         ekf_slam_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()  
