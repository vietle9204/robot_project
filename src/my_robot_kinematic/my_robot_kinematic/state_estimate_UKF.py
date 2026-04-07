# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# import math
# import numpy as np
# from geometry_msgs.msg import TwistStamped
# from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Imu
# from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# def angle_normalize(a):
#     return (a + math.pi) % (2 * math.pi) - math.pi

# class state_estimate(Node):
#     def __init__(self):
#         super().__init__('odom_node')

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )

#         # Parameters
#         self.declare_parameter('vel_encoder_topic', 'robot1/vel_encoder/data')
#         self.declare_parameter('odometry_topic', '/odometry/data')
#         self.declare_parameter('imu_topic', 'robot1/imu/data')
        
#         self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').value
#         self.odometry_topic = self.get_parameter('odometry_topic').value
#         self.imu_topic = self.get_parameter('imu_topic').value
        
#         # UKF
#         self.x_k = np.zeros((5, 1))  # [x, y, theta, v, w]
        
#         self.Q_k = np.diag([0.5, 1.5])
#         self.P_k = np.eye(5) * 0.01  

#         self.imu_theta = 0.0
#         self.imu_last_time = None
        
#         # Sigma Point Weights
#         self.n_x = 5
#         self.n_w = 2
#         self.L = self.n_x + self.n_w
#         self.alpha = 1e-3
#         self.beta = 2
#         self.kappa = 0
#         self.lam = self.alpha**2 * (self.L + self.kappa) - self.L
        
#         self.Wm = np.full(2 * self.L + 1, 1 / (2 * (self.L + self.lam)))
#         self.Wc = self.Wm.copy()
#         self.Wm[0] = self.lam / (self.L + self.lam)
#         self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)

#         # Filter Variables
#         self.timer_period = 0.01  # 100Hz
#         self.timer = self.create_timer(self.timer_period, self.UKF_prediction)

#         # Subscriptions
#         self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
#         self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
#         self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)


#     def generate_sigma_points(self, x, P, Q):
#         x_aug = np.zeros((self.L, 1))
#         x_aug[:self.n_x] = x
        
#         P_aug = np.zeros((self.L, self.L))
#         P_aug[:self.n_x, :self.n_x] = P
#         P_aug[self.n_x:, self.n_x:] = Q

#         # Dùng SVD để lấy căn bậc hai ma trận (ổn định hơn Cholesky)
#         u, s, vh = np.linalg.svd(P_aug)
#         S = u @ np.diag(np.sqrt(np.maximum(s, 0)))
        
#         sigma = np.zeros((self.L, 2 * self.L + 1))
#         sigma[:, 0] = x_aug[:, 0]
#         scale = np.sqrt(self.L + self.lam)
        
#         for i in range(self.L):
#             sigma[:, i + 1] = x_aug[:, 0] + scale * S[:, i]
#             sigma[:, i + 1 + self.L] = x_aug[:, 0] - scale * S[:, i]
#         return sigma

#     def UKF_prediction(self):
#         dt = self.timer_period
#         sigma = self.generate_sigma_points(self.x_k, self.P_k, self.Q_k)
#         sigma_pred = np.zeros((self.n_x, 2 * self.L + 1))

#         for i in range(2 * self.L + 1):
#             x, y, theta, v, w = sigma[:5, i]
#             nv, nw = sigma[5:7, i]
            
#             v_noisy = v + nv
#             w_noisy = w + nw
            

#             new_theta = theta + w_noisy * dt
#             if(w_noisy <= 0.5):
#                 new_x = x + v_noisy * math.cos(theta) * dt
#                 new_y = y + v_noisy * math.sin(theta) * dt
#             else:
#                 new_x = x + v_noisy/w_noisy*(math.sin(new_theta) - math.sin(theta))
#                 new_y = y - v_noisy/w_noisy*(math.cos(new_theta) - math.cos(theta))
            
#             sigma_pred[:, i] = [new_x, new_y, angle_normalize(new_theta), v_noisy, w_noisy]

#         # Tính toán Mean mới (Circular mean cho theta)
#         x_new = np.zeros((self.n_x, 1))
#         x_new[[0,1,3,4], 0] = np.sum(self.Wm * sigma_pred[[0,1,3,4], :], axis=1)
        
#         s_sum = np.sum(self.Wm * np.sin(sigma_pred[2, :]))
#         c_sum = np.sum(self.Wm * np.cos(sigma_pred[2, :]))
#         x_new[2, 0] = math.atan2(s_sum, c_sum)

#         # Tính toán Covariance mới
#         P_new = np.zeros((self.n_x, self.n_x))
#         for i in range(2 * self.L + 1):
#             dx = sigma_pred[:, i:i+1] - x_new
#             dx[2, 0] = angle_normalize(dx[2, 0])
#             P_new += self.Wc[i] * (dx @ dx.T)
        
#         self.x_k = x_new
#         self.P_k = 0.5 * (P_new + P_new.T) + np.eye(self.n_x) * 1e-6
#         self.sigma_c = sigma_pred # Lưu lại cho bước update

        

#     def imu_callback(self, msg: Imu):
#             stamp = msg.header.stamp
#             if self.imu_last_time is None:
#                 self.imu_last_time = stamp
#                 return

#             dt = ((stamp.sec - self.imu_last_time.sec) +
#                   (stamp.nanosec - self.imu_last_time.nanosec) * 1e-9)
#             # guard dt
#             if dt <= 0.0:
#                 # ignore bad dt (too large or non-positive)
#                 self.imu_last_time = stamp
#                 return
#             self.imu_last_time = stamp

#             angular_vel_yaw = float(msg.angular_velocity.z)

#             #================
#             if math.fabs(angular_vel_yaw) < 0.05: 
#                 angular_vel_yaw = 0.0
#             else:
#                 angular_vel_yaw =  angular_vel_yaw #- 0.004
#             #=========================

#             self.imu_theta += angular_vel_yaw*dt
#             self.imu_theta = angle_normalize(self.imu_theta)

#             z = np.array([[self.imu_theta],
#                           [angular_vel_yaw]])
            
#             H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
#                           [0.0, 0.0, 0.0, 0.0, 1.0]])

#             R = np.array([[0.01, 0.00],
#                           [0.00, 0.0035]])
    
#             self.UKF_update(z, H, R, 0)

#             self.publish_odom(msg.header.stamp)

    

#     def encoder_callback(self, msg: TwistStamped):
#         z = np.array([[msg.twist.linear.x], [msg.twist.angular.z]])

#         H = np.array([[0,0,0,1,0], 
#                       [0,0,0,0,1]])
        
#         R = np.diag([0.002, 0.003])

#         self.UKF_update(z, H, R, None)

#         self.publish_odom(msg.header.stamp)

#     def UKF_update(self, z, H, R, yaw_index):
#         if not hasattr(self, 'sigma_c'): return # Đợi prediction đầu tiên
        
#         n_z = z.shape[0]

#         full_sigma = self.generate_sigma_points(self.x_k, self.P_k, self.Q_k)
#         self.sigma_c = full_sigma[:self.n_x, :] # Chỉ lấy 5 hàng trạng thái

#         Z_sigma = H @ self.sigma_c
        
#         # Predicted measurement mean
#         z_pred = np.zeros((n_z, 1))
#         if yaw_index is not None:
#             s_sum = np.sum(self.Wm * np.sin(Z_sigma[yaw_index, :]))
#             c_sum = np.sum(self.Wm * np.cos(Z_sigma[yaw_index, :]))
#             z_pred[yaw_index, 0] = math.atan2(s_sum, c_sum)
#             other_idx = [i for i in range(n_z) if i != yaw_index]
#             if other_idx:
#                 z_pred[other_idx, 0] = np.sum(self.Wm * Z_sigma[other_idx, :], axis=1)
#         else:
#             z_pred[:, 0] = np.sum(self.Wm * Z_sigma, axis=1)

#         # Covariances
#         Pzz = np.zeros((n_z, n_z))
#         Pxz = np.zeros((self.n_x, n_z))
        
#         for i in range(2 * self.L + 1):
#             dz = Z_sigma[:, i:i+1] - z_pred
#             if yaw_index is not None:
#                 dz[yaw_index, 0] = angle_normalize(dz[yaw_index, 0])
            
#             dx = self.sigma_c[:, i:i+1] - self.x_k
#             dx[2, 0] = angle_normalize(dx[2, 0])
            
#             Pzz += self.Wc[i] * (dz @ dz.T)
#             Pxz += self.Wc[i] * (dx @ dz.T)
            
#         Pzz += R
        
#         try:
#             K = np.linalg.solve(Pzz, Pxz.T).T
#             dz_final = z - z_pred
#             if yaw_index is not None:
#                 dz_final[yaw_index, 0] = angle_normalize(dz_final[yaw_index, 0])
                
#             self.x_k += K @ dz_final
#             self.x_k[2, 0] = angle_normalize(self.x_k[2, 0])
#             self.P_k -= K @ Pzz @ K.T
#             self.P_k = 0.5 * (self.P_k + self.P_k.T)
#         except np.linalg.LinAlgError:
#             self.get_logger().error("Update failed: Singular Matrix")

#     def publish_odom(self, stamp):
#         # prepare covariances for nav_msgs/Odometry (6x6 flattened)
#         pose_cov = np.zeros((6,6))
#         pose_cov[0,0] = self.P_k[0,0]
#         pose_cov[0,1] = self.P_k[0,1]
#         pose_cov[0,5] = self.P_k[0,2]
#         pose_cov[1,0] = self.P_k[1,0]
#         pose_cov[1,1] = self.P_k[1,1]
#         pose_cov[1,5] = self.P_k[1,2]
#         pose_cov[5,0] = self.P_k[2,0]
#         pose_cov[5,1] = self.P_k[2,1]
#         pose_cov[5,5] = self.P_k[2,2]

#         twist_cov = np.zeros((6,6))
#         twist_cov[0,0] = self.P_k[3,3]
#         twist_cov[5,5] = self.P_k[4,4]

#         odom = Odometry()
#         odom.header.stamp = stamp
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = float(self.x_k[0,0])
#         odom.pose.pose.position.y = float(self.x_k[1,0])
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = math.sin(float(self.x_k[2,0]) / 2.0)
#         odom.pose.pose.orientation.w = math.cos(float(self.x_k[2,0]) / 2.0)

#         odom.twist.twist.linear.x = self.x_k[3, 0]
#         odom.twist.twist.angular.z = self.x_k[4, 0]

#         odom.pose.covariance = pose_cov.flatten().tolist()
#         odom.twist.covariance = twist_cov.flatten().tolist()

#         self.odom_pub.publish(odom) 

# def main(args=None):
#     rclpy.init(args=args)
#     node = state_estimate()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.linalg import cholesky

def angle_normalize(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

class state_estimate(Node):
    def __init__(self):
        super().__init__('odom_node')

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

        # Parameters
        self.declare_parameter('vel_encoder_topic', 'robot1/vel_encoder/data')
        self.declare_parameter('odometry_topic', '/odometry/data')
        self.declare_parameter('imu_topic', 'robot1/imu/data')
        
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

        
        # Sigma Point Weights
        self.n_x = 5
        self.n_w = 2
        self.L = self.n_x + self.n_w

        self.alpha = 1.0
        self.beta = 2
        self.kappa = 0
        self.lam = self.alpha**2 * (self.L + self.kappa) - self.L
        
        self.Wm = np.full(2 * self.L + 1, 1 / (2 * (self.L + self.lam)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lam / (self.L + self.lam)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)

        # UKF
        self.x_k = np.ones((5, 1)) * 1e-6  # [x, y, theta, v, w]
        self.Q_k = np.diag([0.1, 0.5])
        self.P_k = np.eye(5) * 0.1

        # Filter 
        self.imu_theta = 0.0
        self.imu_last_time = None
        self.last_time = self.get_clock().now()
        self.UKF_init = False
        self.timer_period = 0.01  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.UKF_prediction)

        # Subscriptions
        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos2)


    def generate_sigma_points(self, x, P, Q):
        x_aug = np.zeros((self.L, 1))
        x_aug[:self.n_x] = x
        
        P_aug = np.zeros((self.L, self.L))
        P_aug[:self.n_x, :self.n_x] = P
        P_aug[self.n_x:, self.n_x:] = Q
        P_aug = 0.5 * (P_aug + P_aug.T) + 1e-9 * np.eye(P_aug.shape[0])

        # sqrt(P)
        try:
            S = np.linalg.cholesky(P_aug)
        except np.linalg.LinAlgError:
            u, s, vh = np.linalg.svd(P_aug)
            S = u @ np.diag(np.sqrt(np.maximum(s, 0.0)))
        
        sigma = np.zeros((self.L, 2 * self.L + 1))
        sigma[:, 0] = x_aug[:, 0]
        scale = np.sqrt(self.L + self.lam)
        
        for i in range(self.L):
            sigma[:, i + 1] = x_aug[:, 0] + scale * S[:, i]
            sigma[:, i + 1 + self.L] = x_aug[:, 0] - scale * S[:, i]
        return sigma

    def UKF_prediction(self):
        # dt = self.timer_period
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        dt = max(1e-6, min(dt, 2*self.timer_period))
        self.last_time = now
        if dt <= 1e-4 or dt > 0.1:
            return
        sigma = self.generate_sigma_points(self.x_k, self.P_k, self.Q_k)
        sigma_pred = np.zeros((self.n_x, 2 * self.L + 1))

        for i in range(2 * self.L + 1):
            x, y, theta, v, w = sigma[:5, i]
            nv, nw = sigma[5:7, i]
            
            v_noisy = v + nv
            w_noisy = w + nw
            new_theta = theta + w_noisy * dt

            if(w_noisy <= 0.005):
                mid_theta = theta + 0.5 * w_noisy * dt
                new_x = x + v_noisy * math.cos(mid_theta) * dt
                new_y = y + v_noisy * math.sin(mid_theta) * dt
            else:
                new_x = x + v_noisy/w_noisy*(math.sin(new_theta) - math.sin(theta))
                new_y = y - v_noisy/w_noisy*(math.cos(new_theta) - math.cos(theta))
            
            sigma_pred[:, i] = [new_x, new_y, angle_normalize(new_theta), v_noisy, w_noisy]

        # Tính toán Mean mới (Circular mean cho theta)
        x_new = np.zeros((self.n_x, 1))
        x_new[[0,1,3,4], 0] = np.sum(self.Wm * sigma_pred[[0,1,3,4], :], axis=1)
        
        s_sum = np.sum(self.Wm * np.sin(sigma_pred[2, :]))
        c_sum = np.sum(self.Wm * np.cos(sigma_pred[2, :]))
        x_new[2, 0] = math.atan2(s_sum, c_sum)

        # Tính toán Covariance mới
        P_new = np.zeros((self.n_x, self.n_x))
        for i in range(2 * self.L + 1):
            dx = sigma_pred[:, i:i+1] - x_new
            dx[2, 0] = angle_normalize(dx[2, 0])
            P_new += self.Wc[i] * (dx @ dx.T)
        
        self.x_k = x_new
        self.P_k = 0.5 * (P_new + P_new.T) + np.eye(self.n_x) * 1e-9
        # self.sigma_c = sigma_pred 
        self.UKF_init = True

        
    def imu_callback(self, msg: Imu):
            stamp = msg.header.stamp
            if self.imu_last_time is None:
                self.imu_last_time = stamp
                return

            dt = ((stamp.sec - self.imu_last_time.sec) +
                  (stamp.nanosec - self.imu_last_time.nanosec) * 1e-9)
            # guard dt
            if dt <= 0.0:
                # ignore bad dt (too large or non-positive)
                self.imu_last_time = stamp
                return
            self.imu_last_time = stamp

            angular_vel_yaw = float(msg.angular_velocity.z)

            #================
            if math.fabs(angular_vel_yaw) < 0.05: 
                angular_vel_yaw = 0.0
            else:
                angular_vel_yaw =  angular_vel_yaw #- 0.004
            #=========================

            self.imu_theta += angular_vel_yaw*dt
            self.imu_theta = angle_normalize(self.imu_theta)

            if self.UKF_init is False:
                return
            
            # self.UKF_prediction()
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"imu t_msg: {t:.6f} | theta: {self.imu_theta:.6f} | wz: {angular_vel_yaw:.6f}"
            )

            z = np.array([[self.imu_theta],
                          [angular_vel_yaw]])
            
            H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0, 0.0, 1.0]])

            R = np.array([[0.005, 0.00],
                          [0.00, 0.00036]])
    
            if self.UKF_update(z, H, R, 0):
                self.publish_odom(msg.header.stamp)


    def encoder_callback(self, msg: TwistStamped):
        if self.UKF_init is False:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.get_logger().info(f"enc t_msg: {t:.6f} | v: {msg.twist.linear.x:.6f}, w: {msg.twist.angular.z:.6f}")
        # self.UKF_prediction()

        z = np.array([[msg.twist.linear.x], [msg.twist.angular.z]])

        H = np.array([[0,0,0,1,0], 
                      [0,0,0,0,1]])
        
        R = np.diag([0.0003, 0.0004])

        if self.UKF_update(z, H, R, None):
            self.publish_odom(msg.header.stamp)

        

    def UKF_update(self, z, H, R, yaw_index):
        # if not hasattr(self, 'sigma_c'): return # Đợi prediction đầu tiên
        if self.UKF_init is False: return
        
        n_z = z.shape[0]

        full_sigma = self.generate_sigma_points(self.x_k, self.P_k, self.Q_k)
        self.sigma_c = full_sigma[:self.n_x, :] # Chỉ lấy 5 hàng trạng thái

        Z_sigma = H @ self.sigma_c
        
        # Predicted measurement mean
        z_pred = np.zeros((n_z, 1))
        if yaw_index is not None:
            s_sum = np.sum(self.Wm * np.sin(Z_sigma[yaw_index, :]))
            c_sum = np.sum(self.Wm * np.cos(Z_sigma[yaw_index, :]))
            z_pred[yaw_index, 0] = math.atan2(s_sum, c_sum)
            other_idx = [i for i in range(n_z) if i != yaw_index]
            if other_idx:
                z_pred[other_idx, 0] = np.sum(self.Wm * Z_sigma[other_idx, :], axis=1)
        else:
            z_pred[:, 0] = np.sum(self.Wm * Z_sigma, axis=1)

        # Covariances
        Pzz = np.zeros((n_z, n_z))
        Pxz = np.zeros((self.n_x, n_z))
        
        for i in range(2 * self.L + 1):
            dz = Z_sigma[:, i:i+1] - z_pred
            if yaw_index is not None:
                dz[yaw_index, 0] = angle_normalize(dz[yaw_index, 0])
            
            dx = self.sigma_c[:, i:i+1] - self.x_k
            dx[2, 0] = angle_normalize(dx[2, 0])
            
            Pzz += self.Wc[i] * (dz @ dz.T)
            Pxz += self.Wc[i] * (dx @ dz.T)
            
        Pzz += R
        
        try:
            K = np.linalg.solve(Pzz, Pxz.T).T
            dz_final = z - z_pred
            if yaw_index is not None:
                dz_final[yaw_index, 0] = angle_normalize(dz_final[yaw_index, 0])
                
            self.x_k += K @ dz_final
            self.x_k[2, 0] = angle_normalize(self.x_k[2, 0])
            I = np.eye(self.n_x)
            IK = I - K @ H
            self.P_k = IK @ self.P_k @ IK.T + K @ R @ K.T

            self.P_k = 0.5 * (self.P_k + self.P_k.T) + np.eye(self.n_x)*1e-9
            return True
        except np.linalg.LinAlgError:
            self.get_logger().error("Update failed: Singular Matrix")

        return False

    def publish_odom(self, stamp):
        # prepare covariances for nav_msgs/Odometry (6x6 flattened)
        pose_cov = np.zeros((6,6))
        pose_cov[0,0] = self.P_k[0,0]
        pose_cov[0,1] = self.P_k[0,1]
        pose_cov[0,5] = self.P_k[0,2]
        pose_cov[1,0] = self.P_k[1,0]
        pose_cov[1,1] = self.P_k[1,1]
        pose_cov[1,5] = self.P_k[1,2]
        pose_cov[5,0] = self.P_k[2,0]
        pose_cov[5,1] = self.P_k[2,1]
        pose_cov[5,5] = self.P_k[2,2]

        twist_cov = np.zeros((6,6))
        twist_cov[0,0] = self.P_k[3,3]
        twist_cov[5,5] = self.P_k[4,4]

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x_k[0,0])
        odom.pose.pose.position.y = float(self.x_k[1,0])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(float(self.x_k[2,0]) / 2.0)
        odom.pose.pose.orientation.w = math.cos(float(self.x_k[2,0]) / 2.0)

        odom.twist.twist.linear.x = self.x_k[3, 0]
        odom.twist.twist.angular.z = self.x_k[4, 0]

        odom.pose.covariance = pose_cov.flatten().tolist()
        odom.twist.covariance = twist_cov.flatten().tolist()

        self.odom_pub.publish(odom) 

def main(args=None):
    rclpy.init(args=args)
    node = state_estimate()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()