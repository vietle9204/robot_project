#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from collections import deque
from rclpy.time import Time

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

def angle_normalize(a):
    return (a + math.pi) % (2 * math.pi) - math.pi

def normalize_angle_vec(a):
    return (a + np.pi) % (2*np.pi) - np.pi

class state_estimate(Node):
    def __init__(self):
        super().__init__('odom_node')
        # Parameters
        self.parameters()
        #Init UKF
        self.init_UKF()
        # Init subcription and publish
        self.init_pub_sub()

    def parameters(self):
        self.declare_parameter('vel_encoder_topic', 'robot1/vel_encoder/data')
        self.declare_parameter('odometry_topic', '/odometry/data')
        self.declare_parameter('imu_topic', 'robot1/imu/data')
        
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').value
        self.odometry_topic = self.get_parameter('odometry_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value

    def init_UKF(self):
        # state vector length
        self.n_x = 5
        self.n_w = 2
        self.L = self.n_x + self.n_w

        # define sigma point parameter
        self.alpha = 0.5
        self.beta = 2
        self.kappa = 0
        self.lam = self.alpha**2 * (self.L + self.kappa) - self.L
        self.scale = np.sqrt(self.L + self.lam)
        # sigma point weight
        self.Wm = np.full(2 * self.L + 1, 1 / (2 * (self.L + self.lam)))
        self.Wc = self.Wm.copy()
        self.Wm[0] = self.lam / (self.L + self.lam)
        self.Wc[0] = self.Wm[0] + (1 - self.alpha**2 + self.beta)

        # define state vector
        self.x_k = np.zeros((5, 1))   # [x, y, theta, v, w]
        self.Q_k = np.diag([0.0005, 0.0005])
        self.P_k = np.eye(5) * 0.1
        #define measurement vector
        self.z = np.zeros((8,1))
        self.measure_yaw_idx = (0,4,7)
        self.H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0],
                           [1.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0], 
                           [0.0, 0.0, 0.0, 0.0, 1.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0],])

        # Filter 
        self.timer_period = 0.02  # 100Hz
        self.timer = self.create_timer(self.timer_period, self.ST_process)

        self.last_time = self.get_clock().now().nanoseconds
        self.odom_time = None

        self.sigma_pred_c = None

    def init_pub_sub(self):
        # encoder
        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.last_enc_msg = None
        self.enc_odom = np.zeros((3,1)) # dead reckoning from encoder
        self.enc_buffer = deque(maxlen=20)
        self.enc_R = np.array([0.004, 0.004, 0.005, 0.001, 0.001])
        # imu
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
        self.last_imu_msg =  None
        self.imu_theta = 0.0
        self.imu_buffer = deque(maxlen=20)
        self.imu_R = np.array([0.006, 0.0036]) 
        # mag
        self.create_subscription(MagneticField, 'robot1/mag/data', self.mag_filt_cb, qos)
        self.last_mag_msg = None
        self.mag_yaw_base = None
        self.mag_yaw = None
        self.mag_slope = 0.0
        self.mag_buffer = deque(maxlen=20)
        self.mag_R = np.array([0.005])
        # publish odommetry
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos2)

    def encoder_callback(self, msg):
        if self.last_enc_msg is None:
            self.last_enc_msg = msg
            return
        self.enc_buffer.append(msg)

    def mag_filt_cb(self, msg):
        if self.last_mag_msg is None:
            self.last_mag_msg = msg
            self.mag_yaw_base = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
            self.mag_yaw = 0.0
            self.imu_theta = 0.0
            self.enc_odom[2,0] = 0.0
            self.x_k[2,0] = 0.0
            return
        self.mag_buffer.append(msg)

    def imu_callback(self, msg):
        if self.last_imu_msg is None:
            self.last_imu_msg = msg
            return
        self.imu_buffer.append(msg)
    
    def ST_process(self):
        if self.last_imu_msg is None or self.last_enc_msg is None or self.last_mag_msg is None:
            return
     
        #process data:
        imu_msg, enc_msg, mag_msg = None, None, None
        #enc
        if self.imu_buffer:
            imu_msg = self.imu_buffer[0]
            t_imu = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
            imu_flag = True
        else:
            imu_flag = False
        #imu
        if self.enc_buffer:
            enc_msg = self.enc_buffer[0]
            t_enc = enc_msg.header.stamp.sec + enc_msg.header.stamp.nanosec * 1e-9
            enc_flag = True
        else:
            enc_flag = False
        #mag
        if self.mag_buffer:
            mag_msg = self.mag_buffer[0]
            t_mag = mag_msg.header.stamp.sec + mag_msg.header.stamp.nanosec * 1e-9
            mag_flag = True    
        else:
            mag_flag = False

        #syns timestamp
        data = []
        if imu_flag: data.append((t_imu, "IMU"))
        if enc_flag: data.append((t_enc, "Encoder"))
        if mag_flag: data.append((t_mag, "Mag"))
        t_min, min_sensor_name = None, None
        if data:
            t_min, min_sensor_name = min(data)
 
        if self.odom_time is None:
            if t_min is not None:
                self.odom_time = t_min - 1e-6
                self.last_time = self.get_clock().now().nanoseconds* 1e-9 - 1e-6
            else:
                return
            
        if t_min is None:
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = now - self.last_time
            t_min = self.odom_time + 0.999*dt
            # return

        match min_sensor_name:
            case "IMU":
                if enc_flag and t_enc - t_imu > 0.01:
                    enc_flag = False
                if mag_flag and t_mag - t_imu > 0.01:
                    mag_flag = False
            case "Encoder":
                if imu_flag and t_imu - t_enc > 0.01:
                    imu_flag = False
                if mag_flag and t_mag - t_enc > 0.01:
                    mag_flag = False
            case "Mag":
                if imu_flag and t_imu - t_mag > 0.01:
                    imu_flag = False
                if enc_flag and t_enc - t_mag > 0.01:
                    enc_flag = False
            case None:                pass

        # process IMU
        if imu_flag:
            imu_last_time = self.last_imu_msg.header.stamp.sec + self.last_imu_msg.header.stamp.nanosec * 1e-9
            imu_dt = t_imu - imu_last_time

            angular_vel_yaw = float(imu_msg.angular_velocity.z)
            angular_vel_yaw =  angular_vel_yaw #- 0.004
            self.imu_theta += angular_vel_yaw*imu_dt
            imu_theta = angle_normalize(self.imu_theta)

            self.imu_buffer.popleft()
            self.last_imu_msg = imu_msg
        else:
            imu_last_time = self.last_imu_msg.header.stamp.sec + self.last_imu_msg.header.stamp.nanosec * 1e-9
            imu_dt = t_min - imu_last_time

            if imu_msg is not None:
                diff = float(imu_msg.angular_velocity.z) - float(self.last_imu_msg.angular_velocity.z)
                angular_vel_yaw = float(self.last_imu_msg.angular_velocity.z) + diff * (imu_dt) / (t_imu - imu_last_time)
                imu_theta = self.imu_theta + angular_vel_yaw * imu_dt
                imu_theta = angle_normalize(imu_theta)
            else:
                angular_vel_yaw = self.last_imu_msg.angular_velocity.z
                imu_theta = self.imu_theta + angular_vel_yaw * imu_dt
                imu_theta = angle_normalize(imu_theta)
        # process encoder
        if enc_flag:
            enc_last_time = self.last_enc_msg.header.stamp.sec + self.last_enc_msg.header.stamp.nanosec * 1e-9
            enc_dt = t_enc - enc_last_time

            enc_v = float(enc_msg.twist.linear.x)
            enc_w = float(enc_msg.twist.angular.z)
            self.enc_odom[0,0] += enc_v*math.cos(self.enc_odom[2,0])*enc_dt
            self.enc_odom[1,0] += enc_v*math.sin(self.enc_odom[2,0])*enc_dt
            self.enc_odom[2,0] += enc_w*enc_dt

            enc_odom = self.enc_odom.copy()
            enc_odom[2,0] = angle_normalize(enc_odom[2,0])

            self.enc_buffer.popleft()
            self.last_enc_msg = enc_msg     
        else:
            enc_last_time = self.last_enc_msg.header.stamp.sec + self.last_enc_msg.header.stamp.nanosec * 1e-9
            enc_next_odom = self.enc_odom.copy()
            enc_dt = t_min - enc_last_time

            if enc_msg is not None:
                diff_v = float(enc_msg.twist.linear.x) - float(self.last_enc_msg.twist.linear.x)
                diff_w = float(enc_msg.twist.angular.z) - float(self.last_enc_msg.twist.angular.z)
                enc_v = float(self.last_enc_msg.twist.linear.x) + diff_v * (enc_dt) / (t_enc - enc_last_time)
                enc_w = float(self.last_enc_msg.twist.angular.z) + diff_w * (enc_dt) / (t_enc - enc_last_time)

                enc_next_odom[0,0] += enc_v*math.cos(enc_next_odom[2,0])*enc_dt
                enc_next_odom[1,0] += enc_v*math.sin(enc_next_odom[2,0])*enc_dt
                enc_next_odom[2,0] += enc_w*enc_dt

                enc_odom = enc_next_odom
                enc_odom[2,0] = angle_normalize(enc_odom[2,0])
            else:
                enc_v = float(self.last_enc_msg.twist.linear.x)
                enc_w = float(self.last_enc_msg.twist.angular.z)

                enc_next_odom[0,0] += enc_v*math.cos(enc_next_odom[2,0])*enc_dt
                enc_next_odom[1,0] += enc_v*math.sin(enc_next_odom[2,0])*enc_dt
                enc_next_odom[2,0] += enc_w*enc_dt

                enc_odom = enc_next_odom
                enc_odom[2,0] = angle_normalize(enc_odom[2,0])
        #process mag
        if mag_flag:
            mag_last_time = self.last_mag_msg.header.stamp.sec + self.last_mag_msg.header.stamp.nanosec * 1e-9
            mag_yaw = math.atan2(mag_msg.magnetic_field.y, mag_msg.magnetic_field.x) - self.mag_yaw_base
            mag_yaw = angle_normalize(mag_yaw)

            self.mag_slope = angle_normalize(mag_yaw - self.mag_yaw) / (t_mag - mag_last_time)
            if math.fabs(self.mag_slope) <= 0.1:
                self.mag_slope = 0.0
            self.mag_buffer.popleft()
            self.mag_yaw = mag_yaw
        else:
            mag_last_time = self.last_mag_msg.header.stamp.sec + self.last_mag_msg.header.stamp.nanosec * 1e-9 
            if mag_msg is not None:
                next_mag_yaw = math.atan2(mag_msg.magnetic_field.y, mag_msg.magnetic_field.x) - self.mag_yaw_base
                diff = angle_normalize(next_mag_yaw - self.mag_yaw)
                mag_yaw = self.mag_yaw + diff * (t_min - mag_last_time) / (t_mag - mag_last_time)
                mag_yaw = angle_normalize(mag_yaw)
            else:
                mag_dt = t_min - mag_last_time 
                mag_yaw = angle_normalize(self.mag_yaw + self.mag_slope * mag_dt)

        # Predic
        Q = self.Q_k.copy()
        Q[0,0] = Q[0,0] + 4.0*max(0.0, -4*1e-4 + (math.fabs(enc_v - self.x_k[3,0])**2)) + 10*max(0.0, -10*1e-4+ (math.fabs(enc_v  - self.last_enc_msg.twist.linear.x)**2))
        Q[1,1] = Q[1,1] + 4.0*max(0.0, -10*1e-4 + (math.fabs(0.5*(enc_w + angular_vel_yaw) - self.x_k[4,0])**2)) + 10*max(0.0, -5*1e-3 + (math.fabs(0.5*(enc_w - self.last_enc_msg.twist.angular.z) + 0.5*(angular_vel_yaw - self.last_imu_msg.angular_velocity.z))**2))

        predict_dt = t_min - self.odom_time
        self.UKF_prediction(predict_dt, Q)

        self.odom_time = t_min
        now = self.get_clock().now().nanoseconds * 1e-9
        self.last_time = now
        
        # update

        self.z[0,0] = imu_theta
        self.z[1,0] = angular_vel_yaw
        self.z[2:5,0] = enc_odom[:3,0]
        self.z[5,0] = enc_v
        self.z[6,0] = enc_w
        self.z[7,0] = mag_yaw 

        imu_R = self.imu_R.copy()
        imu_R[0] = imu_R[0] + (0.02*math.fabs(self.imu_theta))**2
        if imu_flag:
            imu_R[0] = imu_R[0] + 0.0001
        enc_R = self.enc_R.copy()
        enc_R[0] = enc_R[0] + (0.0001*(math.fabs(self.enc_odom[0,0])**2 + math.fabs(self.enc_odom[1,0])**2))
        enc_R[1] = enc_R[1] + (0.0001*(math.fabs(self.enc_odom[0,0])**2 + math.fabs(self.enc_odom[1,0])**2))
        enc_R[2] = enc_R[2] + (0.01*math.fabs(self.enc_odom[2,0]))**2
        if enc_flag:
            enc_R[0] = enc_R[0] + 0.0001
            enc_R[1] = enc_R[1] + 0.0001
            enc_R[2] = enc_R[2] + 0.0001
        mag_R = self.mag_R.copy()
        if mag_flag:
            mag_R[0] = mag_R[0] + 0.0001
        R = np.diag([imu_R[0], imu_R[1], enc_R[0], enc_R[1], enc_R[2], self.enc_R[3], self.enc_R[4], mag_R[0]])
                
        self.UKF_update(self.z, self.H, R, (0,4,7))
    
        ros_stamp = Time(seconds=self.odom_time).to_msg()
        self.publish_odom(ros_stamp)
        self.get_logger().info("Odom published: x,y,theta = {:.4f}, {:.4f}, {:.4f}".format(
            self.x_k[0,0], self.x_k[1,0], self.x_k[2,0]
        ))
        self.get_logger().info("enc_buffer length: {}, imu_buffer length: {}, mag_buffer length: {}".format(len(self.enc_buffer)+enc_flag, len(self.imu_buffer)+imu_flag, len(self.mag_buffer)+mag_flag))

        if(self.enc_buffer or self.imu_buffer):
            self.ST_process()

    def generate_sigma_points(self, x, P, Q):
        x_aug = np.zeros((self.L, 1))
        x_aug[:self.n_x] = x
        
        P_aug = np.zeros((self.L, self.L))
        P_aug[:self.n_x, :self.n_x] = P
        P_aug[self.n_x:, self.n_x:] = Q
        P_aug = 0.5 * (P_aug + P_aug.T) + 1e-9 * np.eye(P_aug.shape[0])

        # sqrt(P)
        S = np.linalg.cholesky(P_aug) * self.scale
        
        sigma = np.zeros((self.L, 2 * self.L + 1))
        sigma[:, 0] = x_aug[:, 0]
        sigma[:, 1:self.L+1] = x_aug + S
        sigma[:, self.L+1:] = x_aug - S
        return sigma

    def UKF_prediction(self, dt, Q):
        sigma = self.generate_sigma_points(self.x_k, self.P_k, Q)
        sigma_pred = np.zeros((self.n_x, 2 * self.L + 1))

        v = sigma[3, :] + sigma[5, :]
        w = sigma[4, :] + sigma[6, :]

        theta = sigma[2, :]
        sin_theta = np.sin(theta)
        cos_theta = np.cos(theta)

        new_theta = theta + w * dt
        sin_new_theta = np.sin(new_theta)
        cos_new_theta = np.cos(new_theta)

        mask = np.abs(w) <= 1e-3

        new_x = np.where(
            mask,
            sigma[0, :] + v * cos_theta * dt,
            sigma[0, :] + v/w * (sin_new_theta - sin_theta)
        )

        new_y = np.where(
            mask,
            sigma[1, :] + v * np.sin(theta) * dt,
            sigma[1, :] - v/w * (cos_new_theta - cos_theta)
        )
        sigma_pred = np.vstack([new_x, new_y, new_theta, v, w])
        
        # Tính toán Mean mới (Circular mean cho theta)
        x_new = np.zeros((self.n_x, 1))
        x_new[[0,1,3,4], 0] = np.sum(self.Wm * sigma_pred[[0,1,3,4], :], axis=1)
        
        s_sum = np.sum(self.Wm * np.sin(sigma_pred[2, :]))
        c_sum = np.sum(self.Wm * np.cos(sigma_pred[2, :]))
        x_new[2, 0] = math.atan2(s_sum, c_sum)

        # Tính toán Covariance mới
        P_new = np.zeros((self.n_x, self.n_x))
        dx = sigma_pred - x_new
        dx[2, :] = normalize_angle_vec(dx[2, :]) #nomarlization

        dx_weighted = dx * self.Wc   
        P_new = dx_weighted @ dx.T
        
        self.x_k = x_new
        self.P_k = 0.5 * (P_new + P_new.T) + np.eye(self.n_x) * 1e-9
        self.sigma_pred_c = sigma_pred[:self.n_x, :]


    def UKF_update(self, z, H, R, yaw_index):
        if yaw_index is not None:
            yaw_idx = np.atleast_1d(yaw_index)

        # full_sigma = self.generate_sigma_points(self.x_k, self.P_k, Q)
        # self.sigma_pred_c = full_sigma[:self.n_x, :] # Chỉ lấy 5 hàng trạng thái
        Z_sigma = H @ self.sigma_pred_c
        
        # Predicted measurement mean
        z_pred = np.sum(self.Wm * Z_sigma, axis=1, keepdims=True)

        if yaw_index is not None:
            s_sum = np.sum(self.Wm * np.sin(Z_sigma[yaw_idx, :]), axis=1)
            c_sum = np.sum(self.Wm * np.cos(Z_sigma[yaw_idx, :]), axis=1)
            z_pred[yaw_idx, 0] = np.arctan2(s_sum, c_sum)

        # Covariances
        dz = Z_sigma - z_pred
        dx = self.sigma_pred_c - self.x_k
        if yaw_index is not None:
            dz[yaw_idx, :] = normalize_angle_vec(dz[yaw_idx, :])
        dx[2, :] = normalize_angle_vec(dx[2, :])

        Pzz = (dz * self.Wc) @ dz.T + R
        Pxz = (dx * self.Wc) @ dz.T
        
        try:
            K = np.linalg.solve(Pzz, Pxz.T).T
            dz_final = z - z_pred
            if yaw_index is not None:
                dz_final[yaw_idx, :] = normalize_angle_vec(dz_final[yaw_idx, :])
  
            self.x_k += K @ dz_final
            self.x_k[2, 0] = angle_normalize(self.x_k[2, 0])
            self.P_k -= K @ Pzz @ K.T
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