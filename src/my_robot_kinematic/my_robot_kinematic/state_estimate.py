#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

def angle_normalize(a):
    """Normalize angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def euler_from_quaternion(qx, qy, qz, qw):
    # normalize quaternion to avoid NaNs if not unit
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm == 0:
        return 0.0, 0.0, 0.0
    qx /= norm; qy /= norm; qz /= norm; qw /= norm

    # roll (x-axis rotation)
    sinr_cosp = 2 * (qw * qx + qy * qz)
    cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2 * (qw * qy - qz * qx)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # clamp
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class state_estimate(Node):
    def __init__(self):
        super().__init__('odom_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
        self.declare_parameter('odometry_topic', '/odometry/data')
        self.declare_parameter('imu_topic', '/imu/data')
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
        # self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, qos)
        # self.create_subscription(MagneticField, "/mag/filtered", self.cb_mag, qos)
        # self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, qos)
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

        # Timer 50 Hz
        self.timer_period = 0.02 #seconds
        self.timer = self.create_timer(self.timer_period, self.EKF_prediction)

        self.last_time = None

        self.imu_last_time = None
        self.imu_theta = 0.0

        self.mag_last_time = None
        self.mag_yaw0 = 0.0

        # state: [x, y, theta]^T
        self.x_k = np.zeros((5,1))

        # state transition (identity for small-dt model; A will remain I)
        self.A_k = np.zeros((5,5))

        # # input in KF: will be set in KF(dt)
        # self.G_k = np.zeros((3,2))

        # # control u = [v, omega]
        # self.u_k = np.zeros((2,1))

        # process noise covariance 
        self.Q_k = np.diag([0.01, 0.01, 0.01, 0.922, 2.07])

        # state covariance
        self.P_k = np.diag([0.01, 0.01, 0.01, 0.9, 0.9])

        # # measurement z (yaw from IMU)
        # self.z_k = np.zeros((4,1))

        # # measurement matrix H: measure yaw only
        # self.H_k =  np.zeros((4,9))

        # # measurement noise covariance 
        # self.R_k = np.diag([0.01, 0.01, 0.01, 0.01])

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        yaw = angle_normalize(yaw)

        self.x_k[0,0] = msg.pose.pose.position.x
        self.x_k[1,0] = msg.pose.pose.position.y
        self.x_k[2,0] = yaw

        self.get_logger().info(f"Initial pose set: x={self.x_k[0,0]:.2f}, y={self.x_k[1,0]:.2f}, yaw={self.x_k[2,0]:.2f}")

        self.P_k = np.diag([0.01, 0.01, 0.01, 0.9, 0.9])

        self.publish_odom(msg.header.stamp)


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

        z = np.array([[self.imu_theta],
                      [angular_vel_yaw]])
        
        H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0]])

        R = np.array([[0.01, 0.00],
                      [0.00, 0.003]])
  
        self.EKF_update(z, H, R, 0)

        self.publish_odom(msg.header.stamp)

    def cb_mag(self, msg: MagneticField):
        stamp = msg.header.stamp
        if self.mag_last_time is None:
            # -------- Magnetometer --------
            mx = -msg.magnetic_field.x
            my = -msg.magnetic_field.y

            yaw_mag = math.atan2(-my, mx)
            yaw_mag = angle_normalize(yaw_mag)

            # Lưu yaw ban đầu
            self.mag_yaw0 = yaw_mag

            self.mag_last_time = stamp
            return

        dt = ((stamp.sec - self.mag_last_time.sec) +
              (stamp.nanosec - self.mag_last_time.nanosec) * 1e-9)
        # guard dt
        if dt <= 0.0:
            # ignore bad dt (too large or non-positive)
            self.mag_last_time = stamp
            return
        self.mag_last_time = stamp

        #================
        if self.x_k[3,0] >= 0.05 or self.x_k[4,0] >= 0.05:
            return
        #================

        # -------- Magnetometer --------
        mx = msg.magnetic_field.x
        my = msg.magnetic_field.y

        yaw_mag = math.atan2(-my, mx)
        yaw_mag = angle_normalize(yaw_mag - self.mag_yaw0)

        z = np.array([[float(yaw_mag)]]) 
        
        H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0]])

        R = np.array([[0.5]])
  
        self.EKF_update(z, H, R, 0)

        self.publish_odom(msg.header.stamp)


    def encoder_callback(self, msg: TwistStamped):
        # prefer using the msg.header.stamp if available for consistent timing
        stamp = msg.header.stamp
        if self.last_time is None:
            self.last_time = stamp
            # set initial yaw if imu already arrived? but keep simple
            return

        dt = ((stamp.sec - self.last_time.sec) +
              (stamp.nanosec - self.last_time.nanosec) * 1e-9)
        # guard dt
        if dt <= 0.0:
            # ignore bad dt (too large or non-positive)
            self.last_time = stamp
            return
        self.last_time = stamp
        
        z = np.array([[float(msg.twist.linear.x)],
                      [float(msg.twist.angular.z)]]) 
        
        H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0],
                      [0.0, 0.0, 0.0, 0.0, 1.0]])

        R = np.array([[0.0001, 0.00],
                      [0.00, 0.0002]])
  
        self.EKF_update(z, H, R, None)

        self.publish_odom(msg.header.stamp)


    # def amcl_cb(self, msg: PoseWithCovarianceStamped):
    #     q = msg.pose.pose.orientation
    #     yaw = 2.0 * math.atan2(q.z, q.w)
    #     yaw = angle_normalize(yaw)

    #     z = np.array([[msg.pose.pose.position.x],
    #                   [msg.pose.pose.position.y],
    #                   [yaw]])
        
    #     self.get_logger().info(f"AMCL pose: x={z[0,0]:.2f}, y={z[1,0]:.2f}, yaw={z[2,0]:.2f}")
        
    #     H = np.array([[1.0, 0.0, 0.0, 0.0, 0.0],
    #                   [0.0, 1.0, 0.0, 0.0, 0.0],
    #                   [0.0, 0.0, 1.0, 0.0, 0.0]])

    
    #     R = np.array([[msg.pose.covariance[0], msg.pose.covariance[1], 0.00],
    #                   [msg.pose.covariance[6], msg.pose.covariance[7], 0.00],
    #                   [0.00, 0.00, msg.pose.covariance[35]]])
  
    #     self.EKF_update(z, H, R, 2)

    #     self.publish_odom(msg.header.stamp)


    def EKF_prediction(self):
        dt = self.timer_period
        #================
        v = float(self.x_k[3, 0])
        if math.fabs(v)  < 0.0015: v = 0.0
        w = float(self.x_k[4, 0])
        if math.fabs(w)  < 0.0015: w = 0.0
        #================

        theta = float(self.x_k[2,0])
        theta_f = angle_normalize(float(theta + w*dt))

        if math.fabs(w) <= 0.05:
            # Jacobians
            self.A_k = np.array([[1.0, 0.0, -v*math.sin(theta)*dt, math.cos(theta)*dt, 0.0],
                                [0.0, 1.0, v*math.cos(theta)*dt, math.sin(theta)*dt, 0.0],
                                [0.0, 0.0, 1.0, 0.0, dt],
                                [0.0, 0.0, 0.0, 1.0, 0.0],
                                [0.0, 0.0, 0.0, 0.0, 1.0]])
             
            # ==== Kalman Filter: model forecast steps ====
            # Predict
            # forward kinematics:  x_k_f = f(x,k) 
            self.x_k[0,0] += v*math.cos(theta)*dt
            self.x_k[1,0] += v*math.sin(theta)*dt
            self.x_k[2,0] += w*dt
            self.x_k[2,0] = angle_normalize(float(self.x_k[2,0]))
        else:
            # Jacobians
            self.A_k = np.array([
                [1.0, 0.0, v/w*(math.cos(theta_f) - math.cos(theta)), (math.sin(theta_f)-math.sin(theta))/w, -v/(w*w)*(math.sin(theta_f)-math.sin(theta)) + v/w*math.cos(theta_f)*dt],
                [0.0, 1.0, v/w*(math.sin(theta_f) - math.sin(theta)), -(math.cos(theta_f)-math.cos(theta))/w, v/(w*w)*(math.cos(theta_f)-math.cos(theta)) + v/w*math.sin(theta_f)*dt],
                [0.0, 0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 1.0]
            ])

            # ==== Kalman Filter: model forecast steps ====
            # Predict
            self.x_k[0,0] = self.x_k[0,0] + v/w*(math.sin(theta_f) - math.sin(theta))
            self.x_k[1,0] = self.x_k[1,0] - v/w*(math.cos(theta_f) - math.cos(theta))
            self.x_k[2,0] =  theta_f

        # P_f = A P A^T + Q  
        self.P_k = self.A_k.dot(self.P_k).dot(self.A_k.T) + self.Q_k
        
        

    def EKF_update(self, z, H, R, yaw_indext):
        # ==== Kalman Filter: data assimilation steps ====
        # Kalman gain K = P_f H^T (H P_f H^T + R)^{-1}
        K_k = self.P_k.dot(H.T).dot(np.linalg.inv(H.dot(self.P_k).dot(H.T) + R))

        # Update covariance using Joseph form for numerical stability: 
        # P = (I - K H) P_f (I - K H)^T + K R K^T
        I = np.eye(5)
        KH = K_k.dot(H)
        self.P_k = (I - KH).dot(self.P_k).dot((I - KH).T) + K_k.dot(R).dot(K_k.T)
        self.P_k = 0.5 * (self.P_k + self.P_k.T)        # enforce symmetry

        # == Measurement update: x_k = x_k_f + K_k * y_k ==
        e_k = z - H.dot(self.x_k)
        if yaw_indext is not None:
            e_k[yaw_indext,0] = angle_normalize(float(e_k[yaw_indext,0]))
        # Update state
        self.x_k = self.x_k + K_k.dot(e_k)
        self.x_k[2,0] = angle_normalize(float(self.x_k[2,0]))

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
    import rclpy
    rclpy.init(args=args)
    node = state_estimate()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()