#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
import numpy as np
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def angle_normalize(a):
    """Normalize angle to [-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi

def compute_heading(mag_x, mag_y):
    heading = math.atan2(mag_y, mag_x)
    heading_deg = math.degrees(heading)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

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

class OdomKF(Node):
    def __init__(self):
        super().__init__('odom_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
        self.declare_parameter('odometry_topic', '/odometry/data')
        self.declare_parameter('imu_topic', '/imu/filtered')
        self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
        self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
        self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
        self.create_subscription(MagneticField, "/mag/filtered", self.cb_mag, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

        self.last_time = None
        self.imu_last_time = None

        self.mag_yaw_base = None
        self.mag_yaw = None

        # state: [x, y, theta]^T
        self.x_k = np.zeros((3,1))

        # state transition (identity for small-dt model; A will remain I)
        self.A_k = np.eye(3)

        # input in KF: will be set in KF(dt)
        self.B_k = np.zeros((3,2))

        # control u = [v, omega]
        self.U_k = np.zeros((2,1))

        # measurement z (yaw from IMU)
        self.z_k = np.zeros((1,1))

        # measurement matrix H: measure yaw only
        self.H_k = np.array([[0.0, 0.0, 1.0]])  # shape (1,3)

        # process noise covariance (tunable)
        self.Q_k = np.array([[0.01, 0.00, 0.00],
                             [0.00, 0.01, 0.00],
                             [0.00, 0.00, 0.01]])

        # measurement noise covariance (tunable)
        self.R_k = np.array([[0.005]])

        # state covariance
        self.P_k = np.array([[0.01, 0.00, 0.00],
                             [0.00, 0.01, 0.00],
                             [0.00, 0.00, 0.10]])


    def imu_callback(self, msg: Imu):
        # q = msg.orientation
        # _,_ , yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        # yaw = angle_normalize(yaw)
        # self.z_k[0,0] = yaw
        stamp = msg.header.stamp
        if self.imu_last_time is None:
            self.imu_last_time = stamp
            self.z_k[0, 0] = self.x_k[2,0] 
            return

        dt = ((stamp.sec - self.imu_last_time.sec) +
              (stamp.nanosec - self.imu_last_time.nanosec) * 1e-9)
        # guard dt
        if dt <= 0.0 or dt > 1.0:
            # ignore bad dt (too large or non-positive)
            self.imu_last_time = stamp
            return
        self.imu_last_time = stamp

        self.z_k[0,0] += float(msg.angular_velocity.z) * dt
        self.z_k[0,0] = angle_normalize(float(self.z_k[0,0]))

    def cb_mag(self, msg):
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y
        self.mag_yaw = compute_heading(mag_x, mag_y)

        if self.mag_yaw_base is None:
            self.mag_yaw_base = self.mag_yaw - self.z_k[0, 0]

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
        if dt <= 0.0 or dt > 1.0:
            # ignore bad dt (too large or non-positive)
            self.last_time = stamp
            return
        self.last_time = stamp

        v = float(msg.twist.linear.x)
        omega = float(msg.twist.angular.z)
        self.U_k = np.array([[v],[omega]])

        self.KF(dt)

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

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = float(self.x_k[0,0])
        odom.pose.pose.position.y = float(self.x_k[1,0])
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(float(self.x_k[2,0]) / 2.0)
        odom.pose.pose.orientation.w = math.cos(float(self.x_k[2,0]) / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        odom.pose.covariance = pose_cov.flatten().tolist()

        self.odom_pub.publish(odom)

    def KF(self, dt):
        # update B matrix for discrete integration
        theta = float(self.x_k[2,0])
        self.B_k = np.array([[math.cos(theta)*dt, 0.0],
                             [math.sin(theta)*dt, 0.0],
                             [0.0, dt]])
        
        # ==== Kalman Filter: model forecast steps ====
        # Predict
        # forward kinematics:  x_k_f = A x_k + B u_k 
        x_k_f = self.A_k.dot(self.x_k) + self.B_k.dot(self.U_k)   # (3,1)
        x_k_f[2,0] = angle_normalize(float(x_k_f[2,0]))

        # P_f = A P A^T + Q  
        P_k_f = self.A_k.dot(self.P_k).dot(self.A_k.T) + self.Q_k

        # ==== Kalman Filter: data assimilation steps ====
        # Kalman gain K = P_f H^T (H P_f H^T + R)^{-1}
        K_k = P_k_f.dot(self.H_k.T).dot(np.linalg.inv(self.H_k.dot(P_k_f).dot(self.H_k.T) + self.R_k))

        # # P_k update: P_k = (I - K H) P_f
        # self.P_k = np.dot((np.eye(3) - np.dot(K_k, self.H_k)), P_k_f)
        # self.P_k = 0.5 * (self.P_k + self.P_k.T)

        # Update covariance using Joseph form for numerical stability: 
        # P = (I - K H) P_f (I - K H)^T + K R K^T
        I = np.eye(3)
        KH = K_k.dot(self.H_k)
        self.P_k = (I - KH).dot(P_k_f).dot((I - KH).T) + K_k.dot(self.R_k).dot(K_k.T)
        self.P_k = 0.5 * (self.P_k + self.P_k.T)        # enforce symmetry

        # == Measurement update: x_k = x_k_f + K_k * y_k ==
        # Innovation (measurement residual) and normalize angle residual
        e_k = self.z_k - self.H_k.dot(x_k_f)  # (1,1)
        e_k[0,0] = angle_normalize(float(e_k[0,0]))
        # Update state
        self.x_k = x_k_f + K_k.dot(e_k)
        self.x_k[2,0] = angle_normalize(float(self.x_k[2,0]))


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = OdomKF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

