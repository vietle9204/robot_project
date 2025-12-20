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
#     """Normalize angle to [-pi, pi]."""
#     return (a + math.pi) % (2 * math.pi) - math.pi

# def euler_from_quaternion(qx, qy, qz, qw):
#     # normalize quaternion to avoid NaNs if not unit
#     norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#     if norm == 0:
#         return 0.0, 0.0, 0.0
#     qx /= norm; qy /= norm; qz /= norm; qw /= norm

#     # roll (x-axis rotation)
#     sinr_cosp = 2 * (qw * qx + qy * qz)
#     cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
#     roll = math.atan2(sinr_cosp, cosr_cosp)

#     # pitch (y-axis rotation)
#     sinp = 2 * (qw * qy - qz * qx)
#     if abs(sinp) >= 1:
#         pitch = math.copysign(math.pi / 2, sinp)  # clamp
#     else:
#         pitch = math.asin(sinp)

#     # yaw (z-axis rotation)
#     siny_cosp = 2 * (qw * qz + qx * qy)
#     cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
#     yaw = math.atan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

# class OdomKF(Node):
#     def __init__(self):
#         super().__init__('odom_node')

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )

#         self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
#         self.declare_parameter('odometry_topic', '/odometry/data')
#         self.declare_parameter('imu_topic', '/imu/filtered')
#         self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
#         self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value
#         self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

#         self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
#         self.create_subscription(Imu, self.imu_topic, self.imu_callback, qos)
#         self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

#         self.last_time = None

#         self.imu_last_time = None
#         # self.imu_vx = 0.0
#         # self.imu_vy = 0.0

#         # state: [x, y, theta]^T
#         self.x_k = np.zeros((3,1))

#         # state transition (identity for small-dt model; A will remain I)
#         self.A_k = np.eye(3)

#         # input in KF: will be set in KF(dt)
#         self.B_k = np.zeros((3,2))

#         # control u = [v, omega] U_k
#         self.u_k = np.zeros((2,1))

#         # measurement z (yaw from IMU)
#         self.z_k = np.zeros((3,1))

#         # measurement matrix H: measure yaw only
#         self.H_k = np.eye(3)

#         # process noise covariance (tunable)
#         self.Q_k = np.array([[0.01, 0, 0],
#                              [0, 0.01, 0],
#                              [0, 0, 0.01]])

#         # measurement noise covariance (tunable)
#         self.R_k = np.array([[0.1, 0, 0],
#                              [0, 0.1, 0],
#                              [0, 0, 0.05]])

#         # state covariance
#         self.P_k = np.array([[0.01, 0, 0],
#                              [0, 0.01, 0],
#                              [0, 0, 0.01]])

#         # Covariances for publishing odom
#         self.pose_covariance = np.diag([0.01, 0.01, 0.0, 0.0, 0.0, 0.05]).flatten().tolist()
#         self.twist_covariance = np.diag([0.05, 0.05, 0.0, 0.0, 0.0, 0.1]).flatten().tolist()


#     def imu_callback(self, msg: Imu):
#         if(self.imu_last_time is None):
#             self.imu_last_time = msg.header.stamp
#             # self.z_k = self.x_k
#             return
#         dt = ((msg.header.stamp.sec - self.imu_last_time.sec) +
#               (msg.header.stamp.nanosec - self.imu_last_time.nanosec) * 1e-9)
#         if dt <= 0.0 or dt > 1.0:
#             # ignore bad dt (too large or non-positive)
#              self.imu_last_time = msg.header.stamp
#              return
#         self.imu_last_time = msg.header.stamp

#         #  yaw
#         self.z_k[2,0] += float(msg.angular_velocity.z) * dt
#         self.z_k[2,0] = angle_normalize(float(self.z_k[2,0]))
#         yaw = self.z_k[2,0]
        
#        # Chuyển gia tốc từ body frame sang global
#         a_x_global = msg.linear_acceleration.x * math.cos(yaw) #- msg.linear_acceleration.y * math.sin(yaw)
#         a_y_global = msg.linear_acceleration.x * math.sin(yaw) #+ msg.linear_acceleration.y * math.cos(yaw)

#         # # Tích phân vận tốc trong global frame
#         # self.imu_vx += a_x_global * dt
#         # self.imu_vy += a_y_global * dt

#         # Tích phân vị trí trong global frame
#         x_new = self.z_k[0,0] + self.u_k[0, 0]*math.cos(self.x_k[2, 0])*dt + 0.5*a_x_global*dt**2
#         y_new = self.z_k[1,0] + self.u_k[0, 0]*math.sin( self.x_k[2, 0])*dt + 0.5*a_y_global*dt**2

#         self.z_k = np.array([[x_new],
#                              [y_new],
#                              [yaw]])

#     def encoder_callback(self, msg: TwistStamped):
#         # prefer using the msg.header.stamp if available for consistent timing
#         stamp = msg.header.stamp
#         if self.last_time is None:
#             self.last_time = stamp
#             # set initial yaw if imu already arrived? but keep simple
#             return

#         dt = ((stamp.sec - self.last_time.sec) +
#               (stamp.nanosec - self.last_time.nanosec) * 1e-9)
#         # guard dt
#         if dt <= 0.0 or dt > 1.0:
#             # ignore bad dt (too large or non-positive)
#             self.last_time = stamp
#             return
#         self.last_time = stamp

#         v = float(msg.twist.linear.x)
#         omega = float(msg.twist.angular.z)
#         self.u_k = np.array([[v],[omega]])

#         self.KF(dt)

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
#         self.pose_covariance = pose_cov.flatten().tolist()

#         odom = Odometry()
#         odom.header.stamp = msg.header.stamp
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = float(self.x_k[0,0])
#         odom.pose.pose.position.y = float(self.x_k[1,0])
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = math.sin(float(self.x_k[2,0]) / 2.0)
#         odom.pose.pose.orientation.w = math.cos(float(self.x_k[2,0]) / 2.0)

#         odom.twist.twist.linear.x = v
#         odom.twist.twist.angular.z = omega

#         odom.pose.covariance = self.pose_covariance
#         odom.twist.covariance = self.twist_covariance

#         self.odom_pub.publish(odom)

#     def KF(self, dt):
#         # update B matrix for discrete integration
#         theta = float(self.x_k[2,0])
#         self.B_k = np.array([[math.cos(theta)*dt, 0.0],
#                              [math.sin(theta)*dt, 0.0],
#                              [0.0, dt]])
        
#         # ==== Kalman Filter: model forecast steps ====
#         # Predict
#         # forward kinematics:  x_k_f = A x_k + B u_k 
#         x_k_f = self.A_k.dot(self.x_k) + self.B_k.dot(self.u_k)   # (3,1)
#         x_k_f[2,0] = angle_normalize(float(x_k_f[2,0]))

#         # P_f = A P A^T + Q  
#         P_k_f = self.A_k.dot(self.P_k).dot(self.A_k.T) + self.Q_k

#         # ==== Kalman Filter: data assimilation steps ====
#         # Kalman gain K = P_f H^T (H P_f H^T + R)^{-1}
#         K_k = P_k_f.dot(self.H_k.T).dot(np.linalg.inv(self.H_k.dot(P_k_f).dot(self.H_k.T) + self.R_k))

#         # # P_k update: P_k = (I - K H) P_f
#         # self.P_k = np.dot((np.eye(3) - np.dot(K_k, self.H_k)), P_k_f)
#         # self.P_k = 0.5 * (self.P_k + self.P_k.T)

#         # Update covariance using Joseph form for numerical stability: 
#         # P = (I - K H) P_f (I - K H)^T + K R K^T
#         I = np.eye(3)
#         KH = K_k.dot(self.H_k)
#         self.P_k = (I - KH).dot(P_k_f).dot((I - KH).T) + K_k.dot(self.R_k).dot(K_k.T)
#         self.P_k = 0.5 * (self.P_k + self.P_k.T)        # enforce symmetry

#         # == Measurement update: x_k = x_k_f + K_k * y_k ==
#         # Innovation (measurement residual) and normalize angle residual
#         e_k = self.z_k - self.H_k.dot(x_k_f)  # (1,1)
#         e_k[2,0] = angle_normalize(float(e_k[2,0]))
#         # Update state
#         self.x_k = x_k_f + K_k.dot(e_k)
#         self.x_k[2,0] = angle_normalize(float(self.x_k[2,0]))

#         # self.z_k = self.x_k.copy()  # for next iteration
#         # self.imu_last_time = self.last_time  # sync imu time


# def main(args=None):
#     import rclpy
#     rclpy.init(args=args)
#     node = OdomKF()
#     rclpy.spin(node)
#     node.destroy_node()
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
        self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

        self.last_time = None

        self.imu_last_time = None
        # self.imu_vx = 0.0
        # self.imu_vy = 0.0

        # state: [x, y, theta]^T
        self.x_k = np.zeros((3,1))

        # state transition (identity for small-dt model; A will remain I)
        self.A_k = np.zeros((3,3))

        # input in KF: will be set in KF(dt)
        self.G_k = np.zeros((3,2))

        # control u = [v, omega] U_k
        self.u_k = np.zeros((2,1))

        # measurement z (yaw from IMU)
        self.z_k = np.zeros((3,1))

        # measurement matrix H: 
        self.H_k = np.eye(3)

        # process noise covariance (tunable)
        self.Q_k = np.array([[0.01, 0.00],
                             [0.00, 0.015]])

        # measurement noise covariance (tunable)
        self.R_k = np.array([[0.1, 0, 0],
                             [0, 0.1, 0],
                             [0, 0, 0.05]])

        # state covariance
        self.P_k = np.array([[0.01, 0, 0],
                             [0, 0.01, 0],
                             [0, 0, 0.01]])

        # Covariances for publishing odom
        self.pose_covariance = np.diag([0.01, 0.01, 0.0, 0.0, 0.0, 0.05]).flatten().tolist()
        self.twist_covariance = np.diag([0.05, 0.05, 0.0, 0.0, 0.0, 0.1]).flatten().tolist()


    def imu_callback(self, msg: Imu):
        if(self.imu_last_time is None):
            self.imu_last_time = msg.header.stamp
            return
        dt = ((msg.header.stamp.sec - self.imu_last_time.sec) +
              (msg.header.stamp.nanosec - self.imu_last_time.nanosec) * 1e-9)
        if dt <= 0.0 or dt > 1.0:
            # ignore bad dt (too large or non-positive)
             self.imu_last_time = msg.header.stamp
             return
        self.imu_last_time = msg.header.stamp

        #  yaw
        self.z_k[2,0] += float(msg.angular_velocity.z) * dt
        self.z_k[2,0] = angle_normalize(float(self.z_k[2,0]))
        yaw = self.z_k[2,0]
        
       # Chuyển gia tốc từ body frame sang global
        a_x_global = msg.linear_acceleration.x * math.cos(yaw) #- msg.linear_acceleration.y * math.sin(yaw)
        a_y_global = msg.linear_acceleration.x * math.sin(yaw) #+ msg.linear_acceleration.y * math.cos(yaw)

        # # Tích phân vận tốc trong global frame
        # self.imu_vx += a_x_global * dt
        # self.imu_vy += a_y_global * dt

        # Tích phân vị trí trong global frame
        x_new = self.z_k[0,0] + self.u_k[0, 0]*math.cos(self.x_k[2, 0])*dt + 0.5*a_x_global*dt**2
        y_new = self.z_k[1,0] + self.u_k[0, 0]*math.sin( self.x_k[2, 0])*dt + 0.5*a_y_global*dt**2

        self.z_k = np.array([[x_new],
                             [y_new],
                             [yaw]])

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
        self.u_k = np.array([[v],[omega]])

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
        self.pose_covariance = pose_cov.flatten().tolist()

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

        odom.pose.covariance = self.pose_covariance
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)

    def KF(self, dt):
        if math.fabs(self.u_k[1,0]) <= 0.05:
            # update B matrix for discrete integration
            theta = float(self.x_k[2,0])
            self.A_k = np.array([[1.0, 0.0, -self.u_k[0,0]*math.sin(theta)*dt],
                                [0.0, 1.0,  self.u_k[0,0]*math.cos(theta)*dt],
                                [0.0, 0.0, 1.0]])
            
            self.G_k = np.array([[math.cos(theta)*dt, 0.0],
                                [math.sin(theta)*dt, 0.0],
                                [0.0, dt]])
            
            # ==== Kalman Filter: model forecast steps ====
            # Predict
            # forward kinematics:  x_k_f = A x_k + B u_k 
            x_k_f = self.x_k + self.G_k.dot(self.u_k)   # (3,1)
            x_k_f[2,0] = angle_normalize(float(x_k_f[2,0]))
        else:
            # update B matrix for discrete integration
            v = float(self.u_k[0,0])
            w = float(self.u_k[1,0])
            theta = float(self.x_k[2,0])
            theta_f = angle_normalize(float(theta + w*dt))

            # Jacobians
            self.A_k = np.array([
                [1, 0, v/w*(math.cos(theta_f) - math.cos(theta))],
                [0, 1, v/w*(math.sin(theta_f) - math.sin(theta))],
                [0, 0, 1]
            ])
            self.G_k = np.array([
                [(math.sin(theta_f)-math.sin(theta))/w, -v/(w*w)*(math.sin(theta_f)-math.sin(theta)) + v/w*math.cos(theta_f)*dt],
                [-(math.cos(theta_f)-math.cos(theta))/w, v/(w*w)*(math.cos(theta_f)-math.cos(theta)) + v/w*math.sin(theta_f)*dt],
                [0, dt]
            ])

            # ==== Kalman Filter: model forecast steps ====
            # Predict
            x_k_f = np.zeros((3,1))
            x_k_f[0,0] = self.x_k[0,0] + v/w*(math.sin(theta_f) - math.sin(theta))
            x_k_f[1,0] = self.x_k[1,0] - v/w*(math.cos(theta_f) - math.cos(theta))
            x_k_f[2,0] = theta_f

        # P_f = A P A^T + Q  
        P_k_f = self.A_k.dot(self.P_k).dot(self.A_k.T) + self.G_k.dot(self.Q_k).dot(self.G_k.T) 

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
        e_k[2,0] = angle_normalize(float(e_k[2,0]))
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
#     """Normalize angle to [-pi, pi]."""
#     return (a + math.pi) % (2 * math.pi) - math.pi

# def euler_from_quaternion(qx, qy, qz, qw):
#     # normalize quaternion to avoid NaNs if not unit
#     norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
#     if norm == 0:
#         return 0.0, 0.0, 0.0
#     qx /= norm; qy /= norm; qz /= norm; qw /= norm

#     # roll (x-axis rotation)
#     sinr_cosp = 2 * (qw * qx + qy * qz)
#     cosr_cosp = 1 - 2 * (qx*qx + qy*qy)
#     roll = math.atan2(sinr_cosp, cosr_cosp)

#     # pitch (y-axis rotation)
#     sinp = 2 * (qw * qy - qz * qx)
#     if abs(sinp) >= 1:
#         pitch = math.copysign(math.pi / 2, sinp)  # clamp
#     else:
#         pitch = math.asin(sinp)

#     # yaw (z-axis rotation)
#     siny_cosp = 2 * (qw * qz + qx * qy)
#     cosy_cosp = 1 - 2 * (qy*qy + qz*qz)
#     yaw = math.atan2(siny_cosp, cosy_cosp)

#     return roll, pitch, yaw

# class OdomKF(Node):
#     def __init__(self):
#         super().__init__('odom_node')

#         qos = QoSProfile(
#             reliability=ReliabilityPolicy.BEST_EFFORT,
#             durability=DurabilityPolicy.VOLATILE,
#             depth=10
#         )

#         self.declare_parameter('vel_encoder_topic', '/vel_encoder/data')
#         self.declare_parameter('odometry_topic', '/odometry/data')
#         self.odometry_topic = self.get_parameter('odometry_topic').get_parameter_value().string_value
#         self.vel_encoder_topic = self.get_parameter('vel_encoder_topic').get_parameter_value().string_value

#         self.create_subscription(TwistStamped, self.vel_encoder_topic, self.encoder_callback, qos)
#         self.create_subscription(Imu, '/imu/data', self.imu_callback, qos)
#         self.odom_pub = self.create_publisher(Odometry, self.odometry_topic, qos)

#         # self.x = 0.0
#         # self.y = 0.0
#         # self.theta = 0.0
#         self.last_time = None

#         self.x_k = np.array([[0.0],
#                              [0.0],
#                              [0.0]])

#         self.A_k = np.array([[1.0, 0.0, 0.0],
#                              [0.0, 1.0, 0.0],
#                              [0.0, 0.0, 1.0]])
        
#         self.B_k = np.zeros((3,2))
        
#         self.U_k = np.array([[0.0],
#                              [0.0]])
        
#         self.z_k = np.array([[0.0]])
#         self.H_k = np.array([[0.0, 0.0, 1.0]])
        
#         self.Q_k = np.array([[0.01, 0.0, 0.0],
#                              [0.0, 0.01, 0.0],
#                              [0.0, 0.0, 0.1]])
#         self.R_k = np.array([[0.01]])

#         self.P_k = np.array([[0.01, 0.0, 0.0],
#                              [0.0, 0.01, 0.0],
#                              [0.0, 0.0, 0.1]])

#         # Covariances
#         self.pose_covariance = [0.0]*36
#         self.twist_covariance = [0.0]*36

#     def imu_callback(self, msg: Imu):
#     # Chuyển quaternion sang euler
#         q = msg.orientation
#         roll, pitch, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
#         # Normalize measurement
#         yaw = angle_normalize(yaw)
#         # Lưu measurement và thời gian
#         self.z_k[0, 0] = yaw

#     def encoder_callback(self, msg: TwistStamped):
#         current_time = msg.header.stamp


#         if self.last_time is None:
#            self.last_time = current_time
#            return

#         dt = ((current_time.sec - self.last_time.sec) +
#             (current_time.nanosec - self.last_time.nanosec) * 1e-9)
#         if dt <= 0.0:
#             return
            
#         self.last_time = current_time

#         v = msg.twist.linear.x
#         omega = msg.twist.angular.z

#         self.U_k = np.array([[v],
#                              [omega]])
        
#         self.KF(dt)

#         pose_cov = np.zeros((6, 6))

#         pose_cov[0, 0] = self.P_k[0, 0]   # x-x
#         pose_cov[0, 1] = self.P_k[0, 1]   # x-y
#         pose_cov[0, 5] = self.P_k[0, 2]   # x-theta

#         pose_cov[1, 0] = self.P_k[1, 0]
#         pose_cov[1, 1] = self.P_k[1, 1]
#         pose_cov[1, 5] = self.P_k[1, 2]

#         pose_cov[5, 0] = self.P_k[2, 0]
#         pose_cov[5, 1] = self.P_k[2, 1]
#         pose_cov[5, 5] = self.P_k[2, 2]

#         self.pose_covariance = pose_cov.flatten().tolist()

#         odom = Odometry()
#         odom.header.stamp = msg.header.stamp
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = float(self.x_k[0,0])
#         odom.pose.pose.position.y = float(self.x_k[1,0])
#         odom.pose.pose.position.z = 0.0
#         odom.pose.pose.orientation.x = 0.0
#         odom.pose.pose.orientation.y = 0.0
#         odom.pose.pose.orientation.z = math.sin(self.x_k[2,0] / 2)
#         odom.pose.pose.orientation.w = math.cos(self.x_k[2,0] / 2)
#         odom.twist.twist.linear.x = v
#         odom.twist.twist.angular.z = omega

#         odom.pose.covariance = self.pose_covariance
#         odom.twist.covariance = self.twist_covariance

#         self.odom_pub.publish(odom)

#     def KF(self, dt):
#         self.B_k = np.array([[math.cos(self.x_k[2,0]) * dt, 0.0],
#                              [math.sin(self.x_k[2,0]) * dt, 0.0],
#                              [0.0, dt]])

#         x_k_f = np.dot(self.A_k, self.x_k) + np.dot(self.B_k, self.U_k)
#         P_k_f = np.dot(np.dot(self.A_k, self.P_k), self.A_k.T) + self.Q_k 

#         K_k = np.dot(np.dot(P_k_f, self.H_k.T), np.linalg.inv(np.dot(np.dot(self.H_k, P_k_f), self.H_k.T) + self.R_k))
#         self.P_k = np.dot((np.eye(3) - np.dot(K_k, self.H_k)), P_k_f)

#         self.x_k = x_k_f + np.dot(K_k, (self.z_k - np.dot(self.H_k, x_k_f)))
#         self.x_k[2, 0] = angle_normalize(float(self.x_k[2, 0]))

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdomKF()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()