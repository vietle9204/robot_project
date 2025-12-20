#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

def compute_heading(mag_x, mag_y):
    heading = math.atan2(mag_y, mag_x)
    heading_deg = math.degrees(heading)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

class ButterworthFilter2:
    def __init__(self, fc, fs):
        self.fc = fc
        self.fs = fs

        # TÍNH HỆ SỐ BỘ LỌC
        self.compute_coefficients()

        # Lưu x[n], x[n-1], x[n-2] và y[n], y[n-1], y[n-2]
        self.x_hist = [0.0, 0.0, 0.0]
        self.y_hist = [0.0, 0.0, 0.0]

    def compute_coefficients(self):
        # TÍNH K THEO fc, fs
        K = math.tan(math.pi * self.fc / self.fs)
        norm = 1.0 + math.sqrt(2.0) * K + K * K

        # FEEDFORWARD (b)
        self.b0 = (K * K) / norm
        self.b1 = (2.0 * K * K) / norm
        self.b2 = (K * K) / norm

        # FEEDBACK (a)
        self.a1 = (2.0 * (K * K - 1.0)) / norm
        self.a2 = (1.0 - math.sqrt(2.0) * K + K * K) / norm

    def reset(self):
        self.x_hist = [0.0, 0.0, 0.0]
        self.y_hist = [0.0, 0.0, 0.0]

    def filter(self, x):
        """
        Áp dụng công thức:
        y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
        """
        # Shift lịch sử
        self.x_hist[2] = self.x_hist[1]
        self.x_hist[1] = self.x_hist[0]
        self.x_hist[0] = x

        self.y_hist[2] = self.y_hist[1]
        self.y_hist[1] = self.y_hist[0]

        # Tính y[n]
        y = (self.b0 * self.x_hist[0] +
             self.b1 * self.x_hist[1] +
             self.b2 * self.x_hist[2] -
             self.a1 * self.y_hist[1] -
             self.a2 * self.y_hist[2])

        self.y_hist[0] = y
        return y



class ImuFilterNode(Node):
    def __init__(self):
        super().__init__('imu_butterworth_filter')

        # --------------- PARAMETER ----------------
        self.declare_parameter("fc", 3.0)   # Hz
        self.declare_parameter("fs", 10.0)  # Hz (raw IMU rate)

        fc = self.get_parameter("fc").value
        fs = self.get_parameter("fs").value

        self.get_logger().info(f"Butterworth filter init: fc={fc} Hz, fs={fs} Hz")

        # Tạo filter cho 6 kênh (Accel + Gyro)
        self.fx = ButterworthFilter2(fc, fs)
        self.fy = ButterworthFilter2(fc, fs)
        self.fz = ButterworthFilter2(fc, fs)

        self.gx = ButterworthFilter2(fc, fs)
        self.gy = ButterworthFilter2(fc, fs)
        self.gz = ButterworthFilter2(fc, fs)

        # ---- FILTER MAG ----
        self.mx = ButterworthFilter2(fc, fs)
        self.my = ButterworthFilter2(fc, fs)
        self.mz = ButterworthFilter2(fc, fs)

        # SUBSCRIBER & PUBLISHER
        self.sub = self.create_subscription(Imu, "/imu/data", self.cb_imu, qos)
        self.sub_mag = self.create_subscription(MagneticField, "/mag/data", self.cb_mag, qos)
        self.pub = self.create_publisher(Imu, "/imu/filtered", 10)
        self.pub_mag = self.create_publisher(MagneticField, "/mag/filtered", 10)

    def cb_imu(self, msg: Imu):
        out = Imu()
        out.header = msg.header

        # ----------- LỌC ACCEL -----------
        out.linear_acceleration.x = self.fx.filter(msg.linear_acceleration.x)
        out.linear_acceleration.y = self.fy.filter(msg.linear_acceleration.y)
        out.linear_acceleration.z = self.fz.filter(msg.linear_acceleration.z)

        # ----------- LỌC GYRO ------------
        out.angular_velocity.x = self.gx.filter(msg.angular_velocity.x)
        out.angular_velocity.y = self.gy.filter(msg.angular_velocity.y)
        out.angular_velocity.z = self.gz.filter(msg.angular_velocity.z)



        # Orientation giữ nguyên (hoặc đặt unknown)
        out.orientation = msg.orientation

        self.pub.publish(out)

        self.get_logger().info(
            f"raw: {msg.angular_velocity.x:.3f}, {msg.angular_velocity.y:.3f}, {msg.angular_velocity.z:.3f} | "
            f"filtered: {out.angular_velocity.x:.3f}, {out.angular_velocity.y:.3f}, {out.angular_velocity.z:.3f}"
        )

    def cb_mag(self, msg: MagneticField):

        out = MagneticField()
        out.header = msg.header

        # Lọc 3 trục
        mx_f = self.mx.filter(msg.magnetic_field.x)
        my_f = self.my.filter(msg.magnetic_field.y)
        mz_f = self.mz.filter(msg.magnetic_field.z)

        out.magnetic_field.x = mx_f
        out.magnetic_field.y = my_f
        out.magnetic_field.z = mz_f

        out.magnetic_field_covariance = msg.magnetic_field_covariance

        self.pub_mag.publish(out)

        # LOG
        self.get_logger().info(
            f"[MAG raw] mx={msg.magnetic_field.x:.3f}, "
            f"my={msg.magnetic_field.y:.3f}, mz={msg.magnetic_field.z:.3f} |"
            f"[MAG filt] mx={mx_f:.3f}, my={my_f:.3f}, mz={mz_f:.3f}"
        )

        self.get_logger().info(f"mag : {compute_heading(mx_f, my_f)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuFilterNode()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
