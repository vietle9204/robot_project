#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import math
import csv
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# --- Cấu hình QoS ---
qos_best_effort = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

qos_reliable = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

def euler_from_quaternion(x, y, z, w):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2 * (w * y - z * x)
    pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

class RobotMonitorNode(Node):
    def __init__(self):
        super().__init__('robot_monitor_node')
        self.sources = ['enc', 'odom', 'imu_raw', 'imu_filt', 'mag_raw', 'mag_filt', 'ekf_slam']
        self.max_points = 500  # Tăng số điểm lưu trữ để mượt hơn
        
        # Dữ liệu lưu trữ
        self.data = {s: {'t': [], 'x': [], 'y': [], 'theta': [], 'v': [], 'omega': [], 'ax': []} for s in self.sources}
        
        # --- QUAN TRỌNG: Dùng chung một mốc thời gian gốc ---
        self.global_start_time = None 
        self.last_t_raw = {s: None for s in self.sources}
        self.initial_pose = {s: None for s in self.sources}
        self.pose_integrated = {s: [0.0, 0.0, 0.0] for s in self.sources}
        self.vel_imu = {'imu_raw': 0.0, 'imu_filt': 0.0}
        
        self.lock = threading.Lock()

        # CSV Logging
        self.csv_file = open('robot_data_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['source', 'system_time', 'msg_time', 'x', 'y', 'theta', 'v', 'omega', 'ax'])

        # Subscribers
        self.create_subscription(TwistStamped, '/robot1/vel_encoder/data', self.vel_enc_cb, qos_best_effort)
        self.create_subscription(Odometry, '/odometry/data', self.odom_cb, qos_best_effort)
        self.create_subscription(Imu, '/robot1/imu/data', self.imu_raw_cb, qos_best_effort)
        self.create_subscription(Imu, '/imu/filtered', self.imu_filt_cb, qos_best_effort)
        self.create_subscription(MagneticField, '/robot1/mag/data', self.mag_raw_cb, qos_best_effort)
        self.create_subscription(MagneticField, '/mag/filtered', self.mag_filt_cb, qos_best_effort)
        self.create_subscription(PoseWithCovarianceStamped, '/ekf_slam/pose', self.ekf_slam_cb, qos_reliable)

    def process_time(self, source, header):
        # Lấy thời gian từ header của message (thời gian thực của sensor)
        msg_t = header.stamp.sec + header.stamp.nanosec * 1e-9
        
        # Gán mốc gốc chung cho toàn bộ hệ thống từ message đầu tiên nhận được
        if self.global_start_time is None:
            self.global_start_time = msg_t
        
        dt = (msg_t - self.last_t_raw[source]) if self.last_t_raw[source] else 0.0
        self.last_t_raw[source] = msg_t
        
        # Thời gian tương đối tính theo mốc gốc chung
        relative_t = msg_t - self.global_start_time
        return relative_t, dt, msg_t

    def add_sync_data(self, source, t_rel, x, y, theta, v, omega, ax, msg_t):
        system_t = time.time()
        with self.lock:
            if self.initial_pose[source] is None:
                self.initial_pose[source] = [x, y, theta]
            
            dx = x - self.initial_pose[source][0]
            dy = y - self.initial_pose[source][1]
            dtheta = normalize_angle(theta - self.initial_pose[source][2])

            d = self.data[source]
            d['t'].append(t_rel)
            d['x'].append(dx)
            d['y'].append(dy)
            d['theta'].append(dtheta)
            d['v'].append(v)
            d['omega'].append(omega)
            d['ax'].append(ax)
            
            if len(d['t']) > self.max_points:
                for key in d: d[key].pop(0)

            self.csv_writer.writerow([source, system_t, msg_t, dx, dy, dtheta, v, omega, ax])

    def ekf_slam_cb(self, msg):
        t_rel, _, msg_t = self.process_time('ekf_slam', msg.header)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        self.add_sync_data('ekf_slam', t_rel, x, y, yaw, 0.0, 0.0, 0.0, msg_t)

    def odom_cb(self, msg):
        t_rel, _, msg_t = self.process_time('odom', msg.header)
        _, _, yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                           msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.add_sync_data('odom', t_rel, msg.pose.pose.position.x, msg.pose.pose.position.y, yaw, 
                           msg.twist.twist.linear.x, msg.twist.twist.angular.z, 0.0, msg_t)

    def vel_enc_cb(self, msg):
        t_rel, dt, msg_t = self.process_time('enc', msg.header)
        v, w = msg.twist.linear.x, msg.twist.angular.z
        self.pose_integrated['enc'][2] = normalize_angle(self.pose_integrated['enc'][2] + w * dt)
        self.pose_integrated['enc'][0] += v * math.cos(self.pose_integrated['enc'][2]) * dt
        self.pose_integrated['enc'][1] += v * math.sin(self.pose_integrated['enc'][2]) * dt
        self.add_sync_data('enc', t_rel, self.pose_integrated['enc'][0], self.pose_integrated['enc'][1], 
                           self.pose_integrated['enc'][2], v, w, 0.0, msg_t)

    def process_imu(self, msg, source):
        t_rel, dt, msg_t = self.process_time(source, msg.header)
        ax, wz = msg.linear_acceleration.x, msg.angular_velocity.z
        self.vel_imu[source] += ax * dt
        self.pose_integrated[source][2] = normalize_angle(self.pose_integrated[source][2] + wz * dt)
        self.pose_integrated[source][0] += self.vel_imu[source] * math.cos(self.pose_integrated[source][2]) * dt
        self.pose_integrated[source][1] += self.vel_imu[source] * math.sin(self.pose_integrated[source][2]) * dt
        self.add_sync_data(source, t_rel, self.pose_integrated[source][0], self.pose_integrated[source][1], 
                           self.pose_integrated[source][2], self.vel_imu[source], wz, ax, msg_t)

    def imu_raw_cb(self, msg): self.process_imu(msg, 'imu_raw')
    def imu_filt_cb(self, msg): self.process_imu(msg, 'imu_filt')
    
    def process_mag(self, msg, source):
        t_rel, _, msg_t = self.process_time(source, msg.header)
        mag_yaw = math.atan2(msg.magnetic_field.y, msg.magnetic_field.x)
        self.add_sync_data(source, t_rel, 0.0, 0.0, mag_yaw, 0.0, 0.0, 0.0, msg_t)

    def mag_raw_cb(self, msg): self.process_mag(msg, 'mag_raw')
    def mag_filt_cb(self, msg): self.process_mag(msg, 'mag_filt')

    def destroy_node(self):
        self.csv_file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = RobotMonitorNode()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    fields = ['x', 'y', 'theta', 'v', 'omega', 'ax']
    titles = ['X Relative (m)', 'Y Relative (m)', 'Theta (rad)', 'Linear Vel (m/s)', 'Angular Vel (rad/s)', 'Accel X (m/s^2)']
    colors = {
        'enc': 'blue', 'odom': 'green', 'imu_raw': 'red', 
        'imu_filt': 'orange', 'mag_raw': 'purple', 
        'mag_filt': 'brown', 'ekf_slam': 'cyan'
    }
    
    # --- TẠO 6 CỬA SỔ RIÊNG BIỆT ---
    figs = []
    axs = []
    lines = {field: {} for field in fields} # Lưu trữ các đối tượng line để cập nhật nhanh

    for i, title in enumerate(titles):
        fig = plt.figure(num=i, figsize=(6, 4))
        fig.canvas.manager.set_window_title(title)
        ax = fig.add_subplot(111)
        ax.set_title(title)
        ax.grid(True, linestyle='--', alpha=0.5)
        figs.append(fig)
        axs.append(ax)

    def update_all(frame):
        with node.lock:
            max_t = 0
            for s in node.sources:
                if node.data[s]['t']:
                    max_t = max(max_t, node.data[s]['t'][-1])
            
            window_size = 10 
            t_min_limit = max(0, max_t - window_size)
            t_max_limit = max_t + 0.5
            
            for i, field in enumerate(fields):
                ax = axs[i]
                ax.clear()
                ax.set_title(titles[i])
                ax.grid(True, linestyle='--', alpha=0.5)
                ax.set_xlim(t_min_limit, t_max_limit)

                # Biến để tính toán giới hạn trục Y cho cửa sổ hiện tại
                y_min, y_max = float('inf'), float('-inf')
                has_data = False

                for s in node.sources:
                    d = node.data[s]
                    if len(d['t']) > 1:
                        if 'mag' in s and field != 'theta': continue
                        if field == 'ax' and 'imu' not in s: continue
                        if s == 'ekf_slam' and field in ['v', 'omega', 'ax']: continue
                        
                        # Vẽ dữ liệu
                        ax.plot(d['t'], d[field], color=colors[s], label=s, linewidth=1.0)

                        # --- TÍNH TOÁN TRỤC Y THÍCH ỨNG ---
                        # Chỉ lấy những điểm nằm trong khoảng thời gian đang hiển thị (window)
                        current_window_data = [
                            val for t, val in zip(d['t'], d[field]) 
                            if t >= t_min_limit
                        ]
                        
                        if current_window_data:
                            y_min = min(y_min, min(current_window_data))
                            y_max = max(y_max, max(current_window_data))
                            has_data = True

                # Thiết lập giới hạn trục Y với một khoảng đệm (padding) 10%
                if has_data:
                    padding = (y_max - y_min) * 0.1
                    if padding == 0: padding = 0.1 # Tránh lỗi nếu y_min == y_max
                    ax.set_ylim(y_min - padding, y_max + padding)
                
                if i == 0:
                    ax.legend(loc='upper left', fontsize='x-small', ncol=2)

            for fig in figs:
                fig.canvas.draw_idle()

    try:
        # FuncAnimation gắn vào figure đầu tiên nhưng sẽ điều khiển việc vẽ lại tất cả
        ani = FuncAnimation(figs[0], update_all, interval=100, cache_frame_data=False)
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()