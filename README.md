[README.md](https://github.com/user-attachments/files/28090070/README.md)
# Mobile Robot SLAM — ROS 2 (Jazzy)

Hệ thống định vị và xây dựng bản đồ cho robot di động dạng vi sai (differential drive), chạy trên **ROS 2 Jazzy**, gồm hai thành phần chính:

- **`state_estimate_UKF2`** — Ước lượng trạng thái (odometry) bằng Unscented Kalman Filter
- **`ukf_slam`** — Đồng thời định vị và xây dựng bản đồ (SLAM) bằng UKF

---

## Kiến trúc tổng thể

```
[Encoder]──┐
[IMU]──────┤──► state_estimate_UKF2 ──► /odometry/data ──┐
[Mag]──────┘                                              │
                                                          ▼
[LaserScan] ──────────────────────────────► ukf_slam ──► /ukf_slam/pose
                                                  │
                                                  └──► /ukf_slam/map
                                                  │
                                            map_draw ──► /ukf_slam/binary_map
                                            map_to_tf ──► TF: map → odom
```

---

## Packages

| Package | Ngôn ngữ | Mô tả |
|---|---|---|
| `my_robot_kinematic` | Python | Ước lượng trạng thái (UKF odometry) |
| `ekf_slam` | Python | UKF-SLAM, vẽ bản đồ, phát TF |
| `my_robot_control` | Python | Điều hướng A*, VFH obstacle avoidance |
| `odom_to_tf` | C++ | Chuyển odometry → TF broadcast |

---

## 1. State Estimation — `state_estimate_UKF2`

**File:** `src/my_robot_kinematic/my_robot_kinematic/state_estimate_UKF2.py`

### Mô tả

Node ước lượng trạng thái robot sử dụng **Augmented UKF** với ba nguồn cảm biến: encoder bánh xe, IMU (gyroscope), và la bàn (magnetometer). Node tổng hợp dữ liệu từ ba cảm biến không đồng bộ theo cơ chế hàng đợi, nội suy thời gian, và phát ra odometry chuẩn `nav_msgs/Odometry`.

### Vector trạng thái

```
x = [x, y, θ, v, ω]ᵀ   (5 chiều)
```

| Biến | Ý nghĩa |
|---|---|
| `x, y` | Vị trí robot trong frame `odom` (m) |
| `θ` | Góc heading (rad), chuẩn hóa [-π, π] |
| `v` | Vận tốc tuyến tính (m/s) |
| `ω` | Vận tốc góc (rad/s) |

### Augmented State cho Sigma Points

Vector augment: `x_aug = [x, y, θ, v, ω, nv, nω]ᵀ` (L = 7)

Ma trận P augment:
```
P_aug = [ P_k     0  ]
        [  0    Q_k  ]
```

### Mô hình chuyển động (Prediction)

Robot vi sai, tích hợp bằng mô hình cung tròn:

```
Khi |ω| > 1e-3:
  x' = x + (v/ω)(sin(θ + ωΔt) - sin(θ))
  y' = y - (v/ω)(cos(θ + ωΔt) - cos(θ))

Khi |ω| ≤ 1e-3 (gần thẳng):
  x' = x + v·cos(θ)·Δt
  y' = y + v·sin(θ)·Δt

θ' = θ + ωΔt
```

### Mô hình đo lường (Update)

Ma trận đo lường `H` (8×5), tổng hợp từ ba cảm biến:

| Hàng | Nguồn | Đo lường |
|---|---|---|
| 0 | IMU | θ (từ tích phân gyro) |
| 1 | IMU | ω |
| 2–4 | Encoder (dead reckoning) | x, y, θ |
| 5 | Encoder | v |
| 6 | Encoder | ω |
| 7 | Magnetometer | θ (tuyệt đối) |

### Nhiễu thích nghi (Adaptive Noise)

Q (process noise) được điều chỉnh động dựa trên sai lệch giữa đo lường và dự đoán:

```
Q[v] += a1 * max(0, (enc_v - x_k[v])² - err_th)
      + a2 * max(0, (Δenc_v)²          - inn_th)

Q[ω] += a1 * max(0, (mean(enc_w, imu_w) - x_k[ω])² - err_th)
      + a2 * max(0, (Δenc_w)² + (Δimu_w)²           - inn_th)
```

### Đồng bộ thời gian

Ba buffer (deque) lưu riêng cho encoder, IMU, magnetometer. Mỗi chu kỳ timer (50 Hz), node:

1. Tìm cảm biến có timestamp nhỏ nhất (`t_min`)
2. Loại bỏ các cảm biến khác nếu lệch `> time_syns_thres` (mặc định 10 ms)
3. Nội suy giá trị tại `t_min` cho các cảm biến không đến kịp

### Luồng hoạt động

```
Timer (50 Hz)
  └─► ST_process()
        ├─► Chọn t_min từ 3 buffer
        ├─► Xử lý và nội suy dữ liệu
        ├─► UKF_prediction(dt, Q_adaptive)
        ├─► UKF_update(z, H, R)
        └─► publish_odom()
```

### Topics

| Topic | Kiểu | Hướng | Mô tả |
|---|---|---|---|
| `/robot1/vel_encoder/data` | `TwistStamped` | Sub | Vận tốc bánh xe |
| `/robot1/imu/data` | `Imu` | Sub | Dữ liệu IMU |
| `/robot1/mag/data` | `MagneticField` | Sub | La bàn |
| `/odometry/data` | `Odometry` | Pub | Odometry đầu ra |

### Cấu hình (YAML)

File: `src/my_robot_kinematic/config/ukf_st.yaml`

```yaml
state_estimate_UKF:
  ros__parameters:
    sigmapoint:
      alpha: 0.5      # Spread của sigma points
      beta:  2.0      # Prior knowledge (Gaussian = 2)
      kappa: 0.0

    state:
      P: [...]        # Ma trận hiệp phương sai ban đầu (5x5 flatten)
      Q: [0.0001, 0.0, 0.0, 0.0001]  # Process noise (2x2 flatten)

    adaptive:
      v:
        a1: 4.0       # Hệ số khuếch đại theo sai số trạng thái
        a2: 10.0      # Hệ số khuếch đại theo innovation
      R_enc_scale: 0.01
      R_imu_scale: 0.02
```

---

## 2. UKF-SLAM — `ukf_slam`

**File:** `src/ekf_slam/ekf_slam/ukf_slam.py`

### Mô tả

Node thực hiện **Feature-based UKF-SLAM** trên 2D. Robot đồng thời ước lượng vị trí của mình và xây dựng bản đồ gồm các landmark trích xuất từ dữ liệu LiDAR. Node nhận đồng bộ `(Odometry, LaserScan)` qua `ApproximateTimeSynchronizer`.

### Vector trạng thái SLAM

```
x = [xr, yr, θ, m1x, m1y, m2x, m2y, ..., mNx, mNy]ᵀ
     ─────────────  ──────────────────────────────────
      Robot pose          N landmarks (2D)
```

Kích thước tăng dần: `3 + 2N` (N = số landmark hiện có, tối đa 150).

### Luồng xử lý chính

```
sync_cb(odom, scan)
  ├─► odom_cb()  ──► predict(Δx_robot, Q_motion)
  └─► scan_cb()
        ├─► [Thread 1] extract_features_from_scan()
        ├─► [Thread 2] predict_all_measurements(sigma)
        ├─► association(features, z_hat, S)
        ├─► update(observations, Z_pred, S, Pxz)
        ├─► add_landmark() (cho feature mới)
        ├─► publish_pose()
        └─► publish_map()
```

### Bước 1: Prediction (Motion Model)

Sử dụng UKF để lan truyền toàn bộ vector trạng thái qua mô hình chuyển động:

```
Từ odometry delta:  Δx_robot = [dx, dy, dθ]  (trong robot frame)

Sigma points cập nhật:
  x' = x + cos(θ)·dx - sin(θ)·dy
  y' = y + sin(θ)·dx + cos(θ)·dy
  θ' = θ + dθ                        (normalize [-π, π])
  Landmark points: giữ nguyên
```

Process noise `Q` tỷ lệ với khoảng cách di chuyển:
```
Q = diag([Qa²·dist, Qa²·dist, Qb²·|dθ|²])   (robot frame)
→ xoay sang global frame: Q_global = R·Q·Rᵀ
```

Sau prediction, `(w_m, w_c, sigma_pred)` được lưu lại cho bước scan.

### Bước 2: Trích xuất đặc trưng (Feature Extraction)

Pipeline xử lý LiDAR scan:

```
LaserScan
  └─► segment_scan()              # Tách cụm liên tục theo khoảng cách Euclidean
        └─► extract_curvature_points()   # PCA curvature, cửa sổ trượt k=5
              └─► cluster_features()    # Phân cụm theo góc bearing
                    └─► cluster_to_feature()  # Tính (mean_r, mean_b) + covariance
```

**Tính độ cong PCA (vectorized, dùng cumsum):**

```
curvature = λ_min / (λ_min + λ_max)

Điểm góc (corner): curvature cao ≥ threshold (mặc định 0.185)
```

Mỗi feature trả về tuple `(z_mean, R_obs)`:
- `z_mean = [r, b]` — khoảng cách và bearing trung bình của cụm
- `R_obs` — covariance thực nghiệm của cụm + R cơ sở

### Bước 3: Dự đoán đo lường toàn bản đồ

Từ `sigma_pred`, dự báo đồng thời cho tất cả M landmark bằng vectorized ops:

```
dx[s,m] = mx[m] - xr[s]           # (S, M)
dy[s,m] = my[m] - yr[s]

dist[s,m]    = sqrt(dx² + dy²)
bearing[s,m] = atan2(dy, dx) - θ[s]

→ Z_pred_full  ∈ ℝ^{2M}
→ S_full       ∈ ℝ^{2M×2M}   (innovation covariance)
→ Pxz_full     ∈ ℝ^{n×2M}    (cross-covariance)
```

### Bước 4: Data Association (GNN)

Thuật toán **Global Nearest Neighbor** với kiểm tra Mahalanobis distance:

```
1. Gating thô: |Δr| < range_raw_th  AND  |Δb| < angle_raw_th
2. Mahalanobis: d² = vᵀ (S_block + R_obs)⁻¹ v  <  χ²_threshold
               (mặc định 5.99, df=2, độ tin cậy 95%)
3. One-to-one matching: sắp xếp theo d², greedy assignment
```

Feature không khớp → `new_features` → khởi tạo landmark mới.

### Bước 5: Update (Batch UKF)

Update đồng thời tất cả landmark quan sát được:

```
S = S_match + R_batch             (R_batch = kron(I_m, R_obs))
K = Pxz_match · S⁻¹              (giải bằng Cholesky)

y[r] = r_actual - r_hat
y[b] = normalize(b_actual - b_hat)

x = x + K·y
P = P - K·S·Kᵀ
```

### Bước 6: Thêm/Xoá Landmark

**Thêm landmark mới:**

```
phi = θ + b
mx = xr + r·cos(phi),  my = yr + r·sin(phi)

Gr = ∂[mx,my]/∂[xr,yr,θ]   (2×3)
Gz = ∂[mx,my]/∂[r,b]        (2×2)

P_ll    = Gr·P_rr·Grᵀ + Gz·R_obs·Gzᵀ + ε·I
P_cross = P[:,0:3]·Grᵀ
```

**Quản lý map:** Khi số landmark > `max_landmark` (150), xoá landmark có `landmark_score` thấp nhất. Score tăng +2.0 mỗi lần được quan sát.

### Topics

| Topic | Kiểu | Hướng | Mô tả |
|---|---|---|---|
| `/odometry/data` | `Odometry` | Sub | Odometry từ UKF state estimation |
| `/robot1/scan` | `LaserScan` | Sub | Dữ liệu LiDAR |
| `/ekf_slam/pose` | `PoseWithCovarianceStamped` | Pub | Vị trí robot trong frame `map` |
| `/ekf_slam/map` | `MarkerArray` | Pub | Landmark markers (RViz) |
| `/ekf_slam/binary_map` | `OccupancyGrid` | Pub | Occupancy grid (từ `map_draw`) |

### Cấu hình (YAML)

File: `src/ekf_slam/config/ukf_slam.yaml`

```yaml
ukf_slam:
  ros__parameters:
    scan_topic:  /robot1/scan
    odom_topic:  /odometry/data

    sigmapoint:
      alpha: 0.015    # Nhỏ → sigma points gần mean → ổn định cho state lớn
      beta:  2.0
      kappa: 0.0

    Q_scale:
      a: 0.05         # Hệ số nhiễu dịch chuyển
      b: 0.03         # Hệ số nhiễu góc

    R: [0.0036, 0.0049]   # [σ_r², σ_b²] measurement noise

    max_landmark: 150
    time_syns_period: 0.018   # Slop ApproximateTimeSynchronizer (18ms)

    association:
      chi2_threshold:  5.99
      range_raw_thes:  2.0    # m
      angle_raw_thes:  0.5    # rad
```

---

## Cài đặt & Build

```bash
cd ~/robot_ws/src
git clone <repo_url>

cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y
pip install transforms3d scipy --break-system-packages

colcon build --symlink-install
source install/setup.bash
```

---

## Chạy hệ thống

### Chỉ chạy state estimation

```bash
ros2 launch my_robot_kinematic state_estimate.launch.py
```

### Chạy toàn bộ SLAM

```bash
ros2 launch ekf_slam ukf_slam.launch.py
```

### Chạy kèm điều hướng

```bash
# Terminal 1
ros2 launch ekf_slam localization.launch.py

# Terminal 2
ros2 launch my_robot_control control.launch.py
```

### Tham số launch thường dùng

```bash
# Dùng với Gazebo simulation
ros2 launch ekf_slam ukf_slam.launch.py use_sim_time:=true

# Tắt publish TF odom→base_link
ros2 launch ekf_slam localization.launch.py publish_tf:=false

# Bật low-pass filter cho IMU/Mag
ros2 launch ekf_slam localization.launch.py use_LPF:=true
```

---

## Cấu trúc thư mục

```
src/
├── ekf_slam/
│   ├── ekf_slam/
│   │   ├── ukf_slam.py       # UKF-SLAM (main)
│   │   ├── ekf_slam.py       # EKF-SLAM (phiên bản cũ)
│   │   ├── map_draw.py       # OccupancyGrid từ pointcloud + pose
│   │   └── map_to_tf.py      # Publish TF map → odom
│   ├── config/ukf_slam.yaml
│   └── launch/
│       ├── ukf_slam.launch.py
│       └── localization.launch.py
│
├── my_robot_kinematic/
│   ├── my_robot_kinematic/
│   │   ├── state_estimate_UKF2.py   # UKF odometry (main)
│   │   ├── state_estimate.py        # EKF odometry
│   │   ├── imu_filter.py            # Butterworth LPF cho IMU/Mag
│   │   └── scanToCloud.py           # LaserScan → PointCloud2
│   ├── config/ukf_st.yaml
│   └── launch/state_estimate.launch.py
│
├── my_robot_control/
│   ├── my_robot_control/
│   │   ├── A_star_implementation.py # Path planning A*
│   │   ├── vfh_alg_test.py          # VFH obstacle avoidance
│   │   └── my_robot_nav.py          # Path following
│   └── launch/control.launch.py
│
└── odom_to_tf/                      # C++: Odometry → TF broadcast
```

---

## Yêu cầu hệ thống

- ROS 2 Jazzy
- Python 3.12+
- `numpy`, `scipy`, `transforms3d`
- `sensor_msgs_py`, `laser_geometry`, `message_filters`
- `opencv-python` — obstacle inflation trong A*

---

## Tham số quan trọng cần điều chỉnh

| Tham số | File | Ý nghĩa | Gợi ý |
|---|---|---|---|
| `alpha` (SLAM) | `ukf_slam.yaml` | Spread sigma points | 0.015 cho state lớn |
| `Q_scale.a/b` | `ukf_slam.yaml` | Process noise | Tăng nếu odometry drift nhiều |
| `R` | `ukf_slam.yaml` | Measurement noise LiDAR | Tăng nếu LiDAR nhiễu |
| `chi2_threshold` | `ukf_slam.yaml` | Ngưỡng data association | 5.99 = 95% confidence |
| `curvature_threshold` | `ukf_slam.yaml` | Ngưỡng trích đặc trưng | Giảm → nhiều feature hơn |
| `alpha` (odom) | `ukf_st.yaml` | Spread sigma points | 0.5 cho state nhỏ (5D) |
| `adaptive.v.a1/a2` | `ukf_st.yaml` | Adaptive process noise | Tăng nếu encoder trượt bánh |
| `time_syns_period` | 'both' | tần só chạy thuật toán | nên đặt tương đương tổng tần số các input (ms)
