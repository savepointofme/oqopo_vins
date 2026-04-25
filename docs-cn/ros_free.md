# ROS-Free 离线运行 & 可视化仪表板

> 对应官方说明: <https://docs.openvins.com/gs-installing-free.html>
>
> 本文档描述如何在**不依赖 ROS** 的前提下使用 OpenVINS MSCKF 处理离线 EuRoC/ASL 格式数据集, 并启用一个基于 OpenCV 的四宫格实时仪表板。

## 1. 背景: 为什么可以完全不走 ROS

OpenVINS 的核心滤波器 `VioManager` 本身**与 ROS 无关**, 所有入口都是传感器结构体 + 时间戳:

```cpp
// ov_msckf/src/core/VioManager.h
void feed_measurement_imu(const ov_core::ImuData &message);
void feed_measurement_camera(const ov_core::CameraData &message);
```

`ImuData` / `CameraData` 只是 `(timestamp, wm, am)` 与 `(timestamp, sensor_ids, images, masks)` 的普通 POD, 见 `ov_core/src/utils/sensor_data.h`。ROS 仅仅提供了一层"消息运输 + 参数服务器 + 可视化", 剥掉后:

- 参数由 `ov_core::YamlParser`(以 `cv::FileStorage` 为后端) 直接从 YAML 文件读取
- 数据由我们实现的 `DatasetReaderEuroc` 直接从 CSV 和 PNG 读取
- 可视化由我们新增的 `VizDashboard` (纯 OpenCV) 完成

因此把 IMU / 相机数据按**原始传感器时间戳**严格有序地喂入 VioManager, 与 ROS 版本的结果在数值上一致 (仅受初始化阶段 OpenMP 并行顺序、Ceres 线程调度等微小非确定性影响)。这与"回放 rosbag 再消息转发"本质等价 — ROS 版本内部同样是 `message.timestamp` 驱动的。

## 2. 编译

```bash
sudo apt-get install -y libeigen3-dev libboost-all-dev libceres-dev libopencv-dev

cd open_vins/ov_msckf
mkdir -p build && cd build
cmake -DENABLE_ROS=OFF ..
make -j4
```

构建完成后目录下会有:

- `libov_msckf_lib.so`  — 滤波器库 (可被第三方项目链接)
- `run_simulation` — 官方仿真入口 (ROS-free 下仅做 CSV 输出, 无 RViz)
- `run_serial_msckf_ros_free`  — 本次新增的离线回放 + 可视化入口
- `test_sim_*` — 仿真自测程序

## 3. 新增离线入口 `run_serial_msckf_ros_free`

### 3.1 数据集目录约定 (ASL / EuRoC MAV 风格)

```
mav0/
├── imu0/
│   └── data.csv           # ts_ns, wx, wy, wz, ax, ay, az
├── cam0/
│   ├── data.csv           # ts_ns, filename
│   └── data/*.png
├── cam1/                  # (可选, 立体视觉)
│   ├── data.csv
│   └── data/*.png
├── state_groundtruth_estimate0/
│   └── data.csv           # 17 列 ASL GT (可选)
└── gps.csv                # (可选) ts_ns, x, y, z   或   ts_ns, lat, lon, alt
```

### 3.2 用法

```bash
./run_serial_msckf_ros_free \
    --config ../../config/euroc_mav/estimator_config.yaml \
    --dataset /path/to/MH_01_easy/mav0 \
    --stereo                      \
    --gt    /path/to/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv \
    --gps   /path/to/real_gps.csv \
    --output traj_tum.txt         \
    --video dashboard.mp4         \
    --align-seconds 8.0
```

主要选项:

| 参数 | 含义 |
| --- | --- |
| `--config` | OpenVINS 估计器 YAML (`estimator_config.yaml`) |
| `--dataset` | `mav0/` 目录路径 |
| `--stereo` | 启用 cam1 (需配合 YAML 中 `num_cameras: 2`) |
| `--gt` | ASL 17 列 GT 文件, 用于 SE3 对齐 + ATE 计算 |
| `--gps` | 真实 GPS CSV, 自动判断 WGS84 经纬高 vs 米制 ENU |
| `--output` | TUM 轨迹输出路径 (在 VIO 原始 world frame 下未对齐) |
| `--video` | 把仪表板录制为 MP4 |
| `--align-seconds` | 初始化完成后采集多少秒数据做一次 SE3 拟合 (默认 8s) |
| `--no-display` | 不弹窗 (headless 环境必填) |
| `--dash-every` | 每 N 帧刷新一次仪表板 (默认 1) |
| `--verbose` | 逐帧打印 `feed_measurement_camera` 耗时 |

### 3.3 内部时间轴回放原理

```
┌──────────────┐           ┌──────────────────┐
│ imu0/data.csv│──┐   ┌───▶│ VioManager::feed │
└──────────────┘  │   │    │_measurement_imu  │
                  ▼   │    └──────────────────┘
            ┌──────────┴───────┐
            │ timestamp merge- │     <── 全部按 ns 精度排序, IMU 优先
            │   sort loop      │         于同时刻相机帧
            └────┬─────────────┘
┌──────────────┐ │   ┌──────────────────┐
│ cam0/data.csv│─┘   │VioManager::feed  │
│ cam1/data.csv│────▶│_measurement_camera│
└──────────────┘     └──────────────────┘
```

循环核心逻辑 (见 `run_serial_msckf_ros_free.cpp`):

```cpp
while (imu_i < imu.size() || cam_i < cam0.size()) {
  double t_imu = imu_i < imu.size() ? imu[imu_i].timestamp : INF;
  double t_cam = cam_i < cam0.size() ? cam0[cam_i].timestamp : INF;

  if (t_imu <= t_cam) {
    sys->feed_measurement_imu({t_imu, gyro, accel});
    ++imu_i;
  } else {
    ov_core::CameraData msg = build_msg(cam0[cam_i], /* maybe cam1 */);
    sys->feed_measurement_camera(msg);
    ++cam_i;
    // 查询最新状态 → 仪表板 + TUM 输出
  }
}
```

**关键保证**: 相机帧只有在其时间戳之前的所有 IMU 都已 feed 过后才会被 feed, 这与 `ROS1Visualizer::callback_inertial/callback_monocular` 内部拿到的顺序等价。

## 4. 可视化仪表板 `VizDashboard`

单窗口 1600×900, 四宫格布局:

| 宫格 | 内容 |
| --- | --- |
| 左上 | 俯视 (XY) 轨迹对比. 绿色 = GT/GPS, 橙色 = VIO (应用 SE3 后), 黄色三角 = 当前相机朝向, 红点 = SLAM 地图点, 白点 = MSCKF 临时特征 (每帧覆盖, 自然"lost 后消失") |
| 右上 | 最新相机帧 + KLT 跟踪点 (由 `VioManager::get_historical_viz_image()` 提供) |
| 左下 | 时序曲线: ATE (m) / 速度范数 (m/s) / (GT - VIO) xyz 差值 |
| 右下 | 姿态曲线: roll / pitch / yaw (deg) + 当前数值读数 |

### 4.1 对齐策略: 纯 SE3, 不含尺度

初始化完成后收集 `--align-seconds` 秒的 VIO 位置 + GT/GPS 位置, 用 **无尺度 Umeyama (SVD)** 求解 $T_{GV} = (R_{GV}, t_{GV})$:

$$R_{GV} = V S U^\top,\quad t_{GV} = \bar{p}_G - R_{GV} \bar{p}_V$$

其中 $U\Sigma V^\top = H = \sum_i (p_V^i - \bar{p}_V)(p_G^i - \bar{p}_G)^\top$, $S = \mathrm{diag}(1, 1, \det(VU^\top))$。

**不做尺度缩放**是有意设计: VIO 由 IMU 直接提供绝对尺度, 如果这里拉尺度拟合会掩盖 VIO 本身的尺度漂移 (这恰是 MSCKF 最值得观察的失效模式之一)。ATE 随时间的增长曲线就能直观反映尺度/姿态累积误差。

### 4.2 MSCKF 临时特征的"白点"对应什么

代码层面: `VioManager::get_good_features_MSCKF()` 返回 `good_features_MSCKF` 向量, 这是 `do_feature_propagate_update()` 每一帧更新滤波器后**被边缘化掉**的那部分 MSCKF 特征的世界坐标 (见 `ov_msckf/src/core/VioManager.cpp` 的 MSCKF update 段)。它们代表"当前帧下尚在滑窗内、但即将参与这一次更新然后离开"的临时点, 每一帧重新覆盖, 因此 lost 后下一帧就自动消失。

### 4.3 GPS CSV 格式自动识别

`DatasetReaderEuroc::load_gps()`:

- 若首行第 2/3/4 列满足 `|col1|<=90 ∧ |col2|<=180 ∧ |col3|<100km`, 判定为 **WGS84 经纬高**, 用 WGS84→ECEF→ENU 切平面转换, **以首个样本为原点**
- 否则视为**已是 ENU/UTM 米制** xyz

想显式控制, 直接在上游把 CSV 转成 `ts_ns,x,y,z` 米制即可。

## 5. 与 ROS 版本的等价性

**应当一致**:

- 特征跟踪结果 (`TrackKLT` 完全独立于 ROS)
- 滤波器状态 (`VioManager` / `Propagator` / `Updater*` 完全独立于 ROS)
- 输出的 TUM 轨迹

**可能有微小差异** (与 ROS 版本对比时):

- 初始化阶段若使用动态初始化, 其内部 Ceres 多线程求解器会引入数值级别的非确定性
- 若 YAML 中 `num_opencv_threads > 0`, OpenMP 在 KLT 金字塔上的并行次序不固定
- ROS 版本默认 `use_multi_threading_subs = true`, 本入口强制为 `false` 以保证严格单线程喂数据

如需完全可重复, 可在 YAML 中把 `num_opencv_threads: 0`, 并配合 `use_multi_threading_pubs: false`。

## 6. 相关文件索引

| 文件 | 作用 |
| --- | --- |
| `ov_msckf/src/ros_free/DatasetReaderEuroc.h` | IMU/cam/GT/GPS CSV 读取 |
| `ov_msckf/src/ros_free/TrajectoryAligner.h` | 纯 SE3 (无尺度) Umeyama + 时间配对 |
| `ov_msckf/src/ros_free/VizDashboard.{h,cpp}` | 四宫格 OpenCV 仪表板 |
| `ov_msckf/src/run_serial_msckf_ros_free.cpp` | 主入口, 时间轴合并回放 |
| `ov_msckf/cmake/ROS1.cmake` | 把上述源码编成 `run_serial_msckf_ros_free` 二进制 |
