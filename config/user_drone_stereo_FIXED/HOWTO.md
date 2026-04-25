# 18r.bag 2x 尺度 bug - 修复配置说明书

本目录是针对用户无人机数据 (18r.bag, down-facing fisheye stereo) "估计轨迹整体缩 ~0.48x"
(对齐到 GPS 时需要 s≈2.07 才能贴上) 问题的修复配置 + 代码改动.

---

## 一、根因

ZUPT (Zero-velocity Update) 在**飞行中误触发**.

具体:
- 下视鱼眼相机 (focal ≈ 398 px, 放在机身底部看地面/螺旋桨) 在**纯垂直爬升**时光流是近径向的 —
  对于中心区域 (cx, cy 附近) 的特征点, 它们朝外辐射, **平均 disparity 非常小**.
- OpenVINS 的 ZUPT 判据是 `disp_avg < zupt_max_disparity && num_features > 20`, 默认 `zupt_max_disparity = 0.5 px`.
  只要这个通过, 就**无条件接受 ZUPT** (见 `UpdaterZeroVelocity.cpp:241`).
- 垂直爬升时平均 disparity 轻松 < 0.5 px, 于是 ZUPT 反复触发, 把 VIO 估计速度**拉回 0**.
- 累计下来:
  - baseline log: 138 秒飞行中 **2966 次 ZUPT accepted** (≈ 21/sec), 接近每一帧一次
  - 高度 z 只累积到 **40 m**, 真实 GPS 高度 **73 m** (比例 0.55x)
  - xy 方向爬升段移动少, 所以偏差比 z 小; 但**全轨迹的 Umeyama scale** 在起飞 60s 内算 ≈ **2.0**,
    和用户提供的 2.07 一致

这个问题单双目都会发生, 因为 ZUPT 和视觉模式无关.

---

## 二、修复方案

两级修复 (都在本目录的 `estimator_config_zupt.yaml` 里):

### (1) YAML 调参 - 收紧 ZUPT 阈值

```yaml
zupt_max_velocity: 0.02     # 原 0.1,  只有 VIO 自己估的速度 < 0.02 m/s 才考虑 ZUPT
zupt_max_disparity: 0.1     # 原 0.5,  需要 avg 特征视差 < 0.1 px 才认为"静止"
```

这两项独立收紧就能让 ZUPT 在飞行中的误触发大幅减少 (2966 -> ~860 次), 尺度从 0.48x -> 0.91x.

### (2) 代码级 - 新增"高度闸 (altitude gate)"

在 `UpdaterZeroVelocity.{h,cpp}` 里新增参数 `zupt_max_altitude` (默认 0 = 关闭, 向后兼容).
当 `zupt_max_altitude > 0` 且 **VIO 估计的高度** `state->_imu->pos().z() > zupt_max_altitude` 时,
ZUPT 被无条件拒绝.

```yaml
zupt_max_altitude: 1.0      # [新增] 起飞离地 > 1 m 后, ZUPT 完全禁用
```

物理依据: 一旦 VIO 认为自己在空中 (> 1 m), 就不可能真正静止 — 因为 quadrotor/multirotor 必须持续
电机喷气才能悬浮, 即使"悬停"也有 ±0.1 m 量级抖动. 所以此时"disparity 小"一定是**鱼眼径向流**导致
的假象, 不是真静止.

这一改动加上 (1) 的调参 (= 目录里的 FIXED config) 可以把**full-trajectory Umeyama scale 从 0.87
拉到 1.07** (即 VIO 误差从 -13% 缩小到 +7%), 并且 RMSE 从 21.8 m 降到 14.6 m.

---

## 三、实验对比 (实测数据, 18r.bag 前 300 秒)

用事件对齐 (GPS takeoff t≈6s ≈ VIO init t≈59s, 常数 offset ≈ 53s):

| Config                              | 跑时长  | Umeyama s (全轨迹) | s (起飞 60s) | RMSE   | 发散? |
|-------------------------------------|---------|--------------------|--------------|--------|-------|
| **baseline** (原参数)               | 138 s   | 0.865              | 1.851        | 21.8 m | 是 @138s |
| **E2** (只调 YAML 阈值)             | 297 s   | 1.097              | 1.294        | 18.2 m | 否    |
| **E8** (E2 + `zupt_max_altitude=1`) | 297 s   | **1.066** ★        | **1.283**    | **14.6 m** ★ | 否 |

其中:
- "Umeyama s" 是做 3D 相似变换对齐后的 scale 因子, **理想应为 1.0**
- 越接近 1.0, VIO 轨迹的**绝对尺度**越准
- s < 1 表示 VIO 偏大 (需缩小 s 倍才贴 GPS), s > 1 表示 VIO 偏小 (需放大 s 倍)

也对比过更激进 (超紧阈值 vel=0.005, disp=0.05) 和更保守 (仅 altgate, 无紧阈值) 的变体:

| Config              | s_full | RMSE   | 发散? | 备注 |
|---------------------|--------|--------|-------|------|
| E2b (vel=.005/disp=.05)     | 1.168  | 32.7 m | 否    | 过紧, bias 校正不足, 误差反弹 |
| E7 (只 altgate=1m)         | 0.343  | 24.4 m | 是 @125s | altgate 依赖 VIO 自己估的 z, 而 z 本来就被 ZUPT 压制, 没触发 |
| E3/E4/E5/E6 (zupt_only_at_beginning=true) | 0.127~0.33 | 37~141 m | 是 | 完全关飞行中 ZUPT 导致 filter 协方差炸 |

**E7 失败的教训**: 单独加 altitude gate 不够, 因为 altitude gate 的输入是**被污染的 VIO z**.
要先用 YAML 调参 (E2) 让 VIO 的 z 有机会上升, 闸门才会真正闭合. 两者配合才是 E8 效果.

---

## 四、如何复现 / 运行

### 1. 编译

```bash
cd ~/repos/open_vins
mkdir -p ov_msckf/build && cd ov_msckf/build
cmake -DENABLE_ROS=OFF ..  # 或 ROS 模式, 两种都支持新参数
make -j$(nproc)
```

代码改动涉及 3 个文件 (见 git diff):
- `ov_msckf/src/core/VioManagerOptions.h`  — 加 `zupt_max_altitude` 解析
- `ov_msckf/src/update/UpdaterZeroVelocity.h`  — 构造函数多一个参数
- `ov_msckf/src/update/UpdaterZeroVelocity.cpp` — 加 altitude gate 早拒

### 2. 准备数据 (EuRoC 格式, 从 18r.bag 提取)

```bash
# 假设 bag 在 ~/data/18r.bag
python3 docs-cn/study/scripts/bag_to_euroc.py \
    --bag ~/data/18r.bag \
    --out ~/data/user_dataset/euroc \
    --cam0-topic /camera1/image_raw/compressed \
    --cam1-topic /camera4/image_raw/compressed \
    --imu-topic  /mavros/imu/data_raw
```

### 3. 运行 VIO

```bash
cd ~/repos/open_vins/ov_msckf/build
./run_serial_msckf_ros_free \
    --config ../../config/user_drone_stereo_FIXED/estimator_config_zupt.yaml \
    --dataset ~/data/user_dataset/euroc/mav0 \
    --stereo \
    --output ~/vio_output.tum \
    --no-display
```

预计用时 5~8 分钟, 输出 5900+ 行 TUM 格式轨迹.

### 4. 对比 GPS (可选)

用 `/tmp/final_plot.py` 或 `python3 docs-cn/study/scripts/plot_traj_vs_gps.py` 生成对比图.
关键指标:
- 最终轨迹行数 ≈ 5900 (300s × 20 Hz)
- ZUPT accepted 计数 (日志里 grep `ZUPT.*accepted`) ≈ 800-1000 (baseline 是 2966 / 138s)
- 最终 z 约 60-110 m (baseline 只到 40 m)
- Umeyama scale (全轨迹) ≈ 1.05-1.10

---

## 五、为什么不改标定

用户明确说过 "有人拿相同的标定结果跑出了正确的三角化高度", 所以不怀疑:
- IMU 内参 (kalibr 里 Ta/Tg)
- Stereo baseline
- Focal 焦距
- 畸变模型

这些都**跟标定完全无关**, 只是 ZUPT 判据对下视鱼眼不适用. 朋友那版能跑对, 大概率也是
`zupt_max_velocity` 或 `zupt_only_at_beginning` 设得更严 (或者压根没打开 try_zupt, 依赖 dynamic init
+ 良好的激励) — 都在 YAML 层面.

本 FIXED 配置的好处: **不依赖标定变动, 纯参数+轻量代码改动**, 可直接合到主线分支.

---

## 六、未来改进方向 (如有兴趣)

1. **动态 disparity 阈值**: `zupt_max_disparity = C / max(altitude, 0.5)`. 物理上最严谨:
   飞越高, 像素流越小, 阈值必须线性收紧. 当前只是"一刀切 altitude > 1m 就拒", 是粗糙近似.
2. **加速度幅值旁路**: 除了 disparity/velocity, 再看 IMU 最近 N 帧 `|a - g|` 是否一直 > 0.1 m/s².
   > 0.1 说明一定在动, 可以直接拒绝 ZUPT.
3. **把 ZUPT 的 disparity 判据从"平均"改成"高分位"**: 用 disp_avg 容易被"多数中心区域特征近静止"
   平均掉. 用 p90 能更好反映边缘区域 (disparity 大) 的真实运动.
4. **改用 VIO 更成熟的 motion detection**: Kalman innovation gating, 或看 bg/ba 的 innovation
   NIS.

这些都不是这次修复的范围. 本次只做**最小可用**改动.
