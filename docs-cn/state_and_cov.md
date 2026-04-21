# State 内存布局与协方差索引

> **前置**: 已读 [`math_foundations.md`](./math_foundations.md) (特别是 §5 的 15 维 IMU 误差状态)。
> **目标**: 搞清楚 `State::_variables`, `_Cov`, `_clones_IMU`, `_features_SLAM` 这几个成员在内存里到底怎么组织, 以及你看到的 `row/col` 索引是怎么算出来的。

---

## 1. 核心数据成员

文件: <ref_file file="/home/ubuntu/repos/open_vins/ov_msckf/src/state/State.h" /> (行 `146~192`)

```cpp
class State {
  std::shared_ptr<ov_type::IMU>                                    _imu;              // 活动 IMU (15)
  std::map<double, std::shared_ptr<ov_type::PoseJPL>>              _clones_IMU;       // 滑窗克隆 (N×6)
  std::unordered_map<size_t, std::shared_ptr<ov_type::Landmark>>   _features_SLAM;    // 长期特征

  std::shared_ptr<ov_type::Vec>                                    _calib_dt_CAMtoIMU; // 时间偏移 (1)
  std::unordered_map<size_t, std::shared_ptr<ov_type::PoseJPL>>    _calib_IMUtoCAM;    // 相机外参 (6)
  std::unordered_map<size_t, std::shared_ptr<ov_type::Vec>>        _cam_intrinsics;    // 相机内参+畸变 (8)
  std::shared_ptr<ov_type::Vec>                                    _calib_imu_dw;      // 陀螺内参 (6)
  std::shared_ptr<ov_type::Vec>                                    _calib_imu_da;      // 加计内参 (6)
  std::shared_ptr<ov_type::Vec>                                    _calib_imu_tg;      // 重力敏感度 (9)
  std::shared_ptr<ov_type::JPLQuat>                                _calib_imu_GYROtoIMU; // 陀螺轴 (3)
  std::shared_ptr<ov_type::JPLQuat>                                _calib_imu_ACCtoIMU;  // 加计轴 (3)

  Eigen::MatrixXd                                                  _Cov;               // 大协方差
  std::vector<std::shared_ptr<ov_type::Type>>                      _variables;         // 均值变量表
};
```

**关键洞察**: 只有 `_Cov` 和 `_variables` 是"全局真理", 其余的 `_imu`, `_clones_IMU`, `_features_SLAM`, `_calib_*` 都是指向同一批 `Type` 对象的指针集合, 方便按语义分类访问。

---

## 2. `Type` 体系是一切的"地基"

每个可估变量都继承 `ov_type::Type`:

```
Type                              // 抽象基类, 带 _id, _size, _value, _fej
├── Vec         (dof = N)         // 向量
├── JPLQuat     (dof = 3)         // 4D 存储, 3D 误差状态
├── PoseJPL     (dof = 6)         // Quat + Vec
├── IMU         (dof = 15)        // PoseJPL + v + bg + ba
└── Landmark    (dof = 1/3)       // 特征, 取决于 representation
```

`Type::_id` = 在 `_Cov` 里的起始行索引 (即 `set_local_id()` 设定的值)。
所以 `Cov.block(T->id(), T->id(), T->size(), T->size())` 就是 `T` 自己的边缘协方差。

**验证**: <ref_snippet file="/home/ubuntu/repos/open_vins/ov_core/src/types/IMU.h" lines="64-70" /> 展示了复合 `Type` 怎么把内部 id 串起来:
```cpp
void set_local_id(int new_id) override {
    _id = new_id;
    _pose->set_local_id(new_id);                                    // q, p  → [id, id+6)
    _v->set_local_id(_pose->id() + _pose->size());                  // v     → [id+6, id+9)
    _bg->set_local_id(_v->id() + _v->size());                       // bg    → [id+9, id+12)
    _ba->set_local_id(_bg->id() + _bg->size());                     // ba    → [id+12, id+15)
}
```
所以 IMU 在 `_Cov` 的位置顺序固定是: **θ, p, v, bg, ba**。

---

## 3. `_Cov` 的行列顺序 (时间演进)

滤波器启动后按如下顺序注入变量, **每个注入会把 `_Cov` 做一次 `conservativeResize` + 填入对应块**:

```
初始化完成时 _variables / _Cov 顺序:
────────────────────────────────────────────────────────────
 0..14   IMU 误差状态 (δθ, δp, δv, δb_g, δb_a)
 15      calib_dt_CAMtoIMU          (若 do_calib_camera_timeoffset)
 16..22  calib_IMUtoCAM[0]          (每相机 6, 若 do_calib_camera_pose)
 ...     cam_intrinsics[0..N-1]     (每相机 8, 若 do_calib_camera_intrinsics)
 ...     calib_imu_dw/da/tg/...     (若 do_calib_imu_intrinsics)
────────────────────────────────────────────────────────────
每次 propagate_and_clone:
  +6      新克隆 PoseJPL (q_GtoIi, p_IiinG) ← 追加到 _clones_IMU 末端
每次 MSCKF 更新 (特征三角化通过卡方后):
  不改 Cov 结构 (MSCKF 特征不进状态, 见 updater.md)
每次 SLAM 特征初始化:
  +1 或 +3  Landmark (根据表示方式)
每次边缘化最老克隆:
  -6      从 _Cov 里抠掉最老的 6 列 6 行
```

这一切在 `StateHelper` 里实现 (搜 `augment_clone`, `marginalize`, `initialize_full`, `EKFPropagation`, `EKFUpdate`)。

---

## 4. 状态结构图 (配合 mmd 源码)

渲染的图: `diagrams/07_state_structure.png` (源码 <ref_file file="/home/ubuntu/repos/open_vins/docs-cn/diagrams/07_state_structure.mmd" />)。

一个 5 克隆 + 2 相机 + 1 SLAM 特征的典型 `_Cov` 大小估算:

```
IMU    15
+ dt   1
+ calib_IMUtoCAM[0,1]  6+6 = 12
+ intrinsics[0,1]      8+8 = 16
+ imu_intrinsics       15 (若开)
─────────────
基础 ≈ 59
+ 5 clones ×6 = 30        → 89
+ 1 SLAM feature  ×3 = 3  → 92
```

所以 EuRoC 常见配置下 `_Cov` 大概 80~150 维之间, 属于"中等稠密矩阵", 这也是 OpenVINS 保留 MSCKF 零空间投影的主要原因 — **不要让特征进状态**, 否则维数爆炸。

---

## 5. 实战: 从代码里定位某个变量的 Cov block

例: 想看"第 3 个相机克隆 (timestamp = t) 的位置协方差":

```cpp
auto clone = state->_clones_IMU[t];        // PoseJPL, size=6
int id = clone->id();                      // 起始行
auto Cov_clone = state->_Cov.block(id, id, 6, 6);

// PoseJPL 的子结构:  δθ (3) | δp (3)
auto Cov_rot = state->_Cov.block(id,     id,     3, 3);  // 朝向
auto Cov_pos = state->_Cov.block(id + 3, id + 3, 3, 3);  // 位置
auto Cov_RP  = state->_Cov.block(id,     id + 3, 3, 3);  // 旋转-位置协同项
```

> OpenVINS 没有直接暴露 `_Cov` 的公共 getter (只有 `StateHelper::get_marginal_covariance(order)` 等函数按 `Type` 列表拼接), 所以上面这段是"原理演示"。实战时请走 `StateHelper::get_marginal_covariance` 接口, 不要直接碰 `_Cov`。

---

## 6. `_clones_IMU` 用 `std::map` 的理由

因为 `map` 按 key (timestamp) 有序, 所以:
- 最老克隆 = `_clones_IMU.begin()`, 边缘化时直接 `erase` 头元素。
- 按时间顺序遍历无需自己排序。

代码体现:
```cpp
// State::margtimestep()  (State.h:66-75)
for (const auto &clone_imu : _clones_IMU) {
    if (clone_imu.first < time) time = clone_imu.first;
}
```

这也解释了为什么特征观测里用的是 `std::map<double, shared_ptr<PoseJPL>>` 而不是 vector — 特征可能在某些克隆上没观测, 随机访问比顺序查找更方便。

---

## 7. 调试技巧: 打印 Cov 结构

```cpp
for (auto &v : state->_variables) {
    std::cout << "id=" << v->id() << " size=" << v->size() << std::endl;
}
```

如果你怀疑 `_Cov` 对称性出了问题:
```cpp
double asym = (state->_Cov - state->_Cov.transpose()).norm();
assert(asym < 1e-8);
```

对于正定性:
```cpp
Eigen::LLT<Eigen::MatrixXd> llt(state->_Cov);
assert(llt.info() == Eigen::Success);  // 若 false, 有负特征值
```

---

## 8. 下一步

- 接下来 [`propagation_math.md`](./propagation_math.md) 会告诉你 IMU 积分和协方差 `Φ, Q_d` 具体怎么生成。
- 再之后 [`measurement_math.md`](./measurement_math.md) 讲"像素残差 → Jacobian → 零空间 → QR → EKF 更新"全链路。
