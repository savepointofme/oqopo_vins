# P4 直接滑窗联合初始化重构设计

> 历史设计演化记录：本文依次追加了多轮相互替代的设计，不能把末尾方案自动视为
> 当前正确方案。每轮实际结果和当前处理决定以 `P4_EXPERIMENT_MASTER_RECORD_AND_RULES.md`
> 及 `P4_EXPERIMENT_DECISION_LEDGER.csv` 为准。

## 状态

本文是待实现、待验证的重构设计，不是已完成报告。

## 已确认的问题

当前实现同时运行两套估计：

1. 用单行 FC 状态启动一个临时 OpenVINS；
2. 在后台连续求解 8 s 的 P4 联合图；
3. P4 稳定后，用一个 yaw、水平尺度和 translation 对已经运行的
   OpenVINS 全历史执行相似变换。

该桥接方式在 fly1 有明显收益，但在 fly3 第一条直线上出现 4--7 m/s
的顺航向速度过冲。正式 GPS 时刻评价表明，这一段的 course error 接近
零，主误差是速度/尺度而不是 yaw。它不能作为正式 P4 验收结果。

更根本的问题是：视觉惯性状态在重力和 IMU 动力学建立后，尺度不再是
可以任意修改的纯 gauge。把一个已运行的 VIO 的位置、速度、clone、
landmark 和协方差整体缩放，不能保证缩放后的状态仍满足原来的 IMU
动力学。现有测试只证明重投影几何和协方差映射自洽，没有证明重置后的
惯性传播自洽。

## 参考实现得到的约束

仓库自带的 `ov_init::DynamicInitializer` 采用不同生命周期：初始化期间
只保留 IMU 和视觉轨迹，完成线性初始化和 Ceres MLE 后，直接把同一个
优化问题得到的最新 IMU 状态、clone、landmark 和 covariance 注入一个
尚未运行的 EKF。它不会先启动一个临时滤波器，再去缩放其历史。

P4 应保留 FC 因子和绝对 `G_nav` 约束，但采用同样的物理边界：批量联合
初始化和递推 EKF 只在一次原子释放处交接。

## 新生命周期

```text
FC + board IMU + KLT observations
              |
              v
       fixed 8 s time window
       advances in sensor time
              |
              v
 rebuild complete FC/IMU/visual graph
 warm-start only from overlap states
              |
              v
 compare consecutive solves on their common timestamps
 q / p / v / bg / ba covariance and overlap consistency
              |
              v
 one coherent 15-state release
 q,p,v,bg,ba + full covariance -> clean OpenVINS state
              |
              v
 start propagation / clone / MSCKF / SLAM
```

初始化等待期不运行临时 EKF 后端，不创建需要日后变形的 clone 或 SLAM
landmark。KLT 仍按固定输入 cadence 连续运行，并持续向 P4 提供同一套
FeatureDatabase 观测。

## 连续滑窗一致性

窗口一致性不能比较相邻窗口末端的原始位置和姿态，因为飞机在两个末端
之间真实运动。每次新求解完成后，应在新旧窗口的重叠时间内，把两次
求解的状态插值到相同 camera timestamp，再计算：

- `q`：相对旋转角；
- `p`：同一时刻的位置差；
- `v`：同一时刻的速度差；
- `bg`、`ba`：同一时刻的 bias 差；
- 各状态块 covariance/std；
- FC、IMU、visual factor residual 和 Jacobian support。

一致性证据按真实传感器时间累计，不使用“10 次更新”或“2 次确认”作为
时间合同。窗口内实际 FC、IMU 和视觉量测数由时间长度和数据质量自然
决定；`max_keyframes` 只允许作为求解复杂度上限，不能充当置信度证据。

## 分级置信度与原子释放

`q/p/v/bg/ba` 分别记录 `pending/stable/weak`，但 EKF 启动只发生一次。
原因是 IMU 传播需要一个彼此一致的姿态、速度和 bias 组合；先注入一部分
状态、随后再替换其余状态会再次制造动力学不连续。

生产释放至少要求：

- `q/p/v` 在重叠窗口内持续稳定，并通过当前窗口 covariance 和多因子
  support；
- `bg/ba` 至少在重叠窗口内有稳定估计和有限 covariance；
- 当前窗口和前一窗口均为 usable solution；
- 所有状态、FEJ 和 15 x 15 covariance 一次写入；
- 释放后清理初始化时间之前的 FeatureDatabase measurement，并从下一帧
  建立新的 EKF clone 历史。

如果 bias 证据不足，结果必须继续留在初始化阶段，不能把它标成已标定；
本轮首先验证完整 15 状态稳定释放，不用 GPS 未来误差调在线门限。

## 必须删除的生产依赖

- 临时 VIO 启动；
- P4 与临时 VIO 之间的水平速度尺度拟合；
- 对活动 VIO 执行 Sim(3) scale reset；
- “保留临时 bg/ba、clone、landmark”的释放语义；
- 把固定事件次数当作滑窗置信度。

诊断用的 no-anchor 隔离开关只用于确认旧桥接的故障位置，不能进入正式
配置。

## 验收标准

1. `-j12` 完成受影响 target 构建，initializer 和状态注入测试通过。
2. 运行元数据证明在释放前有多个真实向前移动的 8 s 联合窗口，每个窗口
   都重建 FC、IMU 和视觉因子。
3. 释放前 `internal_vio_initialized=false`；释放时直接写入同一个 P4 解的
   `q/p/v/bg/ba/covariance`；不存在 Sim(3) history reset。
4. fly1、fly3 在冻结的 stride 12、NASA/Lear GPS-Z、相同评价时间和
   start-heading 口径下，整体误差均优于当前源码的单行控制。
5. fly3 第一条直线不得再出现当前 P4 的 4--7 m/s 顺航向速度过冲；分段
   速度和 along error 不能以终点误差偶然抵消掩盖。
6. fly2 用于反向转弯验证，确认结果不是依赖单一转弯方向的补偿。
7. GPS course/yaw 和最终轨迹误差只用于离线评价，不进入 P4 在线决策。
8. P4 通过前不恢复 P5 active 验证。

## 2026-07-17：全状态联合图的第二轮结构修正

直接释放重构排除了旧 provisional-VIO/Sim(3) 桥接，但 fly2 正式全程仍
不如同起点的单行初始化。进一步证据表明这不是单纯的启动延迟：将单行
初始化也延迟到 P4 的释放时刻后，公共时间段 XY RMSE 为 43.58 m，而
当前直接 P4 为 55.16 m。把首状态 `bg/ba` 先验从零改到数据种子虽然使
Ceres 最终代价从 4.68 降到 3.60，正式导航却继续退化到 58.37 m；因此
“更充分地拟合当前目标函数”并不等于得到更可传播的 IMU 状态。

当前图的结构问题是：

- 每个关键帧都有独立 `bg/ba`，但 8 s 初始化窗内真实 bias 应近似常值；
- 第 `k -> k+1` 个 IMU 因子的运动残差只使用第 `k` 个 bias，第 `k+1`
  bias 主要由随机游走残差承接，最终释放 bias 因而不是完整窗口共同估计；
- 姿态、加计 bias 和 FC p/v 在一次求解中同时自由变化，模型误差可以被
  分摊成错误 tilt 与错误 `ba`，并在窗口内取得更低代价；
- 当前预积分只在求解前线性化一次，bias 发生较大修正后没有以修正后的
  bias 重建预积分。

第二轮重构采用以下合同：

1. 一个时间窗只估计一组共享 `bg/ba`；窗口内不使用每关键帧独立 bias
   吸收 FC/视觉/IMU 不一致。
2. 第一阶段固定共享 `ba` 在已标定 IMU 的零中心，只联合估计
   `q/p/v/bg/landmark`，先建立旋转、运动和视觉几何。
3. 第一阶段成功后，用其共享 `bg` 重新预积分全部 IMU 区间，并按更新后
   的 `q/p` 重新三角化 landmark。
4. 第二阶段释放共享 `ba`，联合优化 `q/p/v/bg/ba/landmark`；最终状态和
   15 x 15 covariance 必须来自这个同一问题，禁止拼接 FC p/v 或旧 bias。
5. 两阶段 solver 状态、代价、耗时和实际 invocation count 均写入运行
   证据；任一阶段失败，窗口不得进入稳定释放。

该改动的最小验收是：合成测试证明共享 bias 被所有 IMU 区间共同约束、
第二阶段确实在第一阶段结果处重新线性化；fly2 半圈/全程先超过延迟同起点
单行控制，再验证 fly1/fly3，不能以单个方向或单条飞行通过代替三飞行
验证。

## 2026-07-17：联合图历史必须进入 OpenVINS

共享 bias 两阶段求解把 fly2 公共区间 XY RMSE 从 55.16 m 降到
45.29 m，已接近同释放时刻单行控制的 43.58 m，但仍不如从 700 s 就开始
运行 OpenVINS 的原单行结果 36.52 m。分段结果表明新 P4 在约 950 s 前更好，
随后才因滤波器成熟度不足反转。根因不是“窗口太长”，而是当前释放合同
只注入末端 15 维状态：P4 已经求出的历史位姿、位姿间交叉协方差和对应
KLT 观测全部被删除，OpenVINS 仍需从零积累 clone 和视觉更新历史。

第三轮结构修正采用以下原子合同：

1. P4 covariance recovery 的顺序固定为：末端活动 IMU
   `[q,p,v,bg,ba]`，随后是按时间递增、且不包含末端重复位姿的历史
   `[q_k,p_k]` clone，最后是释放时仍被观测的 persistent landmark。
2. Ceres 一次恢复上述全部变量的联合 tangent covariance，保留
   active-clone 和 clone-clone 交叉项；禁止用独立对角先验伪造历史。
3. OpenVINS 在任何状态写入前完整检查时间单调性、clone 容量、名义值、
   covariance 维数/对称性/正定性。检查通过后一次写入 active IMU、FEJ、
   历史 clone、FEJ 和联合 covariance；失败时状态保持未初始化。
4. 若输出使用局部 `W0` 原点，active、全部 clone 和 global landmark 位置
   统一减去 P4 末端 `p_IinG`；姿态、速度、bias 和 covariance 不变。
5. 不能只转移 clone/covariance 后又让 MSCKF 使用进入 P4 因子的同一批历史
   像素。该做法在 fly2 短程把 XY RMSE 暂时降至 9.15 m，但全程又升到
   51.65 m，属于 batch posterior 与 EKF 重复使用同一视觉信息。正式实现
   必须把仍在跟踪的 P4 landmark 及其与 active/clone/landmark 的交叉
   covariance 一起注入。
6. 也不能在 posterior 注入后清空全部启动期像素。fly2 短程验证表明，这会
   同时删除从未进入 P4 图的独立 KLT 轨迹，使首个后端帧的 MSCKF accepted
   从 40 降为 0，并造成视觉冷启动。正式数据合同按 feature ID 精确消费：
   只删除实际生成 P4 重投影因子的完整轨迹；未被 P4 选择或三角化拒绝的
   KLT 历史保留，可立即约束注入的 clone 窗口。tracker 仍保持上一图像、
   keypoint 和 ID 连续性，被消费 ID 只有释放后的新像素才可再次进入后端。
7. `max_keyframes=10` 的均匀抽样会把正常 8 秒窗口约 20 个 KLT 时刻压成
   稀疏图节点，使未进入 P4 的独立轨迹在注入 clone 上没有足够连续观测。
   正式 P4 因此使用时间窗选择器产生的全部视觉帧建图；36 只保留为内存和
   求解规模的安全上限。释放时再按 OpenVINS clone 容量提取末端连续 pose
   子窗，并从完整图 Hessian 恢复该子窗的边缘联合 covariance。较早图状态
   仍作为 nuisance variables 参与边缘化，不被伪造成 EKF clone。
8. 相邻滑窗差值小不能证明共享 bias 已停止收敛。fly2 的旧释放点前 5 个
   窗口中 `ba_z` 从 -2.017 单向移动到 -1.575 m/s^2，最后一步仍变化
   0.155，而图内标准差只有 0.112；旧 gate 仍错误释放。正式判据在至少
   2 秒真实传感器时间上拟合 `bg/ba` 跨窗斜率，并投影到完整 8 秒求解窗；
   只有每个投影变化均不超过当前 posterior 的 1 sigma 才可释放。该判据
   使用因果内部状态和 covariance，不读取 GPS 误差或未来数据。

该修正先以 fly2 同释放时刻单行结果为必要下限；超过该下限后，再要求
fly2 超过原单行全程结果，并按完全相同配置验证 fly1/fly3。若完整历史注入
仍不能超过单行控制，下一步应重审联合图的量测模型和状态参数化，而不是
继续调整释放门限。

## 2026-07-17：撤销共享 bias，恢复标准动态初始化拓扑

63 个连续、因果前移的 fly2 影子窗口否定了“共享 bias 只需等待更久即可
收敛”的前提。8 秒共享 `ba_z` 随窗口内容在约 -2.02 到 -1.32 m/s² 间
往复变化；只有 3 个互不连续的窗口短暂通过 1 sigma 趋势检查，60 秒内
没有形成连续 2 秒的可释放区间。旧释放状态的 `ba_z=-1.57 m/s²`，而
同起点单行控制从零 bias 启动，后端早期约为 -0.10 m/s²。共享 bias 因此
不是更充分的物理估计，而是在吸收姿态、相关 FC 导航状态和未建模运动间
的不一致。

当前 FC 因子还有一个统计合同错误：约 20 个相邻关键帧的 FC p/v 来自同一
飞控导航滤波器，时间上高度相关，却被按独立 `2 m / 0.75 m/s` 量测重复
加入。窗口越长，相关 FC 输出被重复计权越多；共享 `ba` 随后承担使 IMU
链贴合这条平滑导航轨迹的系统误差。较长窗口因此可能得到更低图代价、
更小形式 covariance，却产生更差的开环传播。

第四轮重构不再在该目标函数上调门限，合同改为：

1. 恢复 OpenVINS `DynamicInitializer` 的标准状态拓扑：每个关键帧都有
   `q/p/v/bg/ba`，相邻状态由完整 15 维 CPI 因子连接，bias 由随机游走链
   传播；不再用一个共享 bias 拟合整段机动。
2. 首状态保留有物理含义的 bias 先验；最终 bias 必须通过整条 IMU 链和
   visual geometry 到达末端，不再从独立的共享变量旁路注入。
3. FC 只建立一个窗口级规范锚点。锚点取角速度和 FC/board 角速度残差最小
   的关键帧，并只在该时刻加入一次完整姿态、位置和速度约束，用于确定
   `G_nav` 的 yaw/translation/velocity gauge。禁止把相邻 FC 导航滤波输出
   当成独立量测逐帧重复灌入。
4. camera-IMU 外参和已接受的 FC-board 安装/时间标定保持固定；GPS course、
   GPS XY 和未来轨迹误差仍不得进入求解或释放。
5. 释放继续原子转移末端 IMU、末端连续 clones、persistent landmarks 和
   同一次求解恢复的完整联合 covariance；精确消费视觉 feature ID 的合同
   不变。
6. 先用合成测试证明末端 bias 受多段 IMU 因子和随机游走链约束、FC 因子
   数量与窗口帧数解耦，再跑 fly2 半圈和同起点全程。若仍不超过单行，
   继续审查该图的状态/量测模型，不回退到共享 bias 或门限调参。

## 2026-07-17：FC 轨迹约束改为归一化时间积分

逐关键帧 bias 和标准 15 维 CPI 链恢复后，fly2 的 63 个 8 秒影子窗口证明
“单个 FC q/p/v 锚点”同样不是正确模型。早期窗口末端速度误差最高达到
264 m/s，而第 25 个窗口的未来 4 秒速度 RMSE 又可降至 0.65 m/s；结果强烈
依赖单个锚点落在哪个图状态。该结构虽然避免了重复计算 FC 信息，却让单目尺度
和窗口内低频轨迹形状欠约束。

FC 导航序列来自同一个飞控滤波器，不能按相互独立的观测逐行累计信息；但也不能
只保留一个 p/v 时刻。正式合同改为窗内轨迹残差的归一化时间积分：

```text
J_FC,pv = (1/T) integral_window (
              ||p(t)-p_FC(t)||^2 / sigma_p^2
            + ||v(t)-v_FC(t)||^2 / sigma_v^2) dt
```

离散实现使用真实关键帧时间戳的梯形积分权重。所有权重严格为正且总和为 1；因此
增加相机帧或 FC 行只提高时间积分精度，不增加 FC 总置信度。每个关键帧都有 p/v
约束以固定尺度和轨迹形状，绝对姿态仍只使用一个低角速度、低 FC/IMU 角速度残差
时刻的完整 q gauge。运行元数据必须记录 p/v factor 数、时间权重总和和
`normalized_trapezoidal_time_integral` 模型名。

连续稳定证据采用并行时间合同：q/p/v 的相邻窗口重叠一致性必须连续通过 2 秒；
当前窗口的 bg/ba 趋势拟合也必须覆盖至少 2 秒并通过。两者在释放时取交集。禁止先让
bias 趋势自身覆盖 2 秒，再要求完整 bias gate 额外连续通过 2 秒；后者会把同一物理
稳定时间重复计算，使名义 2 秒合同实际变成约 4 秒以上。

## 2026-07-17：批优化后验到 EKF 的协方差交接

归一化 FC 时间积分模型在 fly2 短程和首个反向转弯上明显优于单行初始化，
但在随后直线段形成持续水平速度偏差。逐时数据排除了 bias 发散：首转结束后
P4 与两组单行控制的 `bg/ba` 已进入相近轨迹。问题集中在后验交接：P4 把
Ceres 在固定模型、固定标定和当前视觉 landmark 条件下恢复的联合 covariance
原样当作 EKF 的长期先验，而 OpenVINS 原生动态初始化会显式放大 orientation、
velocity、gyro bias 和 accelerometer bias 的初始化 covariance。

同状态、同释放时间的诊断对照把 P4 covariance 替换为保守对角值并删除历史：
首转后直线的局部终点误差从 52.55 m 降为 25.95 m，但首转误差从 2.12 m
恶化为 24.69 m。该结果同时证明：历史 clone/landmark 不能删除；原始 batch
covariance 也不能未经模型不确定性处理直接注入。

正式交接合同为：

1. 保留同一批优化得到的 active IMU、历史 clone、persistent landmark 和
   全部交叉 covariance，禁止退回无历史冷启动。
2. 复用当前 estimator 配置中的 OpenVINS 原生动态初始化 covariance inflation：
   orientation 10、velocity 100、gyro bias 10、accelerometer bias 100；不从
   GPS 误差或本次轨迹结果调系数。
3. 对联合 covariance 使用 congruence transform `P_handoff = D P_batch D^T`。
   `D` 只在 active IMU 的对应误差子空间使用各 inflation 的平方根，position、
   clone 和 landmark 自身尺度为 1。这样 active 与历史/landmark 的交叉项同步
   变换，并严格保持对称正定；禁止只改单个对角块而破坏相关结构。
4. 元数据记录模型名、四个 covariance inflation、原始/交接后的 15 状态标准差；
   CLI covariance override 继续只作为诊断，不属于正式路径。
5. 合成测试必须验证各 block 和 cross block 的缩放、对称性和正定性；飞行验证
   必须同时保留首转优势并降低随后直线误差。若不能同时成立，则否定该根因，
   继续审查量测模型，不能靠释放门限掩盖。

## 2026-07-17：FC 绝对 gauge 与相对轨迹形状分离

完整历史加原生 covariance handoff 后，fly2 严格公共起点半圈已经超过单行
控制；扩展到 1099.8 s 后，P4 的 XY RMSE 仍更低，且首转、第一直线、第二
转场和第二直线的局部误差都不劣于单行，但最后转场终点更差。逐时状态表明这
不是尾段新出现的 `bg_z` 发散：两种方法的 gyro bias 基本重合。P4 在第二直线
持续存在约 -0.87 m/s 的沿航迹速度偏差，来源已经在释放边界出现。

当前“绝对 p/v 残差的归一化时间平均”虽然避免 FC 行数增加总信息，但也把末端
边界因子权重降到约 0.025；以 0.75 m/s 名义 sigma 计算，末端速度的等效 sigma
约为 4.7 m/s。实际释放时，P4 末端水平速度与同刻 FC 相差约 0.61 m/s，位置相差
约 3.8 m，而加权 FC family RMS 仍只有 0.123。该模型只说明窗内平均接近 FC，
不能保证交给 EKF 的末端状态比单行边界更准确。

FC 合同改为 gauge/shape 分解：

1. 在低角速度、低 FC/board 角速度残差的同一个关键帧，加入一次绝对
   q/p/v gauge；它定义 `G_nav` 姿态、平移和共同速度偏置。
2. 其他关键帧不再重复加入绝对 FC p/v，而是约束
   `(p_k-p_anchor)`、`(v_k-v_anchor)` 与 FC 同时刻相对增量的一致性。
3. 相对残差继续使用真实时间梯形权重，并在排除 anchor 后重新归一化为总和 1；
   因此相机帧数不增加总信息。整个窗口等价于一个绝对 gauge 加一个连续相对
   轨迹形状观测，不会把同一 FC 滤波输出当成 N 个独立绝对量测。
4. 绝对和相对残差均使用既有 FC position/velocity sigma，不增加新调参；
   元数据记录 anchor 时间、模型名和末端未加权 p/v residual。
5. 验收要求同时降低释放边界 residual、保持 fly2 首转优势，并在严格公共起点
   全程的 XY RMSE、终点和分段速度上超过单行控制。

实验结论：该 gauge/shape 分解被否决并已从生产代码撤销。它使释放提前到
731.25 s，虽然图内状态仍接近 FC，但第一个正常后端视觉更新立即把水平速度
改动约 3.5 m/s，说明相对轨迹后验与转移的独立 KLT 历史不一致。严格公共起点
半圈终点误差为 52.49 m，单行控制为 41.16 m。后续工作保留此前已通过半圈的
绝对 p/v 归一化时间积分模型，不再沿该相对因子结构继续调参。
