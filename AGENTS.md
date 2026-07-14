# AGENTS.md

## 1. 适用范围

- 本仓库是 OpenVINS 分支，主要包含 `ov_core`、`ov_init`、`ov_msckf`、`ov_eval`，以及 `analysis/`、`tools/`、`baseline/latest/` 中的飞行回放与评估工具。
- `mems-ins-workbench/` 是独立的 React/TypeScript 可视化前端。
- 本文件只保留会影响工作正确性的规则，不充当完整仓库手册。具体命令、目录和依赖应从当前代码、脚本和 CI 中确认。

## 2. 沟通

- 用中文进行解释、计划、评审和总结；代码、变量名、文件名、CLI 参数和提交信息使用英文。
- 结论先行，随后给出关键证据、影响和未解决项。
- 不发送可选或填充式 commentary。只有工具执行、阻塞、必要状态或最终结果需要更新。
- 不盲目附和。前提错误、证据不足或方案会退化时直接指出。
- 目标清楚时直接执行；只有缺少关键选择、权限或继续会造成真实风险时才提问。
- 不用“要不要继续”“如果你愿意”等句式结尾。

## 3. 工作方法

1. 先明确目标和完成标准，区分“调查/解释”“诊断”“实现”“实验验证”。
2. 修改前阅读与任务直接相关的代码、配置、测试、文档和运行入口，确认当前真实行为，不依据旧报告猜测。
3. 用户指定参考代码、仓库或论文时，先弄清它实际解决的问题、核心机制、适用前提和验证证据。检查内容由领域决定，不套固定清单。能够说明哪些可直接复用、哪些必须适配、哪些不适用后，才能开始实现。
4. 会改变算法行为、接口、数据合同或评估口径的工作，先写简短设计说明和验收标准；简单修复不增加流程负担。
5. 行为变更前保存同条件的无改动 baseline，包括命令、配置和输出来源。候选结果退化即判失败，不用次级调参掩盖。
6. 优先复用仓库已有模式和项目自有实现。只做实现目标所需的最小改动，不增加推测性抽象、配置层或依赖。
7. 完成后运行与风险匹配的最窄验证，再做用户要求的同口径实验。失败和不确定结果必须如实保留。
8. 调研如果没有落实为设计选择、实现差异或验证方案，就不算完成。若调研否定当前草稿，停止在草稿上继续修补。

## 4. 范围与安全

- 工作区可能很脏。保留所有无关用户改动，只修改当前任务范围内的文件。
- 未经明确要求，不执行 `git reset --hard`、大范围 `git checkout`、force push、清理脚本、全仓格式化或批量重写。
- 未经明确要求，不修改生成数据、实验结果、正式报告、外部目录或外部系统状态。
- 编辑使用小而可审查的 patch。不要为了局部任务顺手重构相邻模块。
- 不读取、输出、复制或保存 secret；不硬编码凭据。
- 涉及不可信输入、上传、网络请求或命令执行时，先处理输入边界，禁止生产代码使用 `eval()` 等不安全执行方式。

## 5. 导航与估计器正确性

- 严格区分在线估计器输入和离线评价数据。truth、reference、error、GPS course、stereo pseudo-reference 等评价字段不得进入在线逻辑，除非任务明确改变系统输入合同。
- 当前 baseline 禁止融合 GNSS horizontal；只有任务明确要求时才允许改变。GPS-Z 仅按已有显式模式使用。
- 修改估计器前，必须确认名义状态与误差状态定义、坐标系、单位、时间关系、量测进入路径，以及 correction feedback 和 covariance reset 的实际实现。
- release、gate 和自适应阈值优先使用估计器内部量：covariance/std、innovation/NIS、correction、accepted update 和正式状态机。不得用未来窗口或最终误差反推在线门限。
- 常数必须有物理或统计含义。不得为了单次结果盲调 process noise、measurement noise 或 release threshold。
- 当前选择 KLT/LK 时，不得未经同口径集成实验改成 learned frontend。ONNX/XFeat/SuperPoint、RAFT/SERAFT 和其他实验路径保持隔离。
- 只有完成实际集成并在同数据、同对齐、同指标下运行后，才能声称算法改进。区分 shadow、诊断轨迹、视觉分析和真实 estimator integration。

## 6. 飞行实验与评价

- GPS 参考的正式分析使用 `analysis/full_flight_error_analysis.py` 和 `analysis/flight_eval_tool.py`，不另造口径不同的一次性指标脚本。
- 主分析在 GPS update time 采样；主 drift 使用 start-heading alignment。best-fit alignment 只能作为诊断。
- reference velocity 优先使用 FC raw `Ve,Vn,Vu`；VIO velocity 优先使用 `.bias` 中的 `vx,vy,vz`。
- 报告明确数据身份、命令、配置、时间窗、confidence start、分段定义和对齐方式。准确区分 VINS、SINS、GNSS、MEMS、INS、VIO、LK-only、pseudo-reference 和 ground truth。
- 不隐藏失败、退化或无结论实验；要求的图、指标和 provenance 不得在报告改写中消失。
- 正式 Z 评价显式使用 `--reference-mode gps_z --gps-alt-csv ...`；`stereo_pseudo_ref` 仅供调试。已对齐 camera time 的 GPS CSV 使用 `--gps-time-offset 0`。
- 不使用旧 `_corrected` fly2 或旧 fly1 offset 目录中的 `truth_asl_*`。除非专门测试 relative GPS，不使用 `--gps-alt-relative`。

## 7. OpenVINS 回放

- 当前 baseline 入口优先使用 `baseline/latest/scripts/run_baseline_fly.sh`，因为它会保存命令和配置快照。
- baseline 合同为 `--yaw-mode baseline`、`--height-mode guarded`；默认 stride 12，无 thinning 对照为 stride 1。
- 回放实验默认打开实时可视化，不主动使用 `--headless`。显示环境不可用时可使用 headless，但必须记录这一限制。
- globalbaseline、fly 对比和 adaptive-stride 验证在资源允许时并行运行；实现后不能跳过用户要求的实际回放和指标比较。

## 8. 验证

- C++ 改动至少重建直接受影响的 target；常用入口是：

  ```bash
  cmake --build build_ov_msckf --target run_serial_msckf_ros_free -j
  ```

- 按改动选择相关测试，如 `test_joseph_update`、`test_adaptive_stride`、`test_imu_filter`、`test_sim_meas`、`test_sim_repeat`。
- analysis run spec 先执行 `flight_eval_tool.py inspect-run`，确认输入和合同后再执行 `single` 或 `compare`。
- 前端改动运行 `npm run build` 和 `npm run lint`，并检查文本溢出、折叠、交互和大数据路径。
- 验证失败时先定位、修复并重跑。不得把第一个失败草稿交给用户代测。
- 长任务使用有界命令和日志；不运行无限 watcher、server 或 sleep。任务卡住时安全停止、检查日志并选择下一条最短路径。
- 只有缺少凭据/权限、必要外部数据不可用、目标确实不明确或继续会造成破坏性风险时才停止。

## 9. 工具与环境

- 搜索优先用 `rg`；文件编辑使用小范围 patch。
- C++ 遵循 `.clang-format`，但只格式化涉及文件。不要运行会全仓改写的 `run_format.sh` 或 `run_copyright.sh`。
- 工作区位于 Windows `D:\vscode_dir\open_vins`；WSL 对应路径通常为 `/mnt/d/vscode_dir/open_vins`。运行前检查脚本中的硬编码 Desktop 路径。
- PowerShell 读取中文文档和日志时显式使用 UTF-8；避免脆弱的 PowerShell/WSL 混合转义；注意 shell script 的 CRLF。
- 不假设 Docker、CUDA、WSLg、USB、串口或嵌入式目标已经配置。优先采用最小、可逆的运行时变更。
