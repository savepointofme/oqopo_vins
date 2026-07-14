# P4-R1 固定时间滑窗持续联合重估验收报告

## 结论

P4-R1 通过。当前实现已从“一次 Ceres + 冻结 candidate”切换为 shadow-only 的固定 8 秒时间滑窗持续联合重估：每当窗口终点至少前移 0.5 秒，使用当前窗内 FC、board IMU 和单目视觉证据重建并求解 Ceres 图，输出新的 q/p/v/bg/ba，并用上一窗口重叠状态 warm start。

本结论只覆盖 R1 架构语义。R1 不向 OpenVINS 释放状态，不执行分级反馈，不证明导航误差改善，也不代表 R2、P5、June12 或正式算法报告已完成。

## Provenance

- 实验源码 commit：`272fa2392a79f8f07905c4ee88feeeedabd9b748`
- 分支：`redesign/p4-sliding-window-r1-20260714`
- runner SHA256：`49fd445e3049991b73b34c26123914b6edefd3067f3cdd275a4cb1a5aa05bf83`
- initializer test SHA256：`5929883326a43287e7a3e1f6f3b5630417ea2dc9332f7d496043822b68adbe66`
- 最终运行根目录：`C:\Users\baloney\Desktop\P4_sliding_window_r1_20260714\20260714_235508_short_shadow`
- 命令入口：`bash tools/run_p4_sliding_window_r1_short.sh both`
- 配置：`baseline/latest/config/estimator_config.yaml`
- FC 输入：`readonly_audits/P4_global_baseline_fullflight_fc_board_calibration_20260714_v3/p4_inputs`
- fly1 时间段：930.0–952.0 秒；fly3 时间段：618.0–640.0 秒。
- 可视化：WSLg `DISPLAY=:0`，`--viz-fast --dash-every 5`。

运行目录中的原 `git_status.txt` 由 WSL Git 读取 Windows 创建的 linked worktree，因 CRLF/文件模式差异误报全仓修改，不能作为状态证据。`git_status_windows.txt` 是 Windows Git 的权威结果；实验 commit 时除 `build_p4_sliding_r1/` 外没有源码脏改动。

## 构建与测试

- 独立基线构建：`build_p4_sliding_r1`，成功。
- 最终受影响 target 增量构建：`cmake --build build_p4_sliding_r1 --target test_online_alignment_initializer run_serial_msckf_ros_free -j12`，成功。
- `test_online_alignment_candidate_filter`：通过。
- `test_online_alignment_initializer`：通过。
- 强化测试显式要求一个时间窗保留超过旧 36 帧上限的视觉量测，并验证多次窗口前移、warm start、无 candidate/release/close。

## 失败迭代与根因

第一次 fly1/fly3 回放中，fly1 完成 22 次求解，fly3 为 0 次。fly3 的 FC/IMU 覆盖从 626.204 秒起已经满足，唯一持续失败项为 `window_visual_duration`。

实际 camera audit 显示 fly3 KLT 调用 659/659，间隔中位数 0.03334 秒。高视差使视觉选择器约每 0.1 秒选帧，但旧代码把 `stereo_buffer_` 固定截断为 36 帧，只能保留约 3.5 秒视觉历史，无法覆盖 8 秒时间窗。这证明固定帧数仍在暗中决定窗口长度。

最终修复取消 R1 原始视觉缓存的 36 帧截断，改由固定时间 horizon 裁剪；`max_keyframes=10` 只保留为 Ceres 图计算上限，不再决定窗内量测数量。

## fly1/fly3 最终证据

| 项目 | fly1 | fly3 |
| --- | ---: | ---: |
| 真实 Ceres invocation | 22 | 23 |
| 首窗 `[begin,end]` | `[930.279210, 938.279210]` | `[618.137121, 626.137121]` |
| 末窗 `[begin,end]` | `[943.887267, 951.887267]` | `[631.376895, 639.376895]` |
| 窗口长度 | 固定 8 秒 | 固定 8 秒 |
| FC 样本数/窗 | 42–44 | 44–45 |
| IMU 样本数/窗 | 1603–1605 | 1603–1605 |
| 视觉量测帧数/窗 | 13–14 | 78–80 |
| Ceres keyframe/窗 | 10 | 10 |
| 最大 solve wall time | 0.02083 秒 | 0.01857 秒 |
| 回放最大 RSS | 193376 KiB | 194360 KiB |
| 后续窗口全部 warm start | 是 | 是 |
| 唯一 q/p/v/bg/ba 输出数 | 22 | 23 |
| 旧时间戳退出、新时间戳进入 | 是 | 是 |
| 重复窗口 fingerprint | 0 | 0 |
| `released_to_openvins` | false | false |
| `alignment_active` | true | true |

## R1 验收项

- `candidate_active_` 不阻断后续窗口求解：通过；R1 不创建 candidate。
- 新数据到达后固定时间窗持续前移并重建 Ceres：通过。
- 旧数据退出、新数据进入：通过。
- 每个窗口输出新的有限 q/p/v/bg/ba：通过。
- 后续窗口使用上一窗口解 warm start：通过。
- 固定事件次数不再决定 R1 生命周期：通过。
- `max_keyframes=10` 仅作为图规模上限：通过；fly3 每窗视觉量测 78–80 帧，优化节点仍为 10。
- 内存和单次求解开销有界：在两段 22 秒回放中通过；数据量由 8 秒窗口限制，最大 RSS 约 194 MiB，单次求解均低于 0.021 秒。
- R1 无正式状态反馈或 release：通过。

自动验收文件：

- `fly1/r1_acceptance.json`
- `fly3/r1_acceptance.json`
- `fly1/online_alignment_sliding_windows.csv`
- `fly3/online_alignment_sliding_windows.csv`
- `fly1/online_alignment_metadata.json`
- `fly3/online_alignment_metadata.json`

## 停止边界

按 R1 合同，本轮在持续滑窗 shadow 语义通过后停止。下一阶段 R2 才能设计 q/p/v/bg/ba 的按组置信度、反馈后新时间窗确认、正式导航 release，以及未可信状态继续估计的生命周期。
