# 最终结论

## [P4 SLIDING-WINDOW SEMANTICS AUDIT]

**Classification:** `B`，并带有候选活动期 `C` 的特征。

**Root cause:** 时间窗口只控制 raw buffer 保留和首次 Ceres 初始化；候选一旦生成，`try_initialize()` 直接进入 `validate_candidate()`，后续只有固定 candidate 的递归 FC p/v 更新和固定 history/post-feedback confirmation，没有新的时间滑窗联合重估；释放后 `alignment_window_closed_` 永久关闭输入。

**Actual data flow:** raw FC/IMU/selected visual buffers 滑动 → 从 `{3,5,8,12}` 中选一个初始窗口 → 视觉最多 10 个 Ceres keyframes → 一次 Ceres q/p/v/bg/ba/landmark solve → 固定 candidate filter → 因果 IMU propagation + FC p/v Kalman update → group feedback/gate → release 或 reject retry。

**Where fixed counts enter:** `max_keyframes=10` 限制初始 Ceres 状态节点；旧 v6 accepted runtime 的 candidate recursive update/support count 实际为 10；post-feedback confirmation 另有 `required_post_feedback_stable_updates=2`。10 不是“滑窗长度自动推导出的量测次数”。

**Optimizer re-solve after candidate:** `NO` while candidate is active. Only a rejected candidate can return to collection and create a new solve. After accepted release: `NO`, permanently closed.

**Unsupported bg/ba later re-estimation:** `NO` after practical navigation release. Before release, bg/ba can change indirectly through FC p/v cross-covariance and may receive one gated feedback, but they do not enter a new Ceres window. If retained as prior, they do not get a later post-release opportunity.

**Staged release evidence/counter:** `YES, but only inside one frozen candidate`. `navigationReady()` requires q/p/v; `fullAlignmentReady()` adds bg/ba. v6 fly1 released q,p,v,bg and retained ba; v6 fly3 released q,p,v,bg,ba. Evidence includes group support/feedback/stable arrays and `candidate_closed_loop_update_count=10`. This proves staged group release, not persistent sliding-window re-estimation.

**Post-feedback future data:** `PARTIAL`. Future FC/IMU rows are consumed by the candidate filter; future visual snapshots are consumed for diagnostics. `candidate_closed_loop_visual_update_count=0`, and the confirmation is not a separate full time window followed by a new joint solve. No future data is used after release because the window is closed.

**Runtime confirmation:** fly1 v6: one 8 s solve, 10 selected keyframes, 10 candidate FC updates, release at `940.178965807`. fly3 v6: first 3 s candidate rejected, second 3 s solve released, both attempts have 10 selected keyframes; successful candidate has 10 recursive FC updates. These are sufficient to classify the existing accepted runtime as B/C, and do not justify claiming A.

**Design mismatch:** the requested architecture was “time window continuously advances, full joint estimate can be recomputed, each state can be released independently, and unreleased states keep receiving later-window opportunities.” The implementation is “one joint initialization, one frozen candidate recursive filter, fixed confirmation, then close.”

**Files produced:** `AUDIT_REPORT.md`, `CALL_GRAPH.md`, `STATE_MACHINE.md`, `WINDOW_DATAFLOW.md`, `FIXED_COUNT_INVENTORY.csv`, `RUNTIME_TRACE_FLY1.csv`, `RUNTIME_TRACE_FLY3.csv`, `CONCLUSION.md`.

**One minimal architecture direction, not implemented:** keep a candidate alive across causal time windows and make each eligible new window create a new joint solve (or an explicitly equivalent bounded incremental smoother) before release; use the actual selected window duration—not `max_keyframes`, raw event count, or `options_.window_duration_s` overwritten to the maximum—as the temporal evidence contract; after each group feedback retain future data and re-evaluate unreleased groups in later windows; close only when the release contract explicitly permits closure.

**Unambiguous conclusion:** 当前实现不是用户要求的 A 类“基于时间滑窗持续联合重估”。它是 B 类“一次窗口联合初始化 + 固定候选递归验证”，候选期原始 buffer 虽然滑动，但没有触发联合求解，因此同时表现出 C 类特征。P4 的核心语义尚未实现，当前不应进入参数优化、性能优化、P5 或 June12。
