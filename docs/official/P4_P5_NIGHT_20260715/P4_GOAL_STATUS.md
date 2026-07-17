# P4 Goal Status

Validation status: `COMPLETE`

Algorithm acceptance: `FAILED_FULL_FLIGHT`

## 已完成

- 正式有限时间窗口 lifecycle 实现与直接测试；
- fly1/fly3 matched short；
- fly1/fly3 visible full-flight 回放；
- 两飞 frame contract；
- 官方 GPS-update-time、airborne、absolute-navigation、no-post-alignment 评价；
- 与冻结 persistent/globalbaseline reference 的同窗比较；
- fly1/fly3 interactive dashboard。

## 验收判定

P4 full 不能通过：fly1 指标有得有失，fly3 相对 reference 的 final XY 增加
422.265 m，XY RMSE 增加 11.331 m，vertical RMSE 增加 5.829 m，speed RMSE
增加 1.482 m/s。该结果违反“不劣于冻结 reference”的完成条件。

所以“full-flight 验证任务”已经完成，而“P4 算法可用”没有完成。两者不得混写。

## 后续边界

若恢复 P4 算法迭代，首个单变量实验应隔离 fly3 的 bg/ba feedback；在得到证据前
不同时改窗口、noise 和 release threshold，不进入 June12。
