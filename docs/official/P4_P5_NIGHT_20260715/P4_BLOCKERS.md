# P4 Blockers

没有构建、数据、权限或运行阻塞；full-flight 回放和评价均已完成。

当前阻断的是算法验收结果：

- fly1 相对冻结 reference 的 XY/yaw 略差；
- fly3 final XY、XY RMSE、vertical RMSE、speed 和 Vxy vector RMSE 均退化；
- 因此不能更新正式报告为“P4 已可用”，也不能进入 June12。

最直接的待验证假设是 fly3 的 q/p/v/bg/ba 全反馈相对 reference 的 q/p/v-only
release 造成长期退化。它目前只是由 release 差异支持的最小假设，不是已证明根因。
