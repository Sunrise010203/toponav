# 直行：梯度通行与训练轨迹历史选择组合实验

2026-11-03（Asia/Shanghai）。显式变体 `gradient_history`，**未推广**。
它不是上一轮的等价提速：历史窗口变化会改变接受集合。

**结果：不推荐以此替换原梯度版本。** 三个开发条件两版均 0/3 完成，
组合版位移分别从 0.7707/0.2243/0.8880m 降到 0.2050/0.1051/0.8442m。
仅保留显式实验供复现，不修改默认策略。

## 动机与实现

重新录制已知 4m/seed31013 开发条件的完整连续 RGB：原默认及 reference
均位移约 0.6366m 后退出，gradient_transfer 约 0.7707m 后退出。
初发原因包括通行补丁竞争匹配、流残差或地面留出检查失败；随后停车使
九帧滑动窗口的位移基线消退。因此不能把所有失败归咎于低视差，也不能
通过延长盲走或恢复时间来消除失败。

新增 `roadvision/gradient_history_straight.py`，直接组合已有组件：

- 保持 gradient_transfer 的联合参考启动、固定宽侧向 ROI、梯度通行检查。
- 将真实 RGB 环形缓冲从 9 帧扩到 17 帧；使用既有训练轨迹选择器，
  从末尾 5/9/13/17 帧中选一个窗口。所有包先验证，时间戳不压缩；
  原 1.61s 总跨度、0.81s 关键帧间隔、0.250001s 采集间隔不变。
- 只读训练轨迹的位移、偏航、覆盖和预测残差排序，不读取最终几何接受、
  留出结果、物理位姿或场景标签。选定后只执行一次原最终几何拟合。
- 没有合格窗口时返回未知；拟合或通行失败不重试其他窗口、不回退到
  原九帧结果、不使用旧帧接受结论。
- `_update`、停车、静止确认、恢复、控制指令和所有门槛均继承原控制器。
  静止确认仍独立使用三张零速请求后的新图，至少 0.1s 间隔；
  恢复等待仍 1s，不能靠改选窗口重置。默认 `no_motion`。

任务 JSON、运行器、公共 RGB 精确回放和离线 profiler 均显式识别新变体，
禁止混入转角/路线。输出带 `promoted=false` 和带当前评估帧/时间的选窗记录。
选窗记录与策略复制后输出，外部修改不会改变控制器或全局策略。

## 复现

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.continuous_straight_run \
  --output /absolute/new/development-directory \
  --length-m 4 --seed 31013 --straight-variant gradient_history \
  --confirm-initial-clearance --motion-permission bounded_probe
```

这是开发实验，不承诺完成。省略授权或净空确认仍应零运动拒绝。
模型、传感器、标定、速度、0.8s/0.4s 门控、探测/制动/恢复预算均未放宽。
不修复近场盲区、窄障碍覆盖、受信工作预算或曝光来源缺口。

闭环比较、完整回放、独立物理审计、失败及性能结果见
[本轮报告](/home/sunrise/toponav_project/reports/straight-evidence-recovery-20261103/REPORT.md)。
S2 未通过、S6 未完成；实验通过软件契约不等于普通直行验收通过。
