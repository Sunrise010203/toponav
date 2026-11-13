# 普通直走：固定宽侧向取点实验

日期：2026-11-01。本轮承接联合参考选点，不重新开启S6/ROS，也不更换17类模型。

## 为什么继续检查侧向取点

已暴露4m/seed31013中心启动轨迹，在tick33因通行补丁匹配不唯一停车、tick47因地面跨视图支持不足停车，tick55又被侧墙几何区间门槛阻断。不能把所有问题都归因于视差不足，也不能绕过这些拒绝。

只读公共RGB回放显示：tick55原中央侧向区域左右各有75/74条通过描述子过滤的轨迹，航向半区间约4.54°，超过4°门槛。在同一九张真实图、同一地面估计上，用事先固定的更宽侧向区域，得到123/127条轨迹，半区间约2.29°，能满足**原4°门槛**。tick47地面不可用时扩大侧向区域仍然拒绝。这个离线定位结果不等于闭环直走成功。

诊断、冻结条件、逐帧数据和闭环结果：
[/home/sunrise/toponav_project/reports/straight-followup-20261101-02](/home/sunrise/toponav_project/reports/straight-followup-20261101-02)。

## 实现边界

- 显式`wide_reference`变体继承`reference`的联合启动检查，将左右相机固定检测区域从`x32..96, y8..64`改为`x12..116, y8..64`。前相机区域、地面检测区域、角点质量/间距/数量预算均不变。
- 区域在图像处理前固定选择，训练/留出划分及描述子选择不变；一次最终拟合失败就保持失败，**没有中央区域回退或寻找通过模型**。
- 默认`baseline`和既有`reference`仍用中央区域。新增控制器钩子默认仍调用原生产者；授权、初始净空、九帧历史、速度/转向、停车、连续三次重新确认及恢复预算均继承原方法。
- 公共任务和逐帧回放显式绑定变体。实验不能混入转角/路线控制，`promoted=false`。
- **更宽不代表更安全或总能通过**：更可能纳入端墙、转角和非共面表面，须继续通过原时序、每视图留出残差、正深度及条件区间检查。平行墙、静态场景和标定仍只是先验。
- 不修复车旁近场盲区；远处九个补丁仍不能替代完整扫掠覆盖。没有新增盲走或主动恢复。

## 使用

输出目录必须全新；省略授权或净空确认仍拒绝运动：

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.continuous_straight_run \
  --output /home/sunrise/toponav_project/reports/my-wide-reference-NEW \
  --straight-variant wide_reference --length-m 6 --seed 313 \
  --confirm-initial-clearance --motion-permission bounded_probe
```

这是未推广的开发实验，不承诺给定案例完成导航。最终结果与当前/历史测试分项见本轮`REPORT.md`。原S2未通过、S6未完成状态不因本轮代码实现改变。
