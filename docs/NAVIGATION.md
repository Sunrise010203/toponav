# 统一结构导航 v2

2026-10-14。当前入口：python -m simulation.structural_navigation。

2026-10-15补充：本文描述保留的理想结构离散基线。已新增实际模拟图像识别及第五块时序会话，见 `/home/sunrise/toponav_project/docs/TEMPORAL_NAVIGATION.md`；原入口仍在stop后结束，不等于新会话行为。

## 实际调用链

相对道路结构观测 + 上轮候选/命令/执行反馈
→ structural_localization.update_candidates
→ structural_localization.decide
→ path_planning.plan_from_estimate
→ path_planning.next_action
→ StructuralEnvironment.execute。

不存在绕过公共规划块的第二套当前动作选择实现。旧navigation.py独立保留为v1节点概率基线。

## 任务、接口与执行

默认读取configs/graph.json和configs/scene.json（均为v2）。
起点/目标从graph.task读取，朝向从task.initial_heading_deg读取；缺项须用--start/--goal/--heading提供。
不把场景真实起点作为定位观测。初始朝向只支持已确认的0/90/180/270度。

观测严格只有relative_open与confidence；relative_open为front/left/right/back的true/false/null。
反馈严格只有status：succeeded/failed/unknown/sent/stopped，不含真实位置。
运动输出统一straight/left/right/uturn/stop。旧front/back/u_turn仅作输入兼容别名。

succeeded表示原子转向并走完整条通道；候选推进至地图对应节点，到达朝向由目的端向外出口+180°计算。
failed表示未移动且未转向；unknown/sent/缺失反馈保留原位与可能成功候选，并停止。
部分移动不在此模型中，不能把真实部分移动当作failed原地不动。

## 结构定位与安全

置信度低于0.6时不据观测排除候选；null只检查已知方向。
完整可信观测、唯一一致候选、执行已确认才继续。
歧义、观测不完整/不匹配、转换不可能、执行失败/未确认均停止，不擅自全图重置或读取真值修复估计。
控制器状态与最终truth/audit分离：错误先验在对称结构上可能导致错误到达判断，审计必须暴露失败而不是干预控制器。

## 几何与模拟物理

传感器和执行器读取scene.channels独立给出的source_direction/target_direction，而不是显示坐标。
定位/规划读取graph.exits；启动时校验二者一致。
geometry可省略；缺失坐标时真值日志position_m=null，缺失行驶通道长度后distance_m=null。
显示坐标可任意改变，不改变观测或动作。图中坐标不是视觉定位输入。
没有geometry也可导航，但要求绘图时缺少完整坐标会退出2。

## 常用命令

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.structural_navigation
.venv/bin/python -m simulation.structural_navigation --disable C02
.venv/bin/python -m simulation.structural_navigation --disable C06
.venv/bin/python -m simulation.structural_navigation --scenario execution_unknown
.venv/bin/python -m simulation.structural_navigation --scenario ambiguous
.venv/bin/python -m simulation.structural_navigation --start N06 --goal N01 --heading 180
.venv/bin/python -m simulation.structural_navigation --output reports/system_unification/run.json --plot reports/system_unification/run.png
```

其他故障scenario：execution_failed、execution_sent、mismatch。默认fault-step=1，为从0开始的观测/命令索引。
退出0表示控制器到达且真值审计通过；退出1表示安全停止或审计不通过；退出2表示输入错误。
每步日志分别保存truth、prior_candidates、observation、localization、decision、feedback_used和execution_feedback。
日志version=unified_topology_v2；旧structural_*.json为升级前记录，见reports/HISTORY.md。

## 验证与边界

python -m pytest -q；新增集成测试在tests/test_system_integration.py。
当前报告在reports/system_unification，不用历史403项通过替代新集成验收。
本入口仍是理想结构离散仿真，不含图像推理、ROS通信、实车执行、连续运动、概率滤波或自动恢复探索；新增图像/时序入口能力见上方补充，不以本文旧边界否定后续成果。
