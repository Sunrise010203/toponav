# 原17类模型接入 Gazebo 导航闭环

这是新增的 **`vision` 无路标实验模式**，不是原 `route` 的 ArUco/黄线辅助模式。
保持冻结模型、原预处理和 **0.6** 阈值，使用真实 Gazebo Harmonic/DART/Ogre2 四向 RGB；没有重训练。

**2026-10-29代码修复**：物理出口签名与通行权限已分层。`map_exits()`用于结构匹配，
不会因`enabled=false`或临时禁行删除物理开口；运动预测改用`traversable_exits()`，
公共规划及执行继续拒绝禁行边。真实图像若仍与物理签名矛盾，仍保持停车。
旧C02失败记录不修改；本轮仅有回归和旧图只读诊断，未重新完成Gazebo绕行，
不能据此更改原有失败结论。见[第二批修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STRUCTURE_FIXES.md)。

## 打开窗口

```bash
bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh --scenario vision
```

窗口就绪后点击 **开始导航**。可切换拓扑图、小车第一视角、四向相机；下方显示原模型类别/得分/是否接纳、上次节点定位、规划动作及当前阶段。
无上下辅助相机。停止按钮立即进入停车清理；完成后保留窗口，可点击“重置 / 再运行”。

默认任务已经实测跑通：**N01 → N02 → N03 → N04 → N06**，左右转后目标停车。
该固定标定场景是开发联调，不是未见场景泛化评测。详细实测和失败记录见
[本次报告](/home/sunrise/toponav_project/reports/gazebo-model-20261020/REPORT.md)。

## 实际闭环

1. 四个同步原生 128×128 图像按原顺序拼成 256×256 RGB，保留真实仿真采集时间。
2. `FrozenModelPerception` 调用原 `Predictor` 加载的 MobileNetV3-Small 权重与原 `prepare` / `to_observation`。仿真时间单独校验，不伪造 UNIX 采集时刻。
3. 停车后三帧模型结构证据，进入原 `structural_localization.update_candidates` 和 `decide`，由原公共规划器确定出口。
4. 新增受限 RGB 地面/墙沿几何控制器，负责通道居中、原地转向及停靠；不读取仿真坐标、里程计或节点坐标。
5. 转向结束也必须模型确认新朝向结构；行驶中允许模型拒识，但必须有新鲜 RGB 几何。离开节点由模型直道证据或连续三帧双侧封闭墙面佐证，**墙面证据不伪装成模型类别**。
6. 下个节点的模型结构匹配和实际 RGB 停靠几何同时满足，停车稳定三帧，再用另外三帧模型证据更新定位/规划。终点同样确认后保持零速。

缺失视图、旧帧、几何不可见、超时会停车/中止；路口模型证据不足保持零速，超过4仿真秒中止。不回退到 ArUco，也不靠仿真时间/位置报到站。
各次识别概率、状态转换、指令与原图均保存，真值位姿仅写审计和显示，不进入控制器。

## 显式标定与限制

- 原六节点地图及 **0.8m** 道路不变；增加 **0.8m** 顶棚以匹配训练的高度比例。
- 四个水平相机离地 **0.8/3m**，分别向自身观察方向偏置 **0.18m**，避免车身遮挡；与旧预览相机标定不同。
- 底层几何依赖已知蓝灰地面、灰墙、90°视场、水平相机和道路宽度，不是通用视觉驾驶网络。模型只预测相对出口，不预测节点ID。
- 冻结模型既有 unknown 误接受缺陷没有消失。闭环新增守卫不能作为可靠拒识/实车安全保证。
- 封路后的公共可通行出口不一定等于从节点处看见的物理结构：阻挡在路段中点，不应把可见出口硬改成关闭或伪造观察。绕行是否可执行以实测为准，不能套用标记模式的通过结论。
- 接触传感器覆盖不完整；“未观察到墙接触”不等于无碰撞认证。
- 不改变 S2 未通过、S6 未完成等历史验收状态；不是 ROS2 或实车闭环验收。

## 无窗口运行与只读重放

输出目录必须是新的，避免覆盖证据：

```bash
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --scenario vision --output /home/sunrise/toponav_project/reports/my-model-run

cd /home/sunrise/toponav_project
source /home/sunrise/toponav_project/tools/gazebo_env.sh
.venv/bin/python -m simulation.gazebo_replay \
  /home/sunrise/toponav_project/reports/my-model-run --scenario vision
```

重放只读取公共 RGB、原时间戳及公共拓扑，重新运行真实冻结模型；逐项比较概率、定位、规划与控制决策。
仅不比较非确定性的 `model.inference_ms` 耗时诊断。模型/代码/公共拓扑/像素哈希失配拒绝重放，不更新历史冻结清单来强行通过。

核心文件：
[模型闭环](/home/sunrise/toponav_project/roadvision/model_route.py)、
[RGB几何](/home/sunrise/toponav_project/roadvision/visual_corridor_geometry.py)、
[场景标定](/home/sunrise/toponav_project/simulation/gazebo_vision_fixture.py)。
