# Gazebo 多场景与拓扑图

**全流程更新**：默认入口现为新增 `route` 标记/黄线辅助六节点导航，打开窗口后需点击“开始导航”。
支持真实行驶、视觉节点确认、转弯和目标停车；不是原无标记模型验收。
详见 [全流程说明](/home/sunrise/toponav_project/docs/GAZEBO_FULL_FLOW.md)。
以下“仅预览”描述专指保留的原 `topology` 模式及原局部控制器范围。

2026-10-20。此前的 Gazebo 入口仅验证单个 L 转弯；现在增加场景中心、T 型路口、
直走/阻挡检查，以及项目六节点地图的物理场景和规划展示。

## 一键打开

```bash
bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh
```

需要本机桌面和现有 PyQt5（本工作区已具备）。不需要 sudo，不关闭其他 Gazebo 或桌面窗口。
默认打开六节点标记辅助全流程场景（点击“开始导航”后行驶）：

- **Gazebo Sim 窗口**：真实物理道路场景，右侧原生 ImageDisplay 面板显示拓扑图、计划路径、车辆审计位置。
- **场景中心窗口**：切换场景，选择规划目标、封闭通道，查看更大的拓扑图与运行状态。

先点“停止 / 切换场景”，等记录保存后，再更换场景和点击“打开 Gazebo 场景”。
局部控制场景需自行勾选“已确认初始净空”；六节点预览不驾驶，不要求勾选。
2026-10-29修正：`straight/blocked` 另需单独勾选“授权直走受限探测”。默认仍是
`no_motion`，确认净空本身不授权运动；切换场景会清除探测勾选。新增参数传递不会放宽原视觉守卫。
直接关闭 Gazebo 窗口也会结束该次运行，场景中心继续保留。
关闭场景中心会先停止它启动的运行，完成清理后退出。

## 场景与已知边界

| 选择项 / CLI 名称 | 当前行为 |
|---|---|
| 六节点全流程 / `route` | **新增标记/黄线辅助基线**，完整路线执行；详见全流程说明 |
| 六节点地图 / `topology` | 加载原项目地图、物理通道、任务规划、封路重规划；**车保持零速，不自动跑全图** |
| L 型左转 / `left` | 未修改的 CornerController，已完成并停车 |
| L 型右转 / `right` | 同上，右转 |
| T 型左转 / `t-left` | 未修改的 CornerController，使用既有 T 型模式，已完成并停车 |
| T 型右转 / `t-right` | 同上，右转 |
| 直走廊 / `straight` | 默认 `no_motion` 拒绝运动；显式 `bounded_probe` 后仍需通过当前直走启动守卫。无纹理地面的旧证据不足结果不是本次新实测 |
| 阻挡走廊 / `blocked` | 同样需要独立授权。有真实横向挡墙，但授权传递修正**不等于验证行驶中发现障碍并停车** |

“可以打开场景”和“导航能力已通过”不同。拒绝例也保留真实结果，不调阈值强行通过。
六节点模式没有使用真值追踪路线、时间脚本驱动或瞬移车辆来冒充自主导航。

## 拓扑图从哪里来

六节点模式读取原始文件：

- `/home/sunrise/toponav_project/configs/graph.json`：节点、通道、任务、允许方向。
- `/home/sunrise/toponav_project/configs/scene.json`：独立物理通道和展示几何。

两者先通过既有 `validate_pair` 校验，物理地面由通道矩形的并集生成，墙只建在并集外边界，
不会在正常连接的路口中央加墙。通道宽度使用原文件的 **0.8m**，没有为了能开过去而偷偷放宽。
当前仅支持这些正交直线通道；非正交边会明确拒绝，不猜测几何。

路径直接调用原 `simulation.dijkstra.plan`，v2 按**最少跳数**规划，几何坐标不参与规划。

- 默认：`N01 → N02 → N03 → N04 → N06`。
- 封闭 `C02`：改走 `N01 → N02 → N05 → N04 → N06`；Gazebo 中也生成对应真实挡墙。
- 封闭 `C06`：目标 `N06` 不可达，不画伪造路线。
- 可以把目标改为其他非起点节点；原配置文件不会被改写。

图中蓝线是**计划**路径，不是“已走过”的轨迹。
红箭头取自原生位姿审计，仅用于展示，**不是视觉定位输出**；拓扑图像不输入驾驶控制器。
局部 L/T 场景的 S/J/G/B 是该测试夹具的示意拓扑，不冒充六节点任务或已认证的节点到达。

## 命令行直接打开指定场景

```bash
# T 型路口左转，真实 RGB 控制
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --gui --scenario t-left --confirm-initial-clearance \
  --output "/home/sunrise/toponav_project/reports/gazebo-t-left-$(date +%s)"

# 六节点物理地图 + C02 挡墙 + 绕行规划；仅预览、不驾驶
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --gui --scenario topology --disable C02 --goal N06 \
  --output "/home/sunrise/toponav_project/reports/gazebo-map-$(date +%s)"

# 直走受限探测：显式声明两项前提，视觉证据不足时仍会拒绝
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --scenario straight --confirm-initial-clearance --motion-permission bounded_probe \
  --output "/home/sunrise/toponav_project/reports/gazebo-straight-$(date +%s)"
```

不带 `--gui` 就是离屏有限步检查。旧的 `--direction left/right` 入口仍兼容。
图形运行结束后窗口保留，关闭窗口或点场景中心停止按钮才写最终 `result.json`。

每次独立目录增加这些输出：

- `public/topology.json`、`public/scene.json`、`public/planned_route.json`：该次配置与实际规划。
- `public/topology.png`：展示图；`audit/display.json`：单向观察的位姿/状态。
- `gui.config`：该次原生窗口配置；沿用原有四向帧、决策、原生日志和审计文件。

六节点预览的结果明确为 `mode=scene_preview_only`、`controller_result.result_type=scene_preview_only`、
`motion_commands=0`、`completed=false`。
`preview_loaded=true` 和退出码 0 只表示预览加载成功，不表示导航成功。
局部场景继续用 `completed`、原控制器结果、事后几何与停车审计判断；接触传感器仍不构成完整碰撞认证。

局部场景的公开 RGB 离线重放：

```bash
/home/sunrise/toponav_project/.venv/bin/python -m simulation.gazebo_replay \
  /absolute/path/to/run --scenario t-left --confirm-initial-clearance
```

需在项目根目录执行，或预先将项目加入 PYTHONPATH。重放不读取位置/接触审计。
新的直走/阻挡记录会将场景、净空确认和运动许可写入 `public/motion-task.json` 并绑定哈希。
重放同一记录必须显式传入相同的 `--scenario straight`（或 `blocked`）、
`--confirm-initial-clearance` 和 `--motion-permission bounded_probe`；哈希或授权不符会拒绝。
默认 `no_motion` 记录则不需要探测授权。历史冻结源码不匹配仍拒绝，不覆写旧清单。

## 原无标记控制器尚未接通的部分

原无标记控制器的完整六节点 Gazebo 导航仍需要移植并验证节点参考观测、视觉节点确认、连续动作/反馈串联，
不能把 L/T 局部完成当作整个拓扑任务完成。本次没有修改原控制器、17类网络或历史验收结果，
没有补签 S2/S6，也没有验证 ROS 桥或硬件看门狗。

基础安装和相机原理见
[Gazebo交叉验证说明](/home/sunrise/toponav_project/docs/GAZEBO_CROSS_VALIDATION.md)；
本轮实测与截图见
[多场景实测报告](/home/sunrise/toponav_project/reports/gazebo-scenes-20261020/REPORT.md)。
