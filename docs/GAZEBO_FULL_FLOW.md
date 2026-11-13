# Gazebo 六节点全流程（标记辅助新基线）

## 看完整运行

```bash
bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh
```

默认选择“六节点全流程 · 视觉路标/引导线辅助导航”。等待原生 Gazebo 窗口加载，
在场景中心点击 **开始导航**。不会再在你看见窗口前跑完。

- Gazebo：真实差速轮/DART 物理运动，非瞬移、轨迹动画或真值坐标跟踪。左侧实时俯视相机采用
  observer-only 可见性掩码隐藏天花路标，避免挡住小车；这只改变展示，相机驾驶输入不受影响。
  关闭左侧图像面板可操作底层原生3D视角查看完整路标。
- 场景中心：拓扑图、当前阶段、相机已确认的节点序列。
- **小车视角**：默认显示真实前方相机，可选择前/左/右/后；“四向相机”页同时查看四个方向，
  “拓扑图”页保留大地图。原始水平相机128×128，按原像素放大，不生成虚构高清画面。
  页面显示原始采集仿真时间；导航前也显示本次初始画面，重置会清掉旧会话图像。
- Gazebo 右下角增加 **Robot FRONT** 原生实时相机面板，直接订阅 `/toponav/camera/front`。
  水平视角仅供观察；当前标记辅助控制器的上下相机输入和行为不变。
- 到站后保持零速和窗口；点击 **重置 / 再运行** 重新加载，随后再点开始导航。
- **停止 / 切换场景**：零速、保存记录、清理本次进程后可换图/目标/封路。
- 直接关闭场景中心会停止其所属服务，不影响别人的 Gazebo。

默认路线 `N01 → N02 → N03 → N04 → N06`，沿途直行、左转、右转并在 N06 停车。
勾选 C02 后，规划和物理挡墙同时生效，路线变为 `N01 → N02 → N05 → N04 → N06`。
勾选 C06 则 N06 不可达：不发运动命令，不伪造到达。

## 这次接通了什么

`graph/task → Dijkstra（跳数）→ 相机识别节点 → 三帧零速确认 → 基于拓扑出口原地转向
→ RGB 黄线跟随 → 下一节点相机闭环接近 → 目标确认停车`。

使用原六节点几何和 **0.8m** 走廊，没有加宽。新增夹具明确如下：

1. 六个节点上方 1.4m 处的真实渲染 ArUco 标记（边长 0.32m），所有标记具有统一地图朝向。
2. 地面 3.5cm 黄引导线。
3. 小车两台新增真实相机：向上识别标记/相对位置/朝向，向下跟线；256×256/90°。
   原四向 128×128 相机仍记录，**不把它们冒称为新控制器的驾驶输入**。
4. 独立 `roadvision/landmark_route.py` 控制器，保留原 `continuous_straight.py` 和17类模型。

驾驶仅输入原生同步 RGB、原始采集时间、公开拓扑/方位、标记 ID→节点及尺寸。
**不读 scene 几何、仿真位姿、里程计、接触或俯视图**。
标记像素提供相对节点位置/朝向；黄线像素提供局部横向/航向误差。
所有命令仍通过既有 MotorGate，限制 0≤v≤0.15m/s、|ω|≤0.30rad/s；正常巡航0.12m/s。
初始标记丢失、转弯标记丢失、未识别的节点、跟线证据缺失、时戳不新鲜及进度超时均终止并零速。

这是**新增人工标记辅助工程基线**。它不是原无标记自然视觉节点模型的全流程验收，
也不是 S2/S6、ROS2 桥、硬件看门狗或动态避障的通过证明。
原 `topology` 场景仍是无标记零速预览；原 L/T 控制场景继续独立保留。

## 自己交叉试验

在项目目录执行；每次使用新的 output 目录。

```bash
# 原生窗口，直接开始（场景中心则等待手动点击）
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --gui --scenario route --output /home/sunrise/toponav_project/reports/my-route-$(date +%s)

# 离屏实测：封 C02 绕行（删去 --disable C02 就是默认路线）
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --scenario route --disable C02 --output /home/sunrise/toponav_project/reports/my-detour-$(date +%s)

# 不可达：应 aborted / goal_unreachable_no_motion，motion_commands=0
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --scenario route --disable C06 --output /home/sunrise/toponav_project/reports/my-unreachable-$(date +%s)

# 物理移除 N03 天花路标；控制器不会获知故障配置，不应宣称到达 N03/N06
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --scenario route --hide-marker N03 --output /home/sunrise/toponav_project/reports/my-tag-fault-$(date +%s)

# 公开 RGB 离线重放，不读取 audit 或 scene
/home/sunrise/toponav_project/.venv/bin/python -m simulation.gazebo_replay \
  /absolute/path/to/run --scenario route
```

不可达/故障试验退出码1是预期拒绝，不是导航成功。GUI 最终 result.json 在关窗或停止后保存。
需要自动化判断时用离屏模式。

## 看哪些记录

- `decisions.jsonl`：每次原始输入时间/哈希、实际输出命令、状态、visited_nodes。
- `public/up`、`public/down`：驾驶真正使用的原生图像，六相机必须原始时戳一致。
- `public/frames`、`public/overview`：四向相机/俯视记录；俯视仅供展示。
- `public/topology.json`、`planned_route.json`、`markers.json`：公开先验，不含节点 XY 的标记注册表。
- `freeze.json`：控制器、夹具、世界及公开先验哈希。
- `result.json`：导航状态和事后几何/停车核验。
- `audit/poses.json`：事后核验每个视觉确认与真实节点的距离，不输入驾驶。

`completed=true` 需要视觉终态、目标/所有节点确认误差<0.15m、实际静止、DART已加载、
无已观测墙碰撞、无运行异常。接触覆盖不完整，`collision_free_certified=false`；
**“没有观测到碰撞”不等于经过全面碰撞安全认证**。无动态障碍/导航失效后的通用安全保证。

报告见 `/home/sunrise/toponav_project/reports/gazebo-fullflow-20261020/REPORT.md`。
