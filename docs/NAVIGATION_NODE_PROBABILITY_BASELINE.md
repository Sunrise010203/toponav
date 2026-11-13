> 历史节点概率基线：CLI默认已固定到configs/legacy。本说明不代表当前结构观测系统；升级前原文见docs/history/2026-10-14-before-unification。

# 最小导航闭环

远程项目：`/home/sunrise/toponav_project`。2026-10-13（Asia/Shanghai）。

## 运行

在 Ubuntu 或 SSH 远程 shell 中：

```bash
cd /home/sunrise/toponav_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
export MPLBACKEND=Agg
export MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib
python -m simulation.navigation \
  --output reports/navigation_ideal.json \
  --plot reports/navigation_ideal.png
```

默认 N01→N06，理想识别置信度 1.0。标准输出是完整 JSON；`--output` 保存相同 JSON，`--plot` 绘制实际执行轨迹。输出路径是本次 shell 的相对路径；默认地图和场景路径基于模块位置。

```bash
# 临时关闭 C02，保持地图文件不变，走 N05 绕行
python -m simulation.navigation --disable C02 \
  --output reports/navigation_detour.json --plot reports/navigation_detour.png

# C06 关闭后目标不可达；预期退出码 1，不执行移动
python -m simulation.navigation --disable C06 \
  --output reports/navigation_unreachable.json --plot reports/navigation_unreachable.png

# 使用现有关闭地图，也会绕行
python -m simulation.navigation --map configs/graph_blocked_C02.json

# 反向导航；方向是绝对方向，不依赖初始朝向
python -m simulation.navigation --start N06 --goal N01

# 模拟置信度 0.8，其余概率均分：能继续导航
python -m simulation.navigation --confidence 0.8

# 模拟均匀概率：定位不确定，停止，预期退出码 1
python -m simulation.navigation --confidence 0.16666666666666666

# 限制两次移动：观察移动后的 N03，再报告 step_limit
python -m simulation.navigation --max-steps 2

# 完整测试：204 passed（新增 100 项，原有 104 项）
python -m pytest -q
```

退出码：0＝真实环境已到目标；1＝未到达（`unreachable`、`localization_uncertain`、`blocked`、`step_limit` 或 `false_goal`）；2＝无效参数/数据或读写失败。`false_goal` 是防止定位误报目标的真值审计状态，不是用真值修正定位。

## 模块及接口

实现：`simulation/navigation.py`；测试：`tests/test_navigation.py`。复用未修改的 `simulation/dijkstra.py`。

| 部件 | 接口 | 输入/输出 |
| --- | --- | --- |
| 二维环境 | `Environment2D(graph, scene, start, disabled_channels)` | 维护真实节点、米制坐标、角度、累计路程 |
| 模拟识别 | `SimulatedRecognizer(graph, confidence).observe(env)` | 每次读取当前环境节点，输出全节点概率字典 |
| 定位 | `localize(probabilities, threshold=0.6)` | 只读概率；输出估计节点和置信度；低置信度/并列最大值返回 uncertain |
| 规划与决策 | `decide(graph, localization, goal, disabled_channels)` | 从估计节点重跑 Dijkstra；选第一个出口方向或 stop |
| 动作执行 | `env.step(action)` | east/north/west/south/stop；返回 moved/blocked/stopped |
| 闭环 | `run_navigation(...)` | observe→localize→plan/decide→step→重新 observe，输出完整记录 |

环境根据 `scene.json` 的物理通道和坐标执行方向动作，还必须满足 `graph.json` 的 enabled/bidirectional 和临时关闭约束。不接受目标节点作为动作，不能通过修改规划索引冒充移动。即使估计位置错误，动作仍从真实位置执行；真实位置没有对应出口时阻止动作。

识别模拟器是唯一将环境真值转换为概率的部件。定位和决策没有环境输入；日志、成功审计读取真值但不参与选择路径或动作。每次真实移动后必须重新观测，即使已经移到目标或达到动作步数限制。

## 日志核对

`steps` 每项有：
- `before`、`after`：真实 `node_id/position_m/heading_deg/distance_m` 快照。
- `probabilities`：这一步动作前重新取得的识别分布。
- `localization`：从概率得到的节点估计。
- `decision`：状态、方向动作、当步重新规划的完整路线。
- `execution`：实际移动/阻止/停止结果。

理想观测下：`probabilities[before.node_id] == 1`。发生移动时 before≠after，下一项的 before 等于本项 after，下一项概率在该 after 节点为 1。最后一项包含目标节点的新观测和 stop，停止前后坐标相同。

正常运行记录：

| 观测节点 | 动作 | 执行后坐标 (m) |
| --- | --- | --- |
| N01 | east | (2,0) / N02 |
| N02 | east | (4,0) / N03 |
| N03 | north | (4,2) / N04 |
| N04 | east | (6,2) / N06 |
| N06 | stop | (6,2) / N06 |

## 验收与范围

`reports/NAVIGATION_ACCEPTANCE.md`、`reports/navigation_audit.json` 和 `logs/navigation_tests_2026-10-13.txt` 是本次结果。原 README、SCENE 和历史验收文件描述的是 2026-10-12 的环境/地图阶段；本次新增闭环没有覆盖历史记录。

这是节点级二维状态仿真：一个移动动作执行完整相邻通道，不模拟沿途连续位姿、速度、碰撞动力学或时间。不连接摄像头、不训练模型、不发布 ROS 话题，不控制 turtlesim 或实车。置信度模式是确定性的概率模拟，不是随机噪声/真实视觉鲁棒性测试。可以在已有 ROS 环境运行，但不能将本次结果称为 ROS 通信或硬件导航验收。
