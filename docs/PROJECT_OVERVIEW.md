# 项目总览

整理日期：2026-11-13。本文件是当前工作区的入口索引，不重写历史验收结论。

## 1. 项目做什么

利用先验道路拓扑和相对道路结构观测进行定位、规划和导航。代码包含三个不同层次：

1. **离散结构基线**：地图/场景校验、候选定位、公共路径规划与原子动作仿真。
2. **图像及连续仿真研究**：四向 RGB 结构模型、时序会话、直行/转角证据、连续控制与离线实验。
3. **工程接入**：PyBullet 可视化、Gazebo Transport 执行/接收租约联锁、ROS2 桥源码及测试。

这三层不是相同的能力证明。离散示例成功不代表连续运动、完整地图自主导航、ROS2 生产部署或实车安全已验收。

## 2. 模块地图

```text
configs/                    地图、场景、schema、非法输入及冻结实验配置
roadvision/                 RGB 模型、证据提取、直行/转角策略及研究实现
simulation/                 拓扑/路径规划、离散和连续仿真、Gazebo IPC
  native/                   原生 C++ 接收端租约、消息与状态联锁
ros2_ws/src/toponav_bridge/  ROS2 包源码（构建/安装目录不纳入 Git）
experiments/block7/          数据集、感知、回放、实验运行及统计工具
tools/                      测试审计、性能实验、只读审查、安装和可视化入口
tests/                      当前回归、夹具及原生探针测试
datasets/                   工作区现有模拟数据与预览
models/road_structure_frozen_v1_20261014/
                            当前保留的冻结权重及评估材料
docs/                       接口、实验方法、阶段进展和历史说明
reports/                    现有实验输出、源快照、哈希清单及测试证据
data_raw/ data_processed/   本地数据占位；当前内容不上传
edge_deploy/                后续端侧部署占位，尚无实现文件
```

空目录不被 Git 保存；不会以占位目录存在来暗示已有部署能力。

## 3. 从哪里读代码

| 关注点 | 主要入口 | 说明 |
| --- | --- | --- |
| 地图和字段约定 | `simulation/topology.py`、`configs/v2/` | [v2 使用说明](topology-v2-usage.md) |
| 路径规划 | `simulation/dijkstra.py`、`simulation/path_planning.py` | [Dijkstra](DIJKSTRA.md)、[公共规划](PATH_PLANNING.md) |
| 离散结构导航 | `simulation/structural_localization.py`、`simulation/structural_navigation.py` | [导航接口](NAVIGATION.md) |
| 四向 RGB 模型 | `roadvision/contract.py`、`roadvision/learn.py` | [冻结模型结果](ROAD_STRUCTURE_V1_RESULTS.md) |
| 时序会话 | `simulation/navigation_session.py`、`roadvision/temporal_navigation.py` | [时序导航](TEMPORAL_NAVIGATION.md) |
| 连续直行 | `roadvision/continuous_straight.py`、`simulation/continuous_straight_run.py` | [直行研究](STRAIGHT_IMPROVEMENT.md) |
| Gazebo 接入 | `simulation/gazebo_actuator.py`、`simulation/gazebo_receiver.py`、`simulation/native/` | [交叉验证](GAZEBO_CROSS_VALIDATION.md)、[接收联锁](PROJECT_REVIEW_RECEIVER_INTERLOCK.md) |
| 回归与证据 | `tools/run_project_tests.py`、`tools/project_test_audit.py` | [测试范围](TESTING.md) |

## 4. 运行与环境

### 已有本地环境

在项目根目录运行：

```bash
.venv/bin/python -m simulation.topology
.venv/bin/python -m simulation.dijkstra
.venv/bin/python -m simulation.structural_navigation
```

这是最小的纯拓扑/离散结构检查，不启动实体或仿真车辆。默认 v2 代价为 hops，不把拓扑边当成米。

### 新环境安装参考

项目原环境是 Ubuntu 22.04 / Python 3.10；ROS2 Humble 与本地虚拟环境有系统依赖关系。新建环境示意：

```bash
python3.10 -m venv .venv
.venv/bin/python -m pip install -r requirements-cpu.txt
.venv/bin/python -m pip install -r requirements.txt
# 仅在需要 PyBullet 连续仿真时安装：
.venv/bin/python -m pip install -r requirements-simulator.txt
```

本次没有在干净机器重新安装依赖。这是依赖文件的使用顺序，不保证跨平台一键复现。
`requirements.lock.txt` 是原环境的版本记录，含 CPU PyTorch 和系统相关版本，不应直接当作通用平台锁文件。
特别是固定的 `matplotlib==3.5.1` 在 Python 3.10 上可能需要系统包或源码构建；不要为了整理项目擅自升级原实验依赖。

### 可选仿真与 ROS

- [PyBullet 浏览器可视化](VISUAL_SIMULATION.md)：`tools/visualize_simulation.py`，默认监听本机；不要暴露到公网。
- [Gazebo 多场景](GAZEBO_SCENARIOS.md)及[完整流程](GAZEBO_FULL_FLOW.md)：私有运行时不进 Git；安装/构建入口为 `tools/install_gazebo_local.sh` 和 `tools/build_gazebo_receiver.sh`，不在本次整理中重新安装或运行。
- ROS 包位于 `ros2_ws/src/toponav_bridge/`，需独立准备 ROS2 Humble；源码存在不代表当前环境已重新完成构建与运行验证。

直行仍默认 `no_motion`；`bounded_probe` 与初始净空确认是显式前提。不得因 smoke 通过而放开运动权限、超时、租约或安全门槛。

## 5. 当前进展与证据边界

- **S2 未通过，S6 未完成，10Hz 未达到；没有新增实车验收。**
- 2026-11-08 的[结构性匹配复用审计](../reports/straight-match-structural-feasibility-20261108/REPORT.md)是最近的只读研究结论，没有上线新控制器或产生新性能收益。
- 本地只保留部分历史目录。缺失的清单、输入或源快照必须从真实备份恢复，不能从文字结论或当前源码补造。
- [本次状态记录](REPOSITORY_STATUS_2026-11-13.md)披露实际验证及缺口。历史 README 和旧文档中的通过数量仅对原时点与范围有效。

## 6. 文档阅读建议

先读本总览与本次状态，再按需要进入[地图契约](topology-v2-usage.md)、[导航接口](NAVIGATION.md)、[时序会话](TEMPORAL_NAVIGATION.md)或[测试说明](TESTING.md)。
`PROJECT_PROGRESS_*`、`PROJECT_REVIEW_*` 和报告各自保留日期/范围；`docs/history/` 是历史资料，不是当前入口。
为保留原始材料，本次没有批量改写旧报告的绝对路径或哈希。新入口使用仓库相对链接，旧文档中的 `/home/sunrise/toponav_project/...` 需自行映射到本地 checkout；部分引用在本快照中确实缺失。
