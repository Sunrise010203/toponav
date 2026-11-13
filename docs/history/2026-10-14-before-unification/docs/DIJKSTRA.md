# Dijkstra 路径规划与断路重规划

远程项目：`/home/sunrise/toponav_project`。使用既有 `.venv`，无需新增依赖。规划器为 `simulation/dijkstra.py`，没有接入 ROS 控制或障碍检测。

## 命令行

在 Ubuntu 中执行：

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.dijkstra
.venv/bin/python -m simulation.dijkstra --disable C02
.venv/bin/python -m simulation.dijkstra --map configs/graph_blocked_C02.json
.venv/bin/python -m simulation.dijkstra --start N02 --goal N06 --disable C02
.venv/bin/python -m simulation.dijkstra --disable C06
.venv/bin/python -m simulation.dijkstra --disable C02 C05
```

从 Windows 通过 SSH 执行，例如：

```powershell
ssh -i "$HOME\.ssh\codex_ros2" -o BatchMode=yes -o StrictHostKeyChecking=yes ubuntu-vm 'cd /home/sunrise/toponav_project && .venv/bin/python -m simulation.dijkstra --disable C02'
```

正常路线：N01 → N02 → N03 → N04 → N06，通道 C01/C02/C03/C06，8 米。

禁用 C02：N01 → N02 → N05 → N04 → N06，通道 C01/C05/C04/C06，8 米。两条路线等长；正常情况下优先队列按节点 ID 打破距离相同的平局，先处理 N03，并保留首个同距前驱，因此结果稳定。

从 N02 遇到 C02 关闭：N02 → N05 → N04 → N06，6 米。

禁用 C06 或同时禁用 C02/C05：目标不可达。

输出 JSON：`status`、`start`、`goal`、`nodes`、`channels`、`length_m`。`reachable` 退出 0；`unreachable` 返回空节点/通道数组和 `length_m: null`，退出 1；输入错误返回 `invalid` 与 `error`，退出 2。不要将不可达（合法地图中的通道关闭）与非法输入混淆。命令行参数用法错误由 argparse 报告到标准错误，退出 2。

`--disable C02 C05` 可关闭多个通道，也可重复 `--disable`。关闭是本次调用的临时状态，不写回地图；再次调用时需传入仍然关闭的通道。地图已有的 `enabled=false` 始终生效，不能用临时参数重新启用。

## 在程序中重新规划

```python
import json
from pathlib import Path
from simulation.dijkstra import plan

graph = json.loads(Path("configs/graph.json").read_text(encoding="utf-8"))
initial = plan(graph)
closed = {"C02"}
replanned = plan(graph, start="N02", goal="N06", disabled_channels=closed)
closed.add("C06")
unreachable = plan(graph, start="N02", goal="N06", disabled_channels=closed)
assert unreachable["status"] == "unreachable"
```

函数每次读取当前边状态、重新构造可通行邻接表并运行 Dijkstra，没有缓存旧路线，也不会修改输入。以 `length_m` 为代价，只使用 `edges` 中启用且未被禁用的通道；`nodes[].neighbors` 是物理拓扑，可能仍包含已关闭通道，不能直接用作可行驶邻接表。双向通道禁用后两个方向均不可通行；单向通道不自动加入反向边。

起终点相同返回单节点、零距离。未知节点/通道、重复 ID、非法端点、非有限/负长度及非布尔状态会抛出 `ValueError`。函数输入应为本项目 JSON 地图结构和通道 ID 集合/列表；不是场景事实或完整地图一致性校验器，既有 schema/jq 校验仍适用。

## 验证

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m pytest -q --junitxml=reports/dijkstra-pytest.xml
```

测试包括真实地图正常/替代/不可达路线、已有关闭地图、从当前节点重规划、边状态更新后重算、加权而非最少跳数、零权重、单向/反向通道、输入错误、地图不被修改和 CLI 退出码。针对实际六节点地图的全部 64 个通道禁用子集、每个子集全部 36 个起终点组合（共 2304 次规划），以独立 Floyd–Warshall 算法核对最短距离和可达性，并逐边核验输出路线。

虚拟机的实际 CLI 验收结果见 `reports/dijkstra_acceptance.json`；测试日志见 `logs/dijkstra-tests.log`。原地图保留不变。路径规划完成不等于实车导航、视觉检测或控制闭环完成。
