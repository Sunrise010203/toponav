# 第二分块：路径规划与最小相对动作转换

运行环境：Ubuntu 虚拟机 `/home/sunrise/toponav_project`，使用原 `.venv`。新接口为 `simulation/path_planning.py`，原 Dijkstra、测试及地图文件均保留，不新增依赖。

## 与旧实现的关系

原 `simulation/dijkstra.py` 保持不变，继续处理最短路、禁用通道及不可达。

新规划块使用 `plan_from_estimate` 与 `next_action`，而不是旧 `navigation.decide`。旧 `navigation.py` 是历史离散二维仿真，它的 east/north 等动作是仿真器内部世界坐标指令；该仿真执行器未在本次迁移中改变。**不能将新接口的相对动作直接交给旧仿真器，也不能把旧接口的绝对方向作为实车控制动作。** 本次没有迁移执行器或实现 ROS/实车控制。

## 1. 输入与路线

```python
plan_from_estimate(graph, estimate, goal, disabled_channels=())
```

- `graph`：现有 `configs/graph.json` 对应的字典。
- `estimate`：第一分块提供的估计位置。沿用已有格式，例如 `{"status": "localized", "node_id": "N02"}`。`localized` 表示上游已接受此单一节点估计；不在规划块中重做定位或选择最大概率假设。
- `goal`：目标节点 ID。
- `disabled_channels`：本次临时禁用的通道 ID 集合或列表；地图 `enabled=false` 同样生效。

也可直接传入第一分块 `structural_localization.update_candidates(...)` 的完整返回值，无需手工选择其中一个候选。接口会同时核验 `status=localized`、`observation_reliable=true`、`candidates` 只有一个节点/朝向候选，以及 `estimate` 与该候选完全一致；全部成立才将该假设视为可信位置与朝向。即使有单个 `estimate`，执行未确认/失败、低置信度、歧义或观测不匹配等顶层状态仍然停止。不要只提取 `estimate` 而丢弃这些状态。

已确认位置时直接调用原 Dijkstra，原样返回 `status/start/goal/nodes/channels/length_m`。位置缺失、丢失或不确定时返回 `localization_uncertain` 和空路线，**不使用地图默认起点**，也不选择候选节点继续行驶。

未知节点/通道等非法规划输入仍由原 Dijkstra 抛出 `ValueError`；合法地图的断路返回 `unreachable`，不是非法输入。

## 2. 朝向与进入通道

在同一 `estimate` 中添加至少一种明确可信的证据：

```python
{"status": "localized", "node_id": "N02",
 "incoming_channel": "C01", "incoming_status": "certain"}

{"status": "localized", "node_id": "N02",
 "heading_deg": 0, "heading_status": "certain"}
```

只有显式 `certain` 才可使用。没有可信证据、显式 `uncertain`、进入通道有多个候选、未知通道、通道不连接当前节点、单向通道无法从该方向到达或两种证据冲突，均停止并请求重新观测。

进入通道表示**刚从另一端到达当前节点，且未发生未记录的转向**。朝向按“另一端 → 当前节点”求取，不是“当前节点 → 另一端”；若已经原地旋转，不得继续把旧进入通道标为确定的当前朝向。只提供新的可信 `heading_deg`，或将朝向标为不确定后重新观测。

已到达后才关闭的通道仍可以描述可信的到达历史，但不因此重新开放或加入可通行出口。

坐标按现有地图：x 正为东、y 正为北；朝向东为 0°、北为 90°，逆时针增加。出口与到达方向根据节点坐标和通道端点计算，地图中的 `actions: [east, north, ...]` 不会被直接作为控制动作。

## 3. 相对动作与安全停止

```python
next_action(graph, estimate, route, disabled_channels=())
```

输入应为新规划接口刚返回的路线及当前禁用通道，转换的只是路线第一个出口，不是提前生成整条路线的固定动作序列。

| 出口相对当前朝向的逆时针转角 | action |
|---|---|
| 0° | `straight`（直行） |
| 90° | `left`（左转） |
| 270° | `right`（右转） |
| 180° | `uturn`（掉头） |

例如在 N02 选择向北的 C05：从 C01 进入时为左转，从 C02 进入时为右转，从 C05 进入后回到 C05 为掉头；当前朝向北时为直行。同一绝对出口不能总使用同一个控制动作。

只处理这四种相对转角。其他角度（如朝向估计为 45°）、未知/无效/零长度通道几何返回 `turn_unresolved` 或 `geometry_unknown`，不擅自舍入为某个动作。1e-6° 容差只消除浮点误差，不是操控容差或置信度阈值。

结果包含 `status/action/request_observation`：

- 可行动：`move` + 四种相对动作之一；`request_observation=false`。
- 位置/朝向不确定、证据冲突或转角未解：`action=stop`，`request_observation=true`。调用方应保持停止，再触发重新观测，不能先行驶再确认。
- 到达目标：`reached/stop`，不再要求朝向；即使所有通道关闭也停止。
- 不可达：`unreachable/stop`。
- 估计节点已变化、路线首通道关闭或首通道方向不符：`replan_required/stop`，重新调用规划接口；不能执行旧动作。

所有函数只返回数据，不执行运动、不改写地图，也不读取环境真实位置或默认地图朝向。

新接口使用 `straight/left/right/uturn`；第一分块的相对结构键和历史结构仿真使用 `front/left/right/back`。直行对应 front，掉头对应 back，但本次不改执行器或上一动作反馈协议。对接时应在执行边界明确转换名称，不能直接把两套字符串混用。

## 4. 虚拟机使用示例

```bash
cd /home/sunrise/toponav_project
.venv/bin/python - <<'PY'
import json
from pathlib import Path
from simulation.path_planning import plan_from_estimate, next_action

graph = json.loads(Path('configs/graph.json').read_text(encoding='utf-8'))
position = {'status': 'localized', 'node_id': 'N02',
            'incoming_channel': 'C01', 'incoming_status': 'certain'}
closed = {'C02'}
route = plan_from_estimate(graph, position, 'N06', closed)
action = next_action(graph, position, route, closed)
print(json.dumps({'route': route, 'decision': action}, ensure_ascii=False))
# N02 -> N05 -> N04 -> N06，6m；左转进入 C05。
PY
```

## 5. 测试

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m pytest tests/test_dijkstra.py -q
.venv/bin/python -m pytest tests/test_path_planning.py -q
.venv/bin/python -m pytest -q
```

保留原最短路、禁用边、不可达及全禁用组合测试。新测试覆盖四向出口与四种相对动作、同出口不同进入方向、状态不确定、方向冲突、到达后停止、断路重规划、单向进入证据及地图不变。

本次实际验收见 `reports/path_planning_acceptance_2026-10-14.json`，日志见 `logs/path-planning-tests-2026-10-14.log`。
