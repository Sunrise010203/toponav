# 统一路径规划与相对动作接口

2026-10-14。模块：simulation/path_planning.py；当前结构导航已实际调用此模块。

## 路线接口

plan_from_estimate(graph, estimate, goal, disabled_channels=())。

estimate可为明确localized的单节点估计，也可为update_candidates的完整返回值。
完整返回值必须status=localized、observation_reliable=true、仅一个候选且与estimate一致；不能提取estimate丢弃执行失败/未确认等顶层状态。
位置不可靠时返回localization_uncertain和空路线，不回退到地图默认起点。
地图关闭状态与临时禁用通道同时生效，未知通道/节点报错。

v2代价为最少通道跳数，cost_unit=hops、cost/hop_count为跳数，length_m=null。
米制评价只由场景执行器产生。v1显式输入仍按length_m加权，返回原格式。

## 下一动作接口

next_action(graph, estimate, route, disabled_channels=())。

输出status/action/request_observation。运动action统一straight/left/right/uturn；安全状态为stop。
只转换当前路线第一个出口，不提前生成整条固定动作序列。
已到达或不可达均stop；过期路线、关闭的首通道或节点不一致返回replan_required。
朝向/位置不确定及冲突停止并请求观测，不擅自舍入任意角度。

v2出口朝向来自nodes[].exits.direction，绝不从geometry/显示坐标推导。
已确认incoming_channel的进入朝向等于当前节点指向来路的出口角度+180°；支持两端局部方向不相反的弯曲道路。
直接朝向使用heading_deg和heading_status=certain；来路使用incoming_channel和incoming_status=certain。
两种可信证据冲突时停止；已经原地转向后不能继续把旧来路当作当前朝向。
旧v1地图保留原坐标方向约定，仅作为兼容模式，不把它宣称为v2字段。

## 示例

```python
import json
from pathlib import Path
from simulation.path_planning import plan_from_estimate, next_action
graph = json.loads(Path('/home/sunrise/toponav_project/configs/graph.json').read_text())
estimate = {'status': 'localized', 'node_id': 'N02',
            'incoming_channel': 'C01', 'incoming_status': 'certain'}
route = plan_from_estimate(graph, estimate, 'N06', ['C02'])
decision = next_action(graph, estimate, route, ['C02'])
# route: N02 -> N05 -> N04 -> N06，3 hops，length_m=None
# decision: move / left
```

公共规划函数只返回数据，不执行运动、不读取环境真值、不改写地图。
物理执行器只在收到相对动作后依据自己的真实朝向执行；反馈不携带真值。
测试见tests/test_path_planning.py（v1兼容）和tests/test_system_integration.py（v2完整链路）。
