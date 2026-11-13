# Dijkstra：v2跳数规划与v1兼容

2026-10-14。模块simulation/dijkstra.py。默认地图configs/graph.json已切换为v2。

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.dijkstra
.venv/bin/python -m simulation.dijkstra --disable C02
.venv/bin/python -m simulation.dijkstra --start N02 --goal N06 --disable C02
.venv/bin/python -m simulation.dijkstra --disable C06
.venv/bin/python -m simulation.dijkstra --map configs/legacy/graph.json
```

接口plan(graph, start=None, goal=None, disabled_channels=())。
v2默认任务从graph.task读取；任务可省略，但调用方必须明确提供起终点。
v2以每条通道一跳为代价，不读取geometry或用显示坐标计算长度。
返回status/start/goal/nodes/channels/length_m，以及cost/cost_unit/hop_count；cost_unit=hops，length_m=null。
不可达时数组为空，cost/hop_count/length_m均为null；起终点相同为零跳。
正常默认路径N01→N02→N03→N04→N06，4跳；C02关闭经N05绕行仍4跳；从N02绕行3跳。

v1显式输入保持length_m加权最短路及原输出格式，不为旧结果追加或改造单位。
原地图对应路线8m属于v1权重/场景评价，不是v2规划输出中的米制路线长度。

规划只用已启用且未临时禁用的edges，不直接把物理neighbors作为可通行表。
v2在规划前验证schema及节点/边/出口引用；场景配对使用python -m simulation.topology校验。
未知ID、非法权重/字段与不可达是不同情况；CLI分别退出2和1，成功退出0。
每次调用重新规划，不缓存旧关闭状态，不改输入地图或配置文件。
测试涵盖v1权重与全部封路组合，v2以独立Floyd–Warshall跳数距离核对。
