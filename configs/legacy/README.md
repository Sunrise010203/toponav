# v1兼容数据

此目录逐字节保留系统统一前的v1地图、场景、schema和13份错误样例。
旧节点概率入口python -m simulation.navigation默认使用本目录。
v1规划示例：python -m simulation.dijkstra --map configs/legacy/graph.json。
v1结构导航示例：python -m simulation.structural_navigation --map configs/legacy/graph.json --scene configs/legacy/scene.json --heading 0。
v1继续按length_m加权，出口方向保留旧约定；当前主系统及新增能力以默认v2为准。
