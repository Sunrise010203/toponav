# 当前v2地图使用与校验

2026-10-14统一集成后，configs/graph.json和scene.json已是v2；不再处于“配置已做好但程序未适配”阶段。
configs/v2同名文件保留为明确版本入口，与当前默认配置一致。旧版位于configs/legacy。

## 完整校验

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.topology
.venv/bin/python -m simulation.topology --map configs/v2/examples/four_nodes.graph.json --scene configs/v2/examples/four_nodes.scene.json
.venv/bin/python -m simulation.topology --map configs/v2/examples/no_task_no_geometry_no_photos.graph.json --scene configs/v2/examples/no_task_no_geometry_no_photos.scene.json
```

此CLI使用v2 graph/scene Schema加共享运行时跨文件校验，不只检查JSON可解析。
检查唯一ID、引用存在、禁止自连接/重复无向边、物理连通、双向互邻、出口与邻居/通道绑定、方向不歧义、独立场景端点/方向、可选几何元数据及参考路线。
graph.task与geometry均可省略；不要求逐点照片或编号标牌。缺失任务时task_reachable=null。
关闭目标通道时地图仍合法，task_reachable=false，导航输出unreachable。
校验CLI成功退出0，非法配置退出2；导航安全停止退出1。

错误样例必须使用configs/v2/invalid_cases/manifest.json中的scene_file配对。
negative_edge_length对应negative_edge_length.scene.json，属于几何配置错误；invalid_start是任务错误。
原图与场景清单必须独立核实；同时篡改二者不能据此证明现实道路存在。

## 出口与进入方向

exits.direction为当前节点向外的局部道路方向，东0/北90/西180/南270°。
进入朝向为当前节点指向来路的出口方向+180°；再与目标出口相减得到straight/left/right/uturn。
例如从N01进入N02后去N05为left；从N03进入N02后去N05为right。
未知来路和未知朝向不能默认朝东。u_turn旧拼写仅在动作输入边界兼容为uturn。
弯曲通道两端局部方向无需相反，到达朝向由目的端属性确定，不由两节点显示坐标差确定。

## 数据、图片与文档

当前字段约定见reports/topology_v2/FIELDS.md和reports/SCENE.md。
六节点示意图为reports/topology.png、reports/topology.svg，源文件为configs/topology.dot（不是不存在的configs/v2/topology.dot）。
显示坐标、图中文字和通道米制尺寸均不进入结构定位/规划。
当前集成验收见reports/system_unification/ACCEPTANCE.md；2026-10-14升级前的topology_v2验收保留为历史配置阶段证据，版本索引见reports/HISTORY.md。
