**部署状态：v2 在 configs/v2；旧版默认入口已恢复。配置工具验收通过，不等于其他分块已适配。**

# 场景与拓扑地图（v2）

更新日期：2026-10-14。保留原六节点 N01—N06、六通道 C01—C06、原坐标/尺寸和 C02 关闭样例。不改变项目环境，不新增导航业务源码。

## 1. 三类信息

| 类别 | 字段/文件 | 用途 |
| --- | --- | --- |
| 道路结构与出口连接 | graph.nodes: node_id/node_type/neighbors/exits；graph.edges: channel_id/source/target/bidirectional/enabled | 物理邻接、路口结构、出口到通道和邻居的对应、临时可通行状态 |
| 任务 | graph.task: start_node/goal_node/initial_heading_deg（朝向可选） | 一次导航任务；改起终点无需改节点道路类型 |
| 几何、标注和评价 | scene.geometry: node_positions_m/node_annotations/channel_dimensions_m/bounds_m/evaluation | 模拟器绘制、标签显示、距离和停靠半径评价；不作为视觉识别输入或拓扑加载前提 |

scene.nodes / scene.channels 是当前场景独立确认的允许节点和物理通道清单，不是由待验证图临时生成的“证明”。scene.channels 的 source_direction / target_direction 是各端点向外的局部道路方向，可检查出口是否接错。校验器不能在没有独立先验清单时从 JSON 自己证明真实道路存在；同时篡改图和先验清单不属于本次错误连接检测的保证范围。

geometry 可完全省略，也可只提供部分显示信息。reference_images 是节点上的可选辅助元数据，可省略或为空；不要求逐点照片，不把 Nxx 标签牌当作必需设施。node_annotations 只是显示文字，不是视觉监督类别。

N06 的道路类型从 goal 改为 dead_end；其目标角色只存在于 task.goal_node。N01 的 entrance 是场景入口结构标注，不是“本次起点”的必然视觉类别。node_type 描述道路结构，不保证单帧可识别；相同路口形状也不保证可唯一识别 node_id。

## 2. 原场景保持不变

```text
       N05 ── N04 ── N06（当前任务目标）
        │      │
N01 ── N02 ── N03
（当前任务起点）
```

| 节点 | 道路类型 | 模拟器坐标 m | 物理邻居 |
| --- | --- | --- | --- |
| N01 | entrance | (0,0) | N02 |
| N02 | intersection | (2,0) | N01,N03,N05 |
| N03 | corner | (4,0) | N02,N04 |
| N04 | intersection | (4,2) | N03,N05,N06 |
| N05 | corner | (2,2) | N02,N04 |
| N06 | dead_end | (6,2) | N04 |

所有通道双向，原设计长度均 2m、宽 0.8m：C01=N01—N02，C02=N02—N03，C03=N03—N04，C04=N04—N05，C05=N05—N02，C06=N04—N06。原图及关闭样例的 enabled 状态完全保留。

默认任务 N01→N06，起始朝向 0°。主路 N01→N02→N03→N04→N06、备用路 N01→N02→N05→N04→N06 均为 4 跳；在此模拟器的设计尺寸下均为 8m。拓扑规划以通道跳数工作，不读取 geometry 真值来识别地点；米制长度另用于评价。没有尺寸就不报告米制长度。

## 3. 方向约定及进入方向

exit_direction_frame = map_xy_east_north：地图参考轴 x 向东、y 向北；角度以东 0°、北 90°、西 180°、南 270°表示，逆时针为正。“东/北”是地图轴，不要求实际罗盘真东/真北，不是相机坐标。屏幕 y 向下时仅在显示层翻转。

exits.direction 指从当前节点向外离开的道路局部方向。它是道路连接先验，不是相对操作指令，也不是两个节点显示坐标之差。弯曲道路的两端局部方向不必相反，故不增加无条件“端点方向相反”的限制。本轮仍是四方向正交、双向简单图，不引入任意角度、单向边或平行边。

来路已可靠确定时，进入朝向 = 当前节点指向来路的出口角度 + 180°（模 360）。相对转角 = 出口角度 − 进入朝向（模 360）：0 straight，90 left，180 u_turn，270 right。

| 进入 N02 的来路 | 进入朝向 | 去 N01 | 去 N03 | 去 N05 |
| --- | --- | --- | --- | --- |
| N01 | 东 0° | u_turn | straight | left |
| N03 | 西 180° | straight | u_turn | right |
| N05 | 南 270° | right | left | u_turn |

同一路口从不同方向观察，其前方结构分别可为“前+左”“前+右”“左+右”；不为 N02 固定一个“左转路口”视觉类别。u_turn 只是返回来路的对应关系，不自动授权小车掉头。来路/朝向未知时返回 entry_known=false、heading_deg=null、exits=[]，等待定位，不默认朝东。当前工具不从图像恢复来路，这是后续视觉/状态估计分块的职责。

enabled=false 不删除 neighbors/exits；相对结构仍可描述，但对应出口不能作为可通行候选。C02 关闭时，从 N01 进入 N02 的 straight 对应 N03，enabled=false；left 对应 N05，enabled=true。若 C06 关闭，地图仍合法，当前任务无启用边路径；不能把运行时不可达冒充非法连接。

## 4. 通用校验

graph.schema.json / scene.schema.json 仅约束格式、通用结构、字段类型、正尺寸和双向简单图协议。schema_version=2.0 和方向协议常量是格式版本/语义，不是固定地图编号或数量。节点至少一个；边、邻居和出口不固定示例数量；节点/通道/场景 ID 为任意非空字符串。task 和 geometry 可省略。

现有检查在 v2 保留并调整字段：数组下标对齐改为 neighbor/exits 显式绑定；起终点不再等于场景中写死的任务，而是检查 task 引用；方向与显示坐标一致改为方向与独立允许通道端点属性一致。尺寸正值校验移到 geometry；保留唯一 ID、禁止自连接、重复边、双向互邻、端点存在、邻居/边一致、通道事实、物理连通性及参考路线检查。

31 项语义检查会将缺少可选 task/geometry 的相应检查标为 applicable=false。物理图仍要求连通，关闭后的启用边可达性单独报告，不作为非法拓扑判据。schema 不能用静态 enum 判断引用是否存在，必须同时运行语义检查。

现有 13 个错误样例仍保留。negative_edge_length 的错误尺寸迁入对应 .scene.json，manifest 指定 scene_file；它现在是几何配置错误，不能称作非法拓扑边。wrong_exit_direction 仍被拒绝，但依据是允许通道出口事实，而非显示坐标。

## 5. 验收与可复现命令

四节点任意编号、七节点任意编号（含四出口路口）、重命名六节点、不带 task/geometry/照片、改变显示布局、同地图不同任务，以及原图、C02 关闭图，全部通过相同 schema 和语义检查。9 个正常样例；原 13 个错误样例全部拒绝；N02 三进入方向的 9 个对应全部通过；未知来路不猜测；关闭目标通道可加载但运行时不可达。

精确读取、完整校验和相对出口查询命令见根 README.md；结果见 reports/topology_v2/acceptance.json 和 reports/topology_v2/ACCEPTANCE.md。2026-10-12 的环境/地图验收文件属于 v1 历史记录，不应作为 v2 当前验收依据。旧地图配置已保存在 backups/map-review-20261014-1xfFRX/before.tar.gz。
