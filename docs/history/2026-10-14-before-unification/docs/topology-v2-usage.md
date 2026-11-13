# v2 地图配置使用说明（默认程序尚未适配）

本目录配置/工具验收已通过；默认 configs 继续使用旧版以保持现有仿真程序可运行。下列命令显式读取 configs/v2。其他分块的最小字段适配须用户另行确认。

# 先验拓扑图引导的智能小车视觉导航：环境与地图

远程项目：`/home/sunrise/toponav_project`。环境建立于 2026-10-12；第一分块配置更新于 2026-10-14（Asia/Shanghai）。

本交付不新增导航业务源码，只提供环境、二维场景、拓扑地图、声明式校验配置、错误地图样例、图示和验收记录。读取/查询/校验使用现成 CLI；依赖烟测是临时终端命令，不保存自编 Python/ROS 程序。

## 1. 环境边界与启动

复用 Ubuntu 22.04、Python 3.10 和 `/opt/ros/humble`。项目 `.venv` 允许读取 system-site-packages 以复用 apt 安装的 ROS Python 包；pip 升级包只在 `.venv` 内。没有修改系统包、SSH config 或 shell 启动文件；没有安装 CUDA、TensorRT 或 Gazebo。

配置范围：NumPy/SciPy/NetworkX、Matplotlib/Pandas/Seaborn/scikit-learn、OpenCV/Pillow/PyYAML、CPU PyTorch/torchvision、ONNX/ONNX Runtime/ONNX Script、pytest、JSON Schema CLI、FFmpeg（imageio-ffmpeg 自带二进制）。系统工具 jq/Graphviz/colcon/ROS 2 复用已有安装。NumPy 固定 1.26.4，以保持 Humble cv_bridge 的 NumPy 1.x ABI。Matplotlib 固定 3.5.1，与系统继承的 mpl_toolkits 对齐，二维及三维绘图已实际复测；不是盲目安装最新版。补齐 cffi 以满足继承的 PyNaCl 依赖。

虚拟机仅约 4GB 内存：适合小图、CPU 烟测、小批量离线推理；批量训练建议迁移到更大内存或 GPU 机器，不将“安装依赖”宣称为已完成训练或端侧部署。

在 Ubuntu 终端执行：

```bash
cd /home/sunrise/toponav_project
source /opt/ros/humble/setup.bash
source /home/sunrise/toponav_project/.venv/bin/activate
export MPLBACKEND=Agg
export MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib
export TORCH_HOME=/home/sunrise/toponav_project/.cache/torch
export XDG_CACHE_HOME=/home/sunrise/toponav_project/.cache/xdg
export OMP_NUM_THREADS=2
export OPENBLAS_NUM_THREADS=2
export MKL_NUM_THREADS=2
python --version
python -m pip check
```

每次独立 SSH 会话都要重新激活。现有 `ros2`/`colcon` 的系统 shebang 仍用系统 Python；项目内 Python 导入 ROS 包时需先 source ROS setup，烟测已单独覆盖这条路径。激活后可直接使用 ffmpeg（项目 .venv/bin/ffmpeg 符号链接指向 imageio-ffmpeg 自带二进制）。当前使用 headless OpenCV，适合 SSH 图像/视频处理，不提供 `cv2.imshow` 桌面窗口。

## 2. 文件与目录

- `configs/v2/graph.json`：v2 正式拓扑，保留六节点和六条双向通道，任务在可选 `task` 中。
- `configs/v2/scene.json`：独立允许连接清单；可选 `geometry` 仅供模拟器显示、标注和评价。
- `configs/v2/graph.schema.json`、`configs/v2/scene.schema.json`：通用格式，不写死示例编号、数量、位置或尺寸。
- `configs/v2/graph_blocked_C02.json`：C02 暂时关闭，物理拓扑保留。
- `configs/v2/invalid_cases/`：原 13 个错误样例与配对场景清单，不能用作导航输入。
- `configs/v2/examples/`：4/7 节点、任意编号、四出口路口、无 task/geometry/照片及运行状态回归样例。
- `reports/SCENE.md`：v2 字段、方向、任务与几何信息详细约定。
- `reports/topology_v2/acceptance.json`、`reports/topology_v2/ACCEPTANCE.md`：当前验收。
- `reports/topology_v2/*.checks.json`、`reports/topology_v2/invalid_cases/`：逐条语义/错误检查结果。
- `reports/topology.png`、`reports/topology.svg`、`configs/v2/topology.dot`：原六节点图示，连接及当前 N01→N06 任务不变；显示标签不是视觉类别。
- `reports/topology_v2/map.sha256`：当前配置哈希；`backups/map-review-20261014-1xfFRX/`：修改前备份。
- `requirements.txt`、`requirements-cpu.txt`、`requirements.lock.txt`、`reports/environment.json`：未改动的环境依赖与原运行烟测记录。
- `data_raw/`、`data_processed/`、`simulation/`、`models/`、`edge_deploy/`、`ros2_ws/src/`：仍是预留目录，没有新增业务源码、真实图像或训练模型。
- `logs/`、`docs/`、`tools/virtualenv.pyz`：记录、设计计划及第三方环境引导工具。

2026-10-12 的 v1 验收报告和旧逐项检查结果保留为历史证据，不作为当前 v2 通过的依据。

## 3. 读取地图和查询邻居

```bash
cd /home/sunrise/toponav_project
jq '{scene_id,task,node_count:(.nodes|length),channel_count:(.edges|length)}' configs/v2/graph.json
jq --arg id N02 '.nodes[]|select(.node_id==$id)|{node_id,node_type,neighbors,exits}' configs/v2/graph.json
jq --arg id N04 '.nodes[]|select(.node_id==$id)|{node_id,node_type,neighbors,exits}' configs/v2/graph.json
jq -e --arg id N99 '.nodes[]|select(.node_id==$id)|.neighbors' configs/v2/graph.json
```

N02 邻居 N01/N03/N05；N04 邻居 N03/N05/N06。最后一个查询为未知节点示例，无结果并退出 4。exits 显式绑定 neighbor_id、channel_id 和绝对出口 direction，不再使用 actions 平行数组。reference_images 为可选辅助元数据，可省略；地图不依赖逐点照片或编号标牌。

## 4. 完整合法性及相对出口检查

以下先验证两份 schema，再检查跨节点/跨文件一致性；schema 成功不等于连接合法。请选择同一场景配对的 MAP/SCENE。执行 schema 失败时停止，不将历史输出当成当前结果。

```bash
cd /home/sunrise/toponav_project
MAP=configs/v2/graph.json
SCENE=configs/v2/scene.json
.venv/bin/check-jsonschema --schemafile configs/v2/graph.schema.json "$MAP"
.venv/bin/check-jsonschema --schemafile configs/v2/scene.schema.json "$SCENE"
jq -s 'def reachable($g; $start):
 reduce range(0; $g.nodes|length) as $i ([$start];
   . as $seen | ($seen + [$g.nodes[] | select(.node_id as $id | $seen|index($id)) | .neighbors[]]) | unique);
def edge_matches($e; $s):
 any($s.channels[];
   .channel_id==$e.channel_id and .bidirectional==$e.bidirectional and
   ((.source==$e.source and .target==$e.target) or (.source==$e.target and .target==$e.source)));
def exit_matches($x; $id; $s):
 any($s.channels[]; .channel_id==$x.channel_id and
   ((.source==$id and .target==$x.neighbor_id and .source_direction==$x.direction) or
    (.target==$id and .source==$x.neighbor_id and .target_direction==$x.direction)));
.[0] as $g | .[1] as $s |
($g.nodes|map(.node_id)) as $ids |
($s.nodes|map(.node_id)) as $scene_ids |
($s.channels|map(.channel_id)) as $channels |
($g.task.start_node // $g.nodes[0].node_id) as $start |
($s.geometry // {}) as $geo |
[
 {check:"unique_node_ids",pass:(($ids|unique|length)==($ids|length))},
 {check:"scene_node_set",pass:(($ids|sort)==($scene_ids|sort))},
 {check:"task_endpoints_exist",applicable:($g|has("task")),pass:(if $g|has("task") then ($ids|index($g.task.start_node))!=null and ($ids|index($g.task.goal_node))!=null else true end)},
 {check:"neighbor_exit_alignment",pass:all($g.nodes[]; (.neighbors|sort)==(.exits|map(.neighbor_id)|sort))},
 {check:"neighbors_exist",pass:all($g.nodes[].neighbors[]; . as $id | ($ids|index($id))!=null)},
 {check:"no_self_connections",pass:all($g.nodes[]; .node_id as $id | (.neighbors|index($id))==null)},
 {check:"unique_neighbors",pass:all($g.nodes[]; (.neighbors|unique|length)==(.neighbors|length))},
 {check:"reciprocal_neighbors",pass:all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($g.nodes[]; .node_id==$v and (.neighbors|index($n.node_id))!=null)))},
 {check:"neighbors_have_physical_channel",pass:all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($s.channels[]; (.source==$n.node_id and .target==$v) or (.target==$n.node_id and .source==$v))))},
 {check:"neighbors_have_map_edge",pass:all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($g.edges[]; (.source==$n.node_id and .target==$v) or (.target==$n.node_id and .source==$v))))},
 {check:"edge_endpoints_exist",pass:all($g.edges[]; . as $e | ($ids|index($e.source))!=null and ($ids|index($e.target))!=null)},
 {check:"no_self_edges",pass:all($g.edges[]; .source!=.target)},
 {check:"unique_channel_ids",pass:(($g.edges|map(.channel_id)|unique|length)==($g.edges|length))},
 {check:"unique_undirected_edges",pass:(($g.edges|map([.source,.target]|sort)|unique|length)==($g.edges|length))},
 {check:"edges_match_physical_channels",pass:all($g.edges[]; edge_matches(.;$s))},
 {check:"physical_channel_set_complete",pass:(($g.edges|map(.channel_id)|sort)==($channels|sort))},
 {check:"edges_have_reciprocal_neighbors",pass:all($g.edges[]; . as $e | any($g.nodes[]; .node_id==$e.source and (.neighbors|index($e.target))!=null) and any($g.nodes[]; .node_id==$e.target and (.neighbors|index($e.source))!=null))},
 {check:"exit_directions_match_physical_channels",pass:all($g.nodes[]; . as $n | all($n.exits[]; exit_matches(.;$n.node_id;$s)))},
 {check:"all_nodes_reachable_from_start",pass:((reachable($g;$start)|sort)==($ids|sort))},
 {check:"goal_reachable_from_start",applicable:($g|has("task")),pass:(if $g|has("task") then (reachable($g;$start)|index($g.task.goal_node))!=null else true end)},
 {check:"declared_routes_follow_map_edges",applicable:($geo.evaluation|has("reference_routes")),pass:all(($geo.evaluation.reference_routes // {})[]; . as $route |
   (if $geo.evaluation.reference_task then $route[0]==$geo.evaluation.reference_task.start_node and $route[-1]==$geo.evaluation.reference_task.goal_node else true end) and
   all(range(0;($route|length)-1); . as $i | any($g.edges[]; (.source==$route[$i] and .target==$route[$i+1]) or (.target==$route[$i] and .source==$route[$i+1]))))},
 {check:"scene_and_direction_frame_match",pass:($g.scene_id==$s.scene_id and $g.exit_direction_frame==$s.exit_direction_frame)},
 {check:"unique_exit_bindings",pass:all($g.nodes[]; (.exits|map(.neighbor_id)|unique|length)==(.exits|length) and (.exits|map(.channel_id)|unique|length)==(.exits|length))},
 {check:"exit_bindings_match_map_edges",pass:all($g.nodes[]; . as $n | all($n.exits[]; . as $x | any($g.edges[]; .channel_id==$x.channel_id and ((.source==$n.node_id and .target==$x.neighbor_id) or (.target==$n.node_id and .source==$x.neighbor_id)))))},
 {check:"scene_unique_node_ids",pass:(($scene_ids|unique|length)==($scene_ids|length))},
 {check:"scene_unique_channel_ids",pass:(($channels|unique|length)==($channels|length))},
 {check:"scene_channels_valid",pass:(all($s.channels[]; . as $e | .source!=.target and ($scene_ids|index($e.source))!=null and ($scene_ids|index($e.target))!=null) and ($s.channels|map([.source,.target]|sort)|unique|length)==($s.channels|length))},
 {check:"task_start_goal_distinct",applicable:($g|has("task")),pass:(if $g|has("task") then $g.task.start_node!=$g.task.goal_node else true end)},
 {check:"geometry_dimensions_positive",applicable:($geo|has("channel_dimensions_m")),pass:all(($geo.channel_dimensions_m // {})[]; (.length|type)=="number" and (.width|type)=="number" and .length>0 and .width>0)},
 {check:"geometry_metadata_references_exist",applicable:($s|has("geometry")),pass:
   (all(($geo.node_positions_m // {})|keys[]; . as $id | ($ids|index($id))!=null) and
    all(($geo.node_annotations // {})|keys[]; . as $id | ($ids|index($id))!=null) and
    all(($geo.channel_dimensions_m // {})|keys[]; . as $id | ($channels|index($id))!=null) and
    (if $geo.coordinate_system.origin_node then ($ids|index($geo.coordinate_system.origin_node))!=null else true end) and
    (if $geo.evaluation.reference_task then ($ids|index($geo.evaluation.reference_task.start_node))!=null and ($ids|index($geo.evaluation.reference_task.goal_node))!=null else true end))},
 {check:"geometry_bounds_valid",applicable:($geo|has("bounds_m")),pass:(if $geo|has("bounds_m") then $geo.bounds_m.x_min<$geo.bounds_m.x_max and $geo.bounds_m.y_min<$geo.bounds_m.y_max else true end)}
]' "$MAP" "$SCENE" > reports/last_graph_checks.json
jq -e 'all(.[]; .pass==true)' reports/last_graph_checks.json
```

schema 和语义检查缺一不可。没有 geometry/task 时，相应检查标记 applicable=false，不要求照片、坐标或固定任务。场景清单为独立确认的允许连接事实；校验器不能从任意图自己证明真实通道存在。


建议将完整校验块在 Bash 子 shell 中执行并先 `set -e`，或逐条确认退出码。语义明细命令负责生成检查列表，真正的聚合退出码由最后的 jq -e 判定：合法退出 0，规则失败退出 1。不能省略任一 schema 或聚合步骤。

使用四节点样例时，仅将 MAP/SCENE 换为：
`configs/v2/examples/four_nodes.graph.json` / `configs/v2/examples/four_nodes.scene.json`；七节点对应 `seven_nodes.*.json`。不需要修改 schema。

错误样例 scene_file 由 `configs/v2/invalid_cases/manifest.json` 给出。负长度样例使用对应 negative_edge_length.scene.json；它是几何配置错误，不是非法连接。没有 geometry 时不要求尺寸/坐标，也不以显示坐标推导出口方向。

### 4.1 同一路口不同进入方向

```bash
jq --arg node N02 --arg from N01 'def angle: {"east":0,"north":90,"west":180,"south":270}[.];
. as $g |
(first($g.nodes[]|select(.node_id==$node)) // null) as $n |
(first($n.exits[]? | select(.neighbor_id==$from)) // null) as $incoming |
if $n==null or $incoming==null then
 {node_id:$node,incoming_neighbor:$from,entry_known:false,heading_deg:null,exits:[]}
else
 ((($incoming.direction|angle)+180)%360) as $heading |
 {node_id:$node,incoming_neighbor:$from,entry_known:true,heading_deg:$heading,
  exits:[$n.exits[] | . as $x |
    ((($x.direction|angle)-$heading+360)%360) as $delta |
    {neighbor_id:$x.neighbor_id,channel_id:$x.channel_id,absolute_direction:$x.direction,
     relative:({"0":"straight","90":"left","180":"u_turn","270":"right"}[$delta|tostring]),
     enabled:(first($g.edges[]|select(.channel_id==$x.channel_id))|.enabled)}]}
end' configs/v2/graph.json
```

from 是进入当前节点的上一邻居，不是当前朝向字符串。方向是当前节点向外的道路局部切向，不从节点显示坐标相减推断。未知来路返回 entry_known=false / heading_deg=null，不默认朝东。输出包含物理出口及 enabled 状态，u_turn 仅是返回来路的关系，不自动授权掉头。


将 from 改为 N03/N05 可得到对应进入关系。约定 x 向东、y 向北，east=0°、north=90°、west=180°、south=270°，逆时针为正；均为地图轴而非图像轴。exits.direction 是从当前节点向外的局部通道方向。进入朝向取指向来路的方向加 180°，再计算各出口的相对转角，不能直接把指向来路的方向当进入朝向。

物理邻居与启用出口分开，查询 C02 关闭后的可通行邻居：
```bash
jq --arg id N02 '[.edges[]|select(.enabled and (.source==$id or .target==$id))|if .source==$id then .target else .source end]' configs/v2/graph_blocked_C02.json
```
结果为 N01/N05。关闭出口仍可出现在相对结构中，但 enabled=false 不能用于通行；掉头对应不等于掉头授权。关闭 C06 的示例结构合法，但当前任务无可通行路径。

## 5. 环境重建

既有 Ubuntu/ROS 基础必须保留；没有系统 venv/pip 时使用官方 PyPA zipapp。依赖从 PyPI 与官方 PyTorch CPU 索引分别安装，未配置第三方镜像或混合 extra-index。以下重建示例应在新目录执行，不能覆盖当前环境：

```bash
python3 tools/virtualenv.pyz --system-site-packages --app-data .cache/virtualenv .venv_new
.venv_new/bin/python -m pip --cache-dir .cache/pip install --only-binary=:all: -r requirements.txt
.venv_new/bin/python -m pip --cache-dir .cache/pip install -r requirements-cpu.txt
.venv_new/bin/python -m pip check
```

锁定文件包含 CPU torch 本地版本号，因此不能仅用默认 PyPI 索引无条件重装整个锁定文件。先从 requirements-cpu.txt 的官方索引安装指定 torch/torchvision，再按锁定文件对其余本地包安装；系统继承包和 ROS 用环境清单对照。不要将此 CPU 环境误认为 Jetson/TensorRT 部署环境。

## 6. 后续接入

结构、任务和几何用途分离：node_type 描述道路类型；N06 是 dead_end，其目标角色只在 task.goal_node；task 可更换，地图也可不带 task。node_id 是地图身份，不保证仅凭单帧图像能识别。geometry 中的坐标、显示标签、米制尺寸及评价真值不得成为视觉识别输入；地图不要求逐点照片。

若有长度信息，仅在评价层报告米制路径长；拓扑规划无需 geometry，可按通道跳数。scene.channels 是独立确认的物理连接和各端点出口方向事实。修改图和这份清单时需分别核实，不能从待验证图自动生成清单来规避非法连接检查。

v2 仍使用四方向正交、双向简单图，不扩展单向边、平行边或任意角度。格式迁移和完整约定见 reports/SCENE.md；地图加载与测试通过现成工具和临时终端命令完成，未新增导航业务源码。
