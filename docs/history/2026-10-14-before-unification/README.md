# 先验拓扑图引导的智能小车视觉导航：环境与地图

远程项目：`/home/sunrise/toponav_project`。日期：2026-10-12（Asia/Shanghai）。

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

- `configs/scene.json`：独立的场景通道事实、坐标、起终点与设计路线。
- `configs/graph.json`：正式拓扑地图，6 个节点、6 条双向边。
- `configs/graph.schema.json`：该六节点场景专用结构及允许端点校验，不是可直接用于任意场景的 schema。
- `configs/graph_blocked_C02.json`：C02 暂时关闭的合法地图示例。
- `configs/invalid_cases/`：13 个故意非法样例，仅作测试，不能作导航输入。
- `reports/topology.png`、`reports/topology.svg`：二维场景/拓扑示意图。
- `reports/SCENE.md`：节点、通道、起终点、方向与场景边界说明。
- `reports/acceptance.json`、`reports/ACCEPTANCE.md`：最终验收汇总。
- `reports/graph_checks.json`、`reports/neighbors.json`、`reports/invalid_cases/`：可审计检查结果。
- `requirements.txt`、`requirements-cpu.txt`：依赖要求。
- `requirements.lock.txt`：项目内安装包锁定；另见 `reports/environment.json` 与系统依赖清单了解继承部分。
- `data_raw/`、`data_processed/`、`simulation/`、`models/`、`edge_deploy/`：预留目录，目前没有真实视频、训练模型或自编仿真/控制程序。
- `ros2_ws/src/`：预留 ROS 2 工作空间，不添加或编译业务包。
- `logs/`：安装、工具校验和环境烟测日志。
- `docs/`：本次设计与执行计划。
- `tools/virtualenv.pyz`：官方 PyPA 环境引导工具，非自编项目代码。

## 3. 读取地图和查询邻居

```bash
cd /home/sunrise/toponav_project
jq '{scene_id,start_node,goal_node,node_count:(.nodes|length),channel_count:(.edges|length)}' configs/graph.json
jq --arg id N02 '.nodes[] | select(.node_id==$id) | {node_id,neighbors,actions}' configs/graph.json
jq --arg id N04 '.nodes[] | select(.node_id==$id) | {node_id,neighbors,actions}' configs/graph.json
```

N02：N01/N03/N05；N04：N03/N05/N06。未知节点查询用下面命令，找不到时退出码为 4，而不是默认为合法节点：

```bash
jq -e --arg id N99 '.nodes[] | select(.node_id==$id) | .neighbors' configs/graph.json
```

## 4. 完整合法性检查

先做 schema，再做跨节点/跨文件一致性检查；单纯 JSON 可解析不代表连接合法。schema 不能替代互邻、唯一 ID 与可达性检查。校验器不使用自编源码文件，下列命令直接交给已有 CLI。

```bash
cd /home/sunrise/toponav_project
MAP=configs/graph.json
SCENE=configs/scene.json
.venv/bin/check-jsonschema --schemafile configs/graph.schema.json "$MAP"
jq -s '.[0] as $g | .[1] as $s |
($g.nodes | map(.node_id)) as $ids |
[
 {check:"unique_node_ids",pass: (($ids|unique|length)==($ids|length))},
 {check:"scene_node_set",pass: (($ids|sort)==($s.nodes|map(.node_id)|sort))},
 {check:"start_goal_match_scene",pass: ($g.start_node==$s.start.node_id and $g.goal_node==$s.goal.node_id and $g.start_node!=$g.goal_node)},
 {check:"neighbor_action_alignment",pass: all($g.nodes[]; (.neighbors|length)==(.actions|length))},
 {check:"neighbors_exist",pass: all($g.nodes[].neighbors[]; . as $id | ($ids|index($id))!=null)},
 {check:"no_self_connections",pass: all($g.nodes[]; .node_id as $id | (.neighbors|index($id))==null)},
 {check:"unique_neighbors",pass: all($g.nodes[]; (.neighbors|unique|length)==(.neighbors|length))},
 {check:"reciprocal_neighbors",pass: all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($g.nodes[]; .node_id==$v and (.neighbors|index($n.node_id))!=null)))},
 {check:"neighbors_have_physical_channel",pass: all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($s.channels[]; (.source==$n.node_id and .target==$v) or (.source==$v and .target==$n.node_id))))},
 {check:"neighbors_have_map_edge",pass: all($g.nodes[]; . as $n | all($n.neighbors[]; . as $v | any($g.edges[]; (.source==$n.node_id and .target==$v) or (.source==$v and .target==$n.node_id))))},
 {check:"edge_endpoints_exist",pass: all($g.edges[]; . as $e | ($ids|index($e.source))!=null and ($ids|index($e.target))!=null)},
 {check:"no_self_edges",pass: all($g.edges[]; .source!=.target)},
 {check:"unique_channel_ids",pass: (($g.edges|map(.channel_id)|unique|length)==($g.edges|length))},
 {check:"unique_undirected_edges",pass: (($g.edges|map([.source,.target]|sort)|unique|length)==($g.edges|length))},
 {check:"edges_match_physical_channels",pass: all($g.edges[]; . as $e | any($s.channels[]; .channel_id==$e.channel_id and (((.source==$e.source and .target==$e.target) or (.source==$e.target and .target==$e.source))) and .length_m==$e.length_m and .width_m==$e.width_m and .bidirectional==$e.bidirectional))},
 {check:"physical_channel_set_complete",pass: (($g.edges|map(.channel_id)|sort)==($s.channels|map(.channel_id)|sort))},
 {check:"edges_have_reciprocal_neighbors",pass: all($g.edges[]; . as $e | any($g.nodes[]; .node_id==$e.source and (.neighbors|index($e.target))!=null) and any($g.nodes[]; .node_id==$e.target and (.neighbors|index($e.source))!=null))},
 {check:"exit_directions_match_coordinates",pass: all($g.nodes[]; . as $n | all(range(0; $n.neighbors|length); . as $i | first($g.nodes[]|select(.node_id==$n.neighbors[$i])) as $v | ($v.position_m[0]-$n.position_m[0]) as $dx | ($v.position_m[1]-$n.position_m[1]) as $dy | $n.actions[$i]==(if $dy==0 and $dx>0 then "east" elif $dy==0 and $dx<0 then "west" elif $dx==0 and $dy>0 then "north" elif $dx==0 and $dy<0 then "south" else "invalid" end)))},
 {check:"all_nodes_reachable_from_start",pass: (((reduce range(0; $ids|length) as $iteration ([$g.start_node]; . as $seen | ($seen + [$g.nodes[] | select(.node_id as $id | $seen|index($id)) | .neighbors[]])|unique))|sort)==($ids|sort))},
 {check:"goal_reachable_from_start",pass: (((reduce range(0; $ids|length) as $iteration ([$g.start_node]; . as $seen | ($seen + [$g.nodes[] | select(.node_id as $id | $seen|index($id)) | .neighbors[]])|unique)) | index($g.goal_node))!=null)},
 {check:"declared_routes_follow_map_edges",pass: all($s.routes[]; . as $route | $route[0]==$g.start_node and $route[-1]==$g.goal_node and all(range(0; ($route|length)-1); . as $i | any($g.edges[]; ((.source==$route[$i] and .target==$route[$i+1]) or (.target==$route[$i] and .source==$route[$i+1])))))}
]' "$MAP" "$SCENE" > reports/last_graph_checks.json
jq -e 'all(.[]; .pass==true)' reports/last_graph_checks.json
```

合法地图：schema 成功，21 项检查全部 true，最后命令退出 0。非法地图：schema 或 jq 聚合检查退出非零；检查明细的 `check` 字段定位原因。将 MAP 换成 `configs/invalid_cases/asymmetric_connection.json` 可看到 `reciprocal_neighbors` 和 `edges_have_reciprocal_neighbors` 失败。

schema 层失败时仍可单独运行 jq 层取得更具体的诊断。jq 明细生成命令只负责输出每项结果，真正判定退出码的是紧接着的 `jq -e 'all(...)'`；不能省略。

结构校验把 `enabled=false` 视为合法关闭状态，不保证任意关闭组合都可行驶。实际可通行邻居可这样查询：

```bash
jq --arg id N02 '[.edges[] | select(.enabled and (.source==$id or .target==$id)) | if .source==$id then .target else .source end]' configs/graph_blocked_C02.json
```

预期：N01/N05，不含 N03。后续规划须使用启用边，不可直接使用物理邻居列表。

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

地图使用指导书的 `node_id/node_type/neighbors/actions/reference_images` 字段，并增加场景 ID、坐标和边数据。相邻数组方向一一对齐，动作是绝对方向，不是固定左/右转。规划和视觉识别接入属于后续编码任务，本次没有实现控制闭环。新场景需同步更新 scene、graph、schema 和测试，不应盲目复用六节点专用约束。

## 7. v2 地图配置审查（2026-10-14）

已确认的通用 schema、task/exits/geometry 字段迁移在 configs/v2 中，配置和工具层验收通过。默认 configs 暂保留旧版，以免破坏当前 simulation 程序的旧字段读取。

v2 精确使用命令见 docs/topology-v2-usage.md，字段约定见 reports/topology_v2/FIELDS.md，验收见 reports/topology_v2/ACCEPTANCE.md。现有 dijkstra/navigation/structural_localization/path_planning/structural_navigation 尚未适配 v2；其中规划出口还依赖 position_m，需要经授权改为 exits 属性，不能用复制几何真值回旧字段来伪装适配完成。
