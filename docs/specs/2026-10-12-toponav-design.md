# 六节点拓扑导航环境与地图设计

用户已同意建立专用项目目录并先采用六节点场景。保留原要求：不新增项目 Python/ROS 源码；仅配置环境、地图、schema、文档与验收数据，执行现成工具命令。

远程根目录：`/home/sunrise/toponav_project`。复用 Ubuntu 22.04 / Python 3.10 / ROS 2 Humble。项目内 `.venv` 配置 CPU 科学计算、OpenCV、PyTorch/MobileNetV3、ONNX/ONNX Runtime、图规划、数据分析及校验测试依赖。为接入 apt 安装的 ROS Python 包，环境允许读取 system-site-packages；第三方升级仅写入 `.venv`，不改系统包、shell 启动文件或 SSH 配置。NumPy 固定 1.26.4，避免 ROS cv_bridge 的 NumPy 2.x ABI 问题。

场景为二维室内环路加入口与目标支路：N01-N02、N02-N03、N03-N04、N04-N05、N05-N02、N04-N06。所有通道双向、长 2m、宽 0.8m。坐标仅作场景设计，不代表实地测量。起点 N01，目标 N06。主路与备选路均长 8m。动作使用绝对出口方向 east/north/west/south，按 neighbors 的索引对齐；相对左/右/直行必须结合小车进入朝向，不能由固定节点动作直接推断。

`scene.json` 为设计场景的独立通道事实；`graph.json` 为拓扑邻接表与边列表；`graph.schema.json` 为该六节点场景专用结构/允许连接约束。schema 校验之外，用现成 jq 命令核验唯一 ID、互邻、邻接与边一致、通道事实、方向和可达性；命令记入 README，不保存自编验证程序。

验收：全部依赖实际导入；ROS/rclpy/cv_bridge 接入通过；MobileNetV3 CPU 前向与 ONNX 推理烟测通过（不下载预训练权重）；地图读入 6 节点 6 边；查询 N02/N04 得到正确邻居；合法图通过校验；错误样例非零退出且报告对应规则。保存版本锁定文件、验收报告、图示和日志。

边界：不是 Gazebo 世界，不训练模型，不创建导航业务代码，不声明完成实车部署或实车验证。GPU/CUDA/TensorRT/Jetson SDK 仅在后续明确平台时配置。
