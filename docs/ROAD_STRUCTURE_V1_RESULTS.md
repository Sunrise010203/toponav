# 第四分块：四向道路结构识别第一版实测报告

日期：2026-10-14。状态：**正式数据、冻结训练、独立测试与第三分块图像驱动接入已完成；过渡/遮挡安全拒识仍不足，不能作为实车安全控制验收。**

## 1. 与环境烟测的区别

- 既有 `verify_environment.py`、随机张量和绘图检查只验证环境，不是训练或识别能力结果。
- 本次正式输入是 PyBullet 3.2.7 TinyRenderer 渲染的物理墙体、通道、可见封堵与遮挡物 RGB 图像；没有使用随机/噪声图片冒充训练数据。
- 旧节点概率模拟器、旧工具和已有基线保留，没有覆盖节点模型。权威接口来自虚拟机当前 v2 源码，而不是本地旧交付副本。
- 第一版仅覆盖简化正交通道模拟环境，不是公开真实道路数据集，也不证明跨渲染器或真实场景泛化。各布局使用相同渲染器与基本几何模板。

## 2. 类别与标注

输出固定 17 维：`exits_0000` 至 `exits_1111`，加 `unobservable`。四位依次是 **前、左、右、后**，1 表示该方向观察到开放出口，0 表示明确关闭。节点 ID 不参与分类。

四视图面板 256×256：左上前、右上左、左下右、右下后；每个视图 128×128、90° FOV。模拟世界在四次渲染之间不变化，因此四视图对应同一状态。推理时分别缩放到 224×224，并使用 ImageNet 均值/方差归一化。

- 正式已知类：节点中心附近 ±0.08 m、稳定正交朝向；开放/关闭标签经过物理射线核查，关闭通道具有真实可见墙体或封堵。
- 接近路口、通过/离开、转向、遮挡及缺视图：标为 `unobservable`，不能沿用前一个节点标签。
- 这些阶段和真实位置/朝向仅用于采集标注与审计，模型只看 RGB。**单帧图像未必能唯一判断是否正在通过节点或已经稳定；更强骨干并不自动解决这种可观测性问题。**
- `exits_0000` 是四方向全封闭，不等于普通单出口尽头；出口结构不是节点身份、目标到达、道路宽度、距离或通行安全授权。

## 3. 数据隔离与训练

| 布局 | 节点数 | 划分 | 每类样本数 | 总图像数 |
|---|---:|---|---:|---:|
| train_grid | 9 | train | 40 | 680 |
| train_bypass | 12 | train | 40 | 680 |
| val_branch | 16 | val | 16 | 272 |
| test_locked | 15 | test | 24 | 408 |

共 2040 张四向面板，每布局 4 个采集分组，整布局及采集分组不跨 split。布局节点数/边数及拓扑指纹不同。路径、图片 SHA256、清单 SHA256 与跨划分重复文件检查通过；烟测预览不能用于正式训练。

最终测试图片未参与训练特征提取、模型选择或阈值选择；正式测试前已封存模型和配置。采集器生成/标注最终测试图像不属于训练或调参。

- 共享官方预训练 `MobileNetV3-Small.features` 对四视图分别提取 576 维特征，拼接为 2304 维，接 Dropout(0.1)+17 类线性层。
- 冻结骨干参数和 BatchNorm，骨干运行统计与官方预训练张量逐项相等；不是从头训练，也没有微调骨干。
- AdamW，学习率 0.003，weight_decay 0.0001，分类层 batch=128，共 80 轮，固定随机种子 20261014。按验证交叉熵选择第 36 轮。
- 特征提取 batch=32；RTX 5070 Laptop GPU / 8GB，PyTorch 2.10.0+cu128，Python 3.11.9。训练阶段实测 8.49 s（其中特征提取 7.3 s；不含环境安装、权重下载和采集）；峰值 GPU allocated 334.25 MiB，不等于整进程或驱动显存总量。
- 验证集总体 263/272=96.69%，已知结构 256/256=100%；unknown 正确 7/16，错误接受 5/16，说明安全拒识不足。未以此宣称完整安全性，也未根据最终测试重新训练。
- 拒识阈值固定 0.6，来自第三分块既有下限；未在最终测试上选择阈值。Softmax 是未校准分数，不保证等于正确率。

封存标识：

```text
manifest SHA256: 1957cf5bd8ae49cd3551e12b0e431cf991d5369dd6dc272c64d7f346a72d2181
model.pt SHA256: 1188f35dd20d0773b780b73f2000daa25fede9b2db098b02dc0746ad0de6b03b
```

## 4. 独立测试：必须分开看结构识别与拒识

CPU 与 CUDA 的 408 个预测类别和接受/拒绝标记均一致。

| 指标 | 结果 |
|---|---:|
| 17 类总体准确率 | 390/408 = **95.59%** |
| 已知结构准确率 | 381/384 = **99.22%** |
| unknown 原始类别识别率 | 9/24 = **37.50%** |
| 接受覆盖率 | 389/408 = 95.34% |
| 被接受样本准确率 | 381/389 = 97.94% |
| 拒绝样本数 | 19 |
| unknown 错误接受 | **8/24 = 33.33%** |

每类支持数均为 24。原始混淆矩阵不把硬性缺视图拒绝算成模型分类正确；拒识指标另外计算。全部 17×17 混淆矩阵保存在 `confusion_matrix.csv` 与 `test_cpu.json` / `test_cuda.json`；各类召回率在 `per_class_metrics.csv`。每张测试图像的完整概率保存在 `test_predictions_cpu.csv` 和 `test_predictions_cuda.csv`。

unknown 分组（样本很少，不能据此声称统计稳定）：

| 阶段 | 样本数 | 原始分类正确 | 错误接受 |
|---|---:|---:|---:|
| 缺视图 | 8 | 2 | 0 |
| 接近 | 3 | 3 | 0 |
| 遮挡 | 5 | 0 | **5** |
| 通过/离开 | 5 | 2 | 2 |
| 转向 | 3 | 2 | 1 |

尤其是遮挡：该测试分组 5 张全部被错误接受。**阈值、unknown 类和缺帧检测不足以保证所有不完整视野都会被识别并停止。** 不应把总体准确率掩盖这一问题。下一轮应优先丰富遮挡/过渡数据并考虑时序观测有效性，在新的封存测试布局重新验收；不能用本轮最终测试反向优化并继续声称独立测试。

## 5. 延迟实测

batch=1 是一个四向样本；每样本 4 张视图。预热 5 次后测量 50 次，GPU 前向计时有 CUDA 同步。加载单独计时，不包含采集/渲染、图片磁盘读取、SSH、通信或执行器耗时。

| 设备及范围 | 平均 ms | P50 ms | P95 ms |
|---|---:|---:|---:|
| RTX 5070 Laptop GPU，纯前向 | 4.17 | 3.93 | 5.76 |
| RTX 5070 Laptop GPU，预处理+前向+输出适配 | **7.56** | 7.31 | 9.75 |
| VM CPU，纯前向 | 12.93 | 12.93 | 16.38 |
| VM CPU，预处理+前向+输出适配 | **15.80** | 16.10 | 18.64 |

VM：Ubuntu / Python 3.10.12 / PyTorch 2.6.0+cpu，2 vCPU，宿主 CPU Intel Core Ultra 9 275HX，约 3.8 GiB RAM，PyTorch 2 线程。模型加载 GPU 291.60 ms，VM CPU 45.43 ms，跨设备的加载条件不同，不能直接推断优劣。

本机 CPU 与 VM CPU 同一样本概率最大差异 4.77e-7；CPU/CUDA 的 408 张图像概率最大差异约 0.02020，但类别及接受标记相同。不同运行时的概率不能声称逐位一致。

## 6. 输出与第三分块接入

`Predictor.predict` 输出 frame_id、class_id、固定 class_ids、17 维 probabilities、captured_at_ns、predicted_at_ns、inference_ms、frame_age_s、accepted、rejection_reason 及嵌套 observation。旁路包示例保存在 `example_prediction.json`。

第三分块只收到严格接口，例如：

```json
{
  "relative_open": {"front": true, "left": true, "right": true, "back": true},
  "confidence": 0.972804605960846
}
```

- 分数低于 0.6、预测 unknown、任一全黑缺失视图、帧超过 1 秒或时间位于未来：四方向均 null、confidence=0；保留候选并停止，不把未知当作关闭。
- 当前缺视图硬检测只覆盖全黑缺帧；其他遮挡/过曝/不完整依赖模型，实测有漏检。
- 时间戳使用同一机器 Unix 纳秒墙钟；耗时使用单调时钟。真实运行必须传入真实采集时刻。离线类别评估不因历史图片天然过期而统计拒识，在线驱动会检查新鲜度。
- 相机同步、顺序和完整性由本次模拟采集规范保证；不是对任意外部四相机输入已实现同步校验。

新增独立 `roadvision.navigation` 驱动复用原第三分块，**不调用 `StructuralEnvironment.observe()` 理想结构传感器**，也不把模型错误时的真值作为回退。模拟真值仅供相机站位与事后审计；定位和决策仅接收地图假设、任务先验、严格 observation 和执行状态。

原基线仍可单独运行。要运行新图像驱动，在 VM 使用：

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m roadvision.navigation \
  --run /home/sunrise/toponav_project/models/road_structure_frozen_v1_20261014 \
  --layout test_locked \
  --output /home/sunrise/toponav_project/reports/roadvision-new-run.json
```

输出文件必须不存在，防止覆盖。重跑只是复现，不能作为新的独立测试。

## 7. 图像驱动导航与回归

预先定义的测试任务：测试布局每个节点、四种朝向，目标取节点序号向后偏移 7 并循环，共 15×4=60 个任务；不是挑选成功路线后报告。

- 真实到达审计 **60/60=100%**；观测接受率 100%；整套运行约 11.01 s。全过程日志在 `navigation-test-locked.json`。
- 缺视图与过期帧两个故障运行均 `observation_uncertain` → stop，保留 R000、0° 候选，没有移动。
- 单元测试独立验证低分拒绝、unknown 拒绝、维度固定、骨干冻结、物理标注、划分隔离、严格字段、禁用理想传感器。
- 本机：13 passed、3 skipped（Windows 没有 PyBullet / VM simulation 模块）；VM 新测试全部 16 passed。
- VM 全回归：**545 passed**，原有 18 个绘图依赖弃用警告，19.43 s。

**60/60 的条件限制**：任务提供正确起点和确认朝向；每次动作是已有模拟器的完整原子转向+通道遍历；观测在稳定节点站位采集。测试没有覆盖连续行驶时任意观测、任意未知起点、运动控制误差、真实摄像头噪声。不能由此推导未知场景或实车导航成功率 100%。

原核心文件 SHA256 保持：

```text
simulation/navigation.py
fb53d4831618da5e85584ba67ba1fe01e9f9110be44326abfda8c37aeade5822
simulation/structural_localization.py
c57115650c503fd27a1c4bfa2c4b2e03ff78ad3d9041b5608eef1c63c7de3ff1
simulation/structural_navigation.py
7d2594c6e3b851b05a9e90b235521ff2217780edf6d9ddcd2515e056600910e0
```

## 8. 文件位置与边界

本机根目录：`D:\大学资料\大二上\科研项目\local_vision`

- 代码：`D:\大学资料\大二上\科研项目\local_vision\roadvision`
- 数据：`D:\大学资料\大二上\科研项目\local_vision\datasets\road_structure_v1_20261014`
- 模型、测试指标、概率和审计：`D:\大学资料\大二上\科研项目\local_vision\runs\road_structure_frozen_v1_20261014`

VM 根目录：`/home/sunrise/toponav_project`

- 代码：`/home/sunrise/toponav_project/roadvision`
- 数据：`/home/sunrise/toponav_project/datasets/road_structure_v1_20261014`
- 模型与实测：`/home/sunrise/toponav_project/models/road_structure_frozen_v1_20261014`
- 本报告：`/home/sunrise/toponav_project/docs/ROAD_STRUCTURE_V1_RESULTS.md`

数据约 4.87 MiB；模型和全部当前结果约 5.3 MiB，环境/预训练下载/安装包不计入其中。保留 manifest、类别/预处理 metadata、model.pt、指标和采集代码才能复现；本次没有删除训练数据或旧基线。

没有使用大模型作为在线识别器，也没有以结构分类替代通道几何估计、连续定位、完整控制或实车安全保障。
