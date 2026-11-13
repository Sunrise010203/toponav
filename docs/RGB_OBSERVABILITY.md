# 6B-1O：受控RGB可辨性实验

这是冻结6B-1P探针的只读边界实验，不是新的识别模型或车辆控制器。继续使用PyBullet DIRECT/TinyRenderer，创建带固定UV纹理的视觉平面，脚本设置相机/动态表面的采样位姿，输出四向RGB。**没有机器人、轮电机、碰撞几何或导航执行器。** 不能据此声称原车已能达到实验中的相机速度、正确避障或完成驾驶。

## 问题与对照

旧低纹理画面全部unknown后，先分开验证信息和算法的限制：

- 单立面/单地面与多平面房间：单应假设在哪些画面中得到支持？
- 确定性独特纹理、重复棋盘和均匀表面：有角点是否就意味着可追踪、可唯一匹配？
- 静止、纯旋转、微小平移、较大平移：不同图像运动是否能被现有门槛区分？
- 同灰色地面贴片与竖直障碍：材质/颜色相近不意味着几何身份相同。
- 静止相机与运动墙：图像流不能自动证明车体平移。
- 显式错误标定ID，以及隐藏实际FOV/俯仰不符：前者是协议检查；后者不能假装已由ID验证解决。

预声明清单：`/home/sunrise/toponav_project/configs/evaluation/rgb_observability_v1.json`。
固定20条件、每段5帧、0.1秒间隔，每个合法序列3个相邻三帧窗口。详细场景、纹理、轨迹和评价规则见 `/home/sunrise/toponav_project/docs/plans/2026-10-15-rgb-observability.md`。相机运动是信息探测条件，较大基线不遵守原车0.35m/s控制包络，不将其包装成原车性能。

## 数据流：先预测，后审计

```text
预声明场景/相机脚本 → PyBullet RGB生成
                         ├─ public/：PNG + 严格RGB序列清单
                         │       ↓ 独立Python子进程
                         │   冻结roadvision.corridor_rgb_probe_run
                         │       ↓ 全部预测结束
                         └─ audit/：相机/表面真值 → 独立事后评价
```

预测子进程只收到 `--manifest`、`--rgb-root`、`--output`，不导入模拟生成器，也不接收场景、运动、深度、分割、节点或审计字段。场景ID只在父进程命名报告目录。这里是可检查的逻辑输入隔离，不是OS访问控制安全沙箱或独立机构盲测。

生成的像素哈希逐图绑定白名单，纹理贴在真实平面上、跨帧固定，不在屏幕上重新随机生成噪声。每段public与audit分开保存。全部子进程结束后写phase-complete，再由审计器读取真值；真值不会用于选点、追踪、阈值调整或放行。

## 运行

使用现有虚拟环境和依赖；只允许新输出目录：

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
export MPLBACKEND=Agg MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib
.venv/bin/python -m simulation.rgb_observability_run \
  --suite configs/evaluation/rgb_observability_v1.json \
  --freeze reports/rgb-observability-20261015-BTzTFV/freeze.json \
  --output /absolute/path/to/new-study
```

生成模块：`/home/sunrise/toponav_project/simulation/rgb_observability.py`。
协调/审计：`/home/sunrise/toponav_project/simulation/rgb_observability_run.py`。

输出：generated/cases下的全部RGB、纹理/网格、脚本真值；predictions下每个子进程stdout/stderr及原探针summary/windows；audit-result.json汇总全部条件，包含显式ID拒绝、退化条件候选和单平面投影误差。输入拒绝也保留日志，不换ID重跑成“通过”。

## 如何理解结果

- 诊断 `planar_flow_candidate` 不是地面/墙体身份、车体平移、距离或到达。单应分解依然保留多解，不将t/d当米制平移。
- 在单平面已知真值的参考条件中，可以事后比较预测单应的像素转移与几何投影；这不是位姿误差、米制距离精度或在线可用真值。无估计记NA；多平面不强行套一张真值单应。
- 当前审计使用FOV投影的栅格模型f=64/tan(FOV/2)、c=(63.5,63.5)；冻结探针名义c=(64,64)。亚像素约定/渲染差异必须保留，不声称实现了真实相机标定验证。
- 静止/纯转/微视差/重复纹理条件出现诊断候选时，单列为退化拒绝的边界，不掩盖。动态表面或隐藏标定失配有候选，也不能提升为物理几何。
- 当前物理通道/前方表面仍unknown，横偏/航向/距离null，运动不授权、到达未验证。原控制器的误停车和错误居中没有被修复。
- 窗口相关，分别报告条件/帧/窗口/视图分母；这是针对性受控实验，不是总体泛化或导航成功率。新RGB/纹理只作实验资料，不进入正式训练集、不重训。

首次运行前冻结算法、实验代码、20条件清单和参数。看结果后不换场景/降阈值。完整验收入口：`/home/sunrise/toponav_project/reports/rgb-observability-20261015-BTzTFV/ACCEPTANCE.md`。
