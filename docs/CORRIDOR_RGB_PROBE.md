# 6B-1P：只读RGB时序几何探针

此入口分析真实RGB像素，不开车、不读模拟器位姿、不调用结构模型。它是6B-1D之后的**特征和竞争几何解释诊断**，还不是物理几何生产者或可接轮速的通用走廊感知。

## 输入与运行

使用现有虚拟环境/OpenCV，无新增依赖。仿真拼图为256×256 uint8 RGB，四块128×128依次front、left、right、back。分析前三个视向内的跨时刻追踪，不把近同光心四视向当立体相机。固定标定ID `corridor_panel_128_f64_c64_level_v1`：f=64px，c=(64,64)，假定固定近水平安装、无畸变；ID检查不证明标定正确。

先生成只含以下字段的清单，**不能把旧run.json直接交给探针**：

```json
{
  "version": "rgb_probe_sequence_v1",
  "calibration_id": "corridor_panel_128_f64_c64_level_v1",
  "frames": [
    {"frame_id": "capture-0", "captured_sim_s": 0.5,
     "rgb_path": "frames/0000.png", "rgb_sha256": "实际RGB原始像素的64位小写SHA256"}
  ]
}
```

上述是格式示意，示例哈希不是合法运行值。每条PNG必须实际解码为RGB形状，校验其C顺序像素哈希；不以PNG文件编码字节哈希替代。路径必须相对并留在显式RGB根目录内。严格键拒绝audit/pose/scene等附加信息；流式读取保持小内存。帧ID与时间在全序列内不能重复/倒退。

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
.venv/bin/python -m roadvision.corridor_rgb_probe_run \
  --manifest /absolute/path/to/whitelist.json \
  --rgb-root /absolute/path/to/rgb-sequence \
  --output /absolute/path/to/new-output
```

输出目录必须不存在，拒绝覆盖旧结果。每段序列逐帧核对哈希，分析全部相邻三帧窗口；三帧均须在末帧时刻的0.25模拟秒包络内。超时窗记录input_rejected，不放宽窗口。少于三帧明确记无窗口，不虚构观测。

## 算法与拒绝

1. 每视图灰度Shi–Tomasi角点，7px归一化补丁的相似/反相重复检查。只是一种歧义筛查，不能保证排除所有周期或动态纹理。
2. 三帧LK追踪，逐段往返一致性和首尾直达/串接路径一致性；保留完整角点、有效轨迹、残差和拒绝计数。
3. 至少12条追踪、4×4网格至少6格、二维分布，避免只沿走廊一条边拟合。
4. 首尾及相邻单应RANSAC，检查内点支持、空间分布和模型组合一致性；不把外点自动当成障碍。
5. 标定bearing的最佳最小二乘旋转解释，与单应解释竞争。位移太小、旋转残差太小均unknown。高旋转残差也可能来自错误追踪或动态表面，**不是已证明真实车体平移**。
6. 保留单应的所有有限分解及法向方向：ground-compatible、vertical-compatible或oblique。camera y-down为假设地面法向；不挑选一个解为真实地面，不将t/d当米制距离，不用正深度/姿态真值替生产者消歧。

全部开发参数在 `/home/sunrise/toponav_project/docs/plans/2026-10-15-corridor-rgb-probe.md`，实际常量在 `/home/sunrise/toponav_project/roadvision/corridor_rgb_probe.py` 的POLICY。先以解析合成测试开发，随后冻结再读取旧26序列，不按旧审计结果改参数。这些不是实车经过认证的门限。

## 输出解释：诊断不等于物理candidate

每个窗口输出 `diagnostic_status=unknown|planar_flow_candidate`，按front/left/right记录追踪和模型。planar_flow_candidate仅意味着：该视图当前追踪满足开发门限，单应解释优于简单纯旋转解释；不是物理平面身份、米制几何或到达证明。跨视图不作立体匹配，也未联合求解共享刚体运动。

同时生成6B-1D契约包并调用原审查器。物理 `corridor_boundary`、`front_surface` **均为unknown**，横偏/航向/距离均null，appearance也不擅自分类。review始终不授权运动、到达unverified。若诊断有候选但物理输出未知，不是矛盾，而是缺少唯一平面解释、尺度与跨视图约束的实际边界。

`windows.jsonl`逐窗留全部诊断、实际像素哈希和墙钟耗时；`summary.json`分别计诊断候选、unknown、input_rejected、各视图原因。未知/拒绝不能计导航成功或安全率。耗时只表示本机离线处理，不是硬实时验证；窗口重叠，不能当独立样本计算总体成功率。

## 测试和数据纪律

合成测试包含随机纹理平面投影、纯旋转、静止、无纹理、微小位移、重复纹理、亮度变化、遮挡以及协议/路径/像素哈希负例。合成图只作单元诊断，不是训练或独立泛化数据。

旧26序列的审计字段仅存在于独立历史报告；白名单提取后，探针和运行器不接受它们。原25新条件用于本轮开发回归后，不再称修正版的独立新验证。没有重跑物理运动，没有修复旧控制器的三个坏停车；后续需先设计新的只读受控可辨性实验，再决定生产者和控制接线。
