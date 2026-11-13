# 普通直走：固定梯度通行描述子实验

2026-11-01。承接`wide_reference`，只研究当前RGB通行补丁的竞争匹配，S2未通过/S6未完成状态不变。不改模型、相机/标定、速度、制动、授权和恢复预算。

## 定位与方案

对23个已暴露公共RGB时刻检查：关闭光流金字塔没有改变任何通行状态；原生未整流直接跟踪的22个时刻中，原来11个可用状态有9个反而不可用。因此这两个探测均不接入控制，没有根据目标帧失败反复尝试求通过。

固定3×3 Sobel x/y描述子在同一23个已暴露时刻上，保留了原11个可用状态，4个灰度竞争匹配的未知状态得到梯度支持，其余8个仍未知。这个定向、已暴露诊断不是独立正确率或清障证明。

实验采用一个预先固定的描述子，而不是按每帧是否通过在灰度/梯度间投票：

1. 完整执行一次原`assess_free_ground()`，保留相同九个中心、每格原生参考图选点、真实帧/运动绑定和光流测量。
2. 将已进行地面整流的第一张灰度图和最新原生灰度图分别计算固定3×3 Sobel x/y，作为三通道`(gx,gy,0)`图；逐通道去均值的NCC与OpenCV多通道竞争匹配保持一致。
3. 梯度11×11补丁额外需要一像素导数边界支持（有效原生足迹13×13），缺失、填充或平坦不能供证。竞争搜索仍是完整有效画布、双向匹配和两张图的自重复检查，不只搜局部。
4. **每格强制**梯度NCC≥0.70、竞争margin≥0.06、自相似<0.94。同时保留原生参考有效、原生NCC≥0.70、原生自重复<0.94、剩余光流≤0.5px及往返≤0.25px的否决。
5. 九格全部支持才保持原0.6m条件停车净空；缺少任何一格、错误绑定或低基线均未知。

**替换的是灰度竞争margin这一描述子判据，不是“仍要求原灰度margin通过”。相同数值不意味着同一接受集合或安全单调增强。** 灰度检查通过但梯度失败也要停车，梯度不能救回原光流、平坦、重复或外观改变否决；没有OR回退、目标图换点、重拟合或放宽门槛。

## 接入范围

新增`roadvision/gradient_ground_path.py`和`roadvision/gradient_straight.py`。显式`gradient_transfer`继承`wide_reference`的启动、几何及全部控制，仅覆盖通行检查钩子；默认`baseline`、`reference`、`wide_reference`仍调用原函数。公共任务/回放严格绑定变体，禁止混入转角/路线；`promoted=false`。

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m simulation.continuous_straight_run \
  --output /home/sunrise/toponav_project/reports/my-gradient-transfer-NEW \
  --straight-variant gradient_transfer --length-m 6 --seed 313 \
  --confirm-initial-clearance --motion-permission bounded_probe
```

输出必须全新；省略授权或净空确认仍拒绝运动。这是开发实验，不是默认启用，也不保证给定案例完成。

## 未解决的限制

梯度会放大噪声，也可能因光照、渲染/插值、低纹理和表面变化拒绝正确对应。它不是深度或语义障碍检测；相机水平/已标定、场景刚体静态、地面水平和最小障碍尺寸仍是未认证先验。剩余光流/地面几何失败仍会停车，不能用梯度高分覆盖它们。完整车旁近场扫掠盲区没有修复，远处九个点仍不代表完整区域覆盖；不启用盲走和主动恢复，不延长期限。

冻结条件、物理闭环、负例、公共回放、当前/历史测试及结论见
[本轮报告](/home/sunrise/toponav_project/reports/straight-followup-20261101-03/REPORT.md)。
