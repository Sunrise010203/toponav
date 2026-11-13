# 6B-1I：原生像素外观与逆透视重复否决，只读实验

用途：诊断H在正常横移/单墙运动上的误拒，而不是替换控制器。新模块`roadvision.rgb_resampling_checks`保留完整H结果，并另给一组实验诊断。没有改动旧冻结文件、17类模型、正式数据、轮速、ROS或拓扑。

## 方法

仍用G的三帧轨迹、H的固定拟合/检查点分组。训练点估计单应，地面逆透视映射与单应合成后，在原生第一幅128px图上比较11px方形patch：目标图只重采样一次，不先把目标拉到地面画布再二次预对齐。原生patch与地面画布patch的物理足迹不同，这是**新描述子策略**，不是数学等价加速。

已匹配NCC≥.70、双向全画布最佳竞争优势≥.06、自相似<.94，数值不降。竞争排除半径11px现为原生像素，不能声称与旧地面画布一样严格。有效支持mask只从全1图经过同一像素映射产生，不是场景分割/模拟器真值；patch和竞争位置必须全足迹有效，填充边缘不能提供匹配证据。无有效竞争搜索区域仍拒绝，不把缺失竞争当成唯一匹配。

地面区域另外保留H原地面画布的自身重复检查，防止透视压缩让重复纹理在原生图上显得唯一；该检查不可用也拒绝。提供去掉该否决的完整诊断消融，**仅供审计，不能用于授权运动**。原图/上部区域没有这个地面额外否决。

过滤后重新执行原数量、空间覆盖、单应、时序组合、纯旋转门槛；不改轨迹坐标。严格共享与锚点残差佐证复用H。内部检查点不参与预对齐或优化，但既有追踪/全区域筛选仍用同窗特征，不是整流水线独立测试。至少一对区域支持不等于所有视图唯一一致。

## 运行

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
.venv/bin/python -m roadvision.rgb_resampling_checks_run \
  --manifest /absolute/public/sequence.json --rgb-root /absolute/public \
  --output /absolute/new-diagnostics
```

新条件整套实验入口：`python -m simulation.rgb_resampling_checks_study --suite configs/evaluation/rgb_resampling_checks_v1.json --freeze /absolute/freeze.json --output /absolute/new-study`。输出目录必须不存在。先运行H与I公共RGB子进程，再读取audit。逻辑输入分隔，不是OS沙箱或随机盲测。

## 不变的限制

所有物理输出unknown/null、不授权运动、到达unverified。同光心四图不是双目。单动态墙和移动相机仍可能完全同RGB；更好的外观匹配不能证明车在动、地面/墙身份、标定正确、米制尺度或到达。隐藏标定不保证检出。

开发已发现：原生footprint可恢复部分横移匹配，却会损失慢速/其他正常分支；单墙仍不足。不得默认替换H或只报新增候选。新种子结果见本轮验收及进度增量。
