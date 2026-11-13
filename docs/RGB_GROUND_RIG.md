# 只读地面追踪/跨视图共享解释原型（6B-1G）

新增模块，不替换冻结RGB探针或现有车辆控制。它不调用17类模型、不接轮速/ROS/节点到达，不生产可授权的物理证据。

## 实现

- front/left/right分别追踪原图与固定地面假设ROI。水平标定下，逆透视映射把地面投影转换为无量纲lateral/height、forward/height；没有输入米制相机高度。128px画布覆盖[-1.5,1.5]×[1.2,4.5]。
- 对地面画布的每段正反向/直接路径独立相位相关，作为LK初值。失败点退出后续LK，不用置零失败坐标参与下一次追踪。输出detected/unique/ROI、六段LK、原图边界、FB/path/retained各阶段计数。
- 回到原图坐标检查FB、三帧直接/链式路径、单应RANSAC、相邻单应组合、位移与纯旋转竞争。误差仍是原始图像px；不靠延长0.25秒证据时效取得更大基线。
- **新覆盖定义**：地面分支按固定逆透视ROI的4×4格计数，至少6格，原图分支按原图格；内点也检查。原图spread≥.1保持。虽保留格数门槛，但这不是冻结探针相同的接受域；评测显式对照，不能称所有原门槛未改变。
- 保留全部单应分解，以已声明四向外参变换到同一rig坐标，穷举互相兼容分支：旋转≤3°，无向平移轴≤12°；至少2视图、法线至少有一对分离≥20°。只保留最大区域支持的竞争组合（最多列64，真实数量和截断标记保留）。输出支持**区域**数，不冒充独立平面数；同一地面跨视图本身不满足多平面条件。
- t/d不同比例、正负、法线和旋转分解仍有歧义；无向轴比较未验证cheirality；未做全局束调整或静态场景证明。共享解释是条件性代数兼容，不是“已测得车体运动”。保留原图/地面分别共享检验消融与完整冻结基线。

共光心相机没有双目深度基线。地面warp区域也可能是墙、重复纹理或混合表面，标签`_ground`只命名假设分支，不是识别结果。隐藏焦距/俯仰失配可能漏检，动态单墙和移动相机仍可产生完全相同RGB。

所有返回的物理corridor/front仍unknown，offset/heading/range为null，motion_authorized=false、arrival_status=unverified。不得把候选数当导航成功数或安全性能提升。

## 运行

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
.venv/bin/python -m roadvision.rgb_ground_rig_run \
  --manifest /absolute/public/sequence.json \
  --rgb-root /absolute/public --output /absolute/new-output
```

使用旧`rgb_probe_sequence_v1`严格公共清单。只接受PNG、实际RGB字节SHA、帧ID/采样时间和标定ID；路径不可越出root；输出必须不存在。保留最多三帧，时间超过包络记录输入拒绝，不放宽门槛。输入不接受场景、位姿、平面、深度、分割或模型标签。

评测入口：`python -m simulation.rgb_ground_rig_study --suite configs/evaluation/rgb_ground_rig_v1.json --freeze /absolute/freeze.json --output /absolute/new-study`。固定旧生成器→旧探针独立子进程→新原型独立子进程→双方全部完成→审计。逻辑分隔而非OS安全沙箱。预测阶段不读取audit；事后用有限可见平面计算跟踪内点的转移误差，不是距离或位姿精度，也不是真实世界标注。

旧20条件用于四次开发尝试，不能算新验证。新种子20条件是冻结后的针对性验证，不是随机盲测，窗口重叠；较快脚本相机位移不是车辆轮速可达性测试。源代码与方法说明冻结后不得依据新结果调参。
