# 直行：已接受实现的完整正横偏热点复核

2026-11-04，Asia/Shanghai。

**本轮完成诊断，没有修改生产、工具或测试源码，没有新增/接入优化候选。**
当前默认保留此前已接受的成对总体复用、精确局部排除几何、原生补丁对比度快速判定。
完整80条正横偏公共决策，阶段/工作量与cProfile/工作量各一遍，**160条完整序列化输出
全部精确一致**，每tick两遍工作量一致。未运行普通计时、预热、AB比较或条件回放。

## 结论与下一步

- **全画布模板匹配仍为最大单项**：本次阶段仪器排他时间占 **46.97%**，每遍
  **126105次**实际匹配；114756次在成对描述子内（91.00%），11349次在配对外。
- H证据、I证据排他时间分别占11.77%、10.20%；成对描述子父级排他占7.67%。
  H/I准备合计排他约0.287s，占0.67%，不是当前优先方向。
- 当前快速路径确实执行：H/I新入口各28732次，contrast gate/integral2各114842次，
  local exclusion outside为84223次。不是由于观察器或显式过滤器身份回退而测到旧实现。
- 下一步优先做**匹配实现的精确等价可行性检查**，先检查原生输入表示/后端是否能
  保持完整分数图逐位相同；任何分数差异都不能用容差、只看最终接受结果或放宽阈值
  掩盖。本轮尚未实施该检查，也没有证明存在可用替换。
- 不重接已拒绝的scratch/patch-NCC缓存/求逆复用/masked-max方案。后续若形成候选，
  仍须中心、默认、正横偏三条完整序列的总wall中位和均值全部严格改善；门槛不变。

**本轮不是提速对比。** 新仪器结果不能与历史剖析或普通计时相减，不能把差值归因于
某一项已接受优化，也不能据此声称达到10Hz、直行通过或导航完成率提高。

## 事前计划、冻结与执行

- [计划](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/PLAN.json) 固定完整80条、两种仪器各一次、普通计时/预热均0，
  focus_ticks为空，未挑帧替代全序列；没有训练/留出变化。
- 公共输入为既有正横偏 `gradient_transfer` 实验记录，未推广为默认策略。严格加载器
  将公共任务、原顺序RGB和时间传入控制器；真值、结果或物理审计不参与决策。
- [原始冻结](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/FREEZE.json) 221文件、[实际执行冻结](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/FREEZE-v2.json)
  228文件。v2只补充启动配置修正及首次失败工件pin，原冻结不覆盖，生产源码未变化。
- OpenCV单线程，其他线程环境不调整，主机非独占。进程CPU统计含进程内所有线程。
  stage先、cProfile后，各完整一遍、新控制器；全部输出/RGB hash/源码pin核对。
  退出后别名与OpenCV线程均恢复。
- 观察器尺寸计数、探针和cProfile有开销；控制器外的解码与输出比较不计入仪器调用。
  cProfile中的JSON编码等部分是工作量观察器开销，不能归为生产证据计算或直接扣除。
- 复用工具SUMMARY中的旧措辞“Current restored implementation”没有改动；它是历史
  固定文字，**本轮没有恢复或撤回源码，执行的是当前已接受实现**。

## 当前阶段归因

该完整阶段遍仪器wall合计 **43.152997s**，CPU合计
146.604942s，未覆盖wall 0.063215s。
排他占比分母仅为本遍仪器wall；包含时间相互嵌套，不能求和。探针开销仍可能落入父级。

| 阶段 | 调用 | 包含wall(s) | 排他wall(s) | 排他占比 |
|---|---:|---:|---:|---:|
| template_matching | 126105 | 20.267 | 20.267 | 46.97% |
| h_patch_evidence | 28732 | 22.740 | 5.077 | 11.77% |
| i_appearance_evidence | 29371 | 4.831 | 4.402 | 10.20% |
| paired_descriptors | 459 | 31.542 | 3.309 | 7.67% |
| nonlinear_solver | 710 | 2.628 | 2.628 | 6.09% |
| metric_ground_fit | 82 | 2.425 | 2.257 | 5.23% |
| free_path | 72 | 4.420 | 1.507 | 3.49% |
| region_models | 1249 | 1.275 | 1.275 | 2.95% |
| optical_flow | 2896 | 0.734 | 0.734 | 1.70% |
| roi_tracking | 459 | 1.519 | 0.318 | 0.74% |

全部阶段、更新前状态、更新后状态/原因和连续四个20tick区段均保存在
[完整分析](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/ANALYSIS.json)，不只保留热点帧。

| 更新前状态 | 决策数 | 仪器wall(s) | 实际匹配数 |
|---|---:|---:|---:|
| bootstrapping | 6 | 0.595 | 1709 |
| following | 55 | 31.723 | 92502 |
| observing | 3 | 0.356 | 854 |
| waiting_evidence | 16 | 10.480 | 31040 |

waiting_evidence的16次更新仍做31040次匹配，不能把“等待证据”误当成无需视觉计算，
也不能靠减少重确认、恢复或证据检查换速度。

## 工作量和独立cProfile

每遍459个配对区域、28732条输入轨迹，全部走native区域。三视图各153次区域调用，
front/left/right输入轨迹4738/11734/12260，配对内匹配18804/46920/49032。
每次匹配输出118×118，完整序列累计 **1755886020个分数位置**；这是累计工作量，
不是同时驻留内存。搜索画布、竞争范围、H/I独立判断和支持/有限性检查都保留。

cProfile是另一遍仪器执行，总self时间 48.588772s，不是普通时延。
下表的包含时间相互嵌套，不可求和；与阶段仪器时间也不可当作before/after。
函数源文件定位保存在二进制prof与[函数明细](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/profile/CPROFILE.json)。

| 文件名::函数 | 调用 | self(s) | 包含(s) |
|---|---:|---:|---:|
| ~::<matchTemplate> | 126105 | 17.546 | 17.546 |
| paired_descriptor_checks.py::population | 114192 | 0.354 | 2.008 |
| paired_descriptor_checks.py::reduce | 228384 | 1.788 | 2.607 |
| local_exclusion_population.py::outside | 84223 | 1.075 | 1.584 |
| native_patch_contrast.py::prepared_patch | 114928 | 0.934 | 2.348 |
| native_patch_contrast.py::_contrast_gate | 114842 | 0.198 | 0.434 |
| rgb_motion_checks.py::_ncc | 63729 | 0.558 | 2.239 |
| corridor_rgb_probe.py::_project | 113579 | 1.294 | 2.940 |

## 测试、失败保留与复核

1. 初次测试命令遗漏了此前定向回归已使用的两个ROS插件隔离开关，`launch_testing`
   导入了未选的历史测试，因缺历史manifest收集失败，exit4、8个收集错误；没有执行
   测试断言或启动真实剖析。原命令/log/XML/exit和原freeze全部保留，没有覆盖。
2. 按上一轮同一八文件命令补回 `-p no:launch_testing -p no:launch_ros` 后，
   **262通过，0跳过，1个既有非有限投影warning**。这是启动配置修正，不是修改
   测试、削弱断言或修复完整current；见[说明](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/PREFLIGHT_CORRECTION.json)。
   独立拒绝helper的合成回归仍在集合内，但没有运行其完整序列候选或重新开性能门槛。
3. 此后只启动一次已计划的完整诊断，exit0，160条精确，工作量跨遍一致。
4. [独立校验](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/INDEPENDENT_CHECK.json) 逐tick核对实际原生匹配次数=
   阶段次数=观察器次数；二进制prof全部函数计数/self时间等于逐帧聚合；阶段排他
   分区、状态分组分母、完整序列工作量均一致，不重跑控制器。
5. [最终检查](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/FINAL_CHECK.json) 验证861个预存文件全部原字节不变、
   两版freeze和13个分析输入pin不变、最近已接受交付的5个最终pin相同、当前快速路径
   实际调用存在、数组/别名/线程恢复。没有旧源码需要备份或恢复，因为本轮未编辑旧文件。

没有重跑扩大全量或标准current。此前33项缺历史工件的失败，以及缺
`reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json`导致的current收集阻断
仍未解决；定向通过不能冒充完整验证通过。没有伪造工件、skip/xfail或削弱断言。

默认no_motion、显式bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、
.8s租约/.4s审计截止、模型/相机/标定全部不变。没有新物理/实车验收；
**10Hz未证明达到，S2未通过，S6未完成。**

## 工件与复现

- [逐tick原始记录](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/profile/rows.jsonl)、[剖析SUMMARY](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/profile/SUMMARY.json)。
- [命令](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/COMMANDS-v2.json)、[分析源码](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/analyze.py)、
  [独立校验源码](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/independent_check.py)、[最终校验源码](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/final_check.py)。
- [预存文件pin](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/PREEXISTING_PINS.json)、[分析输入pin](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/ANALYSIS_INPUT_PINS.json)、
  [交付清单](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/DELIVERY_MANIFEST.json)。

剖析输出目录必须不存在；重跑属于新的诊断而非本轮补样。分析和校验输出也独占创建。
冻结文件会在重跑前后核对；运行命令不是完整全套测试或物理验收命令。
