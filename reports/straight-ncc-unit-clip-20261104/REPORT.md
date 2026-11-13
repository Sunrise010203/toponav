# 直行NCC单位区间裁剪：性能门槛失败，默认已恢复

2026-11-04，Asia/Shanghai。

**本轮不接入。** 唯一一次固定完整比较的输出全部精确，但默认序列总wall中位/均值退化
**0.827% / 0.048%**，正横偏中位退化 **0.291%**。不满足三条完整序列总wall中位和均值
全部严格改善的原门槛。中心改善、正横偏均值改善和局部结果都不能抵消失败。

唯一生产改动 `roadvision/rgb_motion_checks.py` 已恢复before原字节，**1211个预存文件全部
字节不变**，旧测试零修改。此前已接受的齐次投影分配、成对总体复用、局部排除几何、
整数矩对比度优化保留。候选仅在candidate归档中，不再导入默认链路。

## 只省恒等裁剪，不是NCC缓存

原_ncc最后返回 `float(np.clip(a@b/den,-1,1))`，分母<1e-9仍返回None。
候选保持原astype(float)、ravel、均值/中心化、两次norm、点积与除法不变，算出score后，
仅在 `type(score) is np.float64` 且 `-1 <= score <= 1` 时直接 `float(score)`。
这时原clip是**逐位恒等映射**，包括有符号零；其他类型、超界、NaN/Inf全部保留原clip。
不使用容差，不把“接受bool相同”冒充浮点证据精确。

没有patch/NCC缓存、共享归一化或近似点积，没有合并H/I接受；每个原NCC仍分别执行
完整均值、范数和点积。补丁提取、std门槛、全画布匹配、支持/有限性检查、搜索总体、
RANSAC/模型/求解预算不变。原H模块只有_ncc被修改过，其他函数AST逐项相同；候选的
归一化、原微小分母比较和点积/除法AST也与旧版相同。原I、配对、对比度模块未编辑。

[意图](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/INTENT.json)、[固定计划](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/PLAN.json)、旧source备份及1211个before pin在编辑前保存。
新79项数值测试从初版起就显式加载归档候选，覆盖dtype/布局、原始输入不变、极端/
非有限值、分母边界、forced IEEE位模式（含signed zero与NaN payload）、warning/异常，
并记录两次norm的全部操作数字节。另有12项合成测量基础设施测试。旧断言未削弱。

## 对称基线与实际工作量

本轮新增离线 [SymmetricFunctionBinding](/home/sunrise/toponav_project/tools/straight_function_benchmark.py)，生产不导入，
原benchmark工具源码不修改。沿用上一轮修正后的公平原则，从一开始使用同一
FunctionVariant.__call__包装新旧；控制器遍的别名切换位于timer外，遍内只透传原配对函数。
局部调用则两版都进入同一context。不存在单边的计时内重复别名切换。

[旧NCC绑定](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/ncc_binding.py)提取before真旧_ncc，并按函数身份替换整个已预加载控制器链路的
三个别名：H、I、reference_patch_evidence。不是只换配对函数中的部分调用。原H/I模块、
filter_h/filter_i、配对函数与证据回调身份都不替换，避免显式filters触发独立旧路径。
12项合成测试验证local/controller作用域、相同包装、无计时内切换、函数身份、异常/
重入拒绝、benchmark配置恢复。[运行退出核对](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/BENCHMARK_RUNTIME_RESTORED.json)确认别名及dispatcher恢复。

[六例预检](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/PREFLIGHT.json)记录旧新各显式/隐式完整输出，逐项比较实际match、NCC次数、
每次NCC输入/输出字节、两次norm操作数字节、支持采样次数及逐次hash。所有字段完全
一致，唯有被证明恒等的clip调用数减少。RGB和region不变。另分别通过对称dispatcher的
局部/完整controller context验证全部六例×两版×显式/隐式，均精确并恢复别名。

| 局部例 | 实际匹配（两版相同） | NCC（两版相同） | norm（两版相同） | NCC内clip旧→候选 |
|---|---:|---:|---:|---:|
| random | 100 | 50 | 100 | 50→10 |
| flat | 0 | 0 | 0 | 0→0 |
| repeated | 100 | 50 | 100 | 50→12 |
| blur | 100 | 50 | 100 | 50→10 |
| shift_padding | 100 | 50 | 100 | 50→0 |
| rectified | 148 | 33 | 66 | 33→9 |

random的10次候选clip仍由超界等原路径处理，不因理论NCC范围就擅自假定不会发生。
rectified的完整148次匹配也不减。省clip次数本身不是实测提速或导航改善证据。
初次14文件定向 **459通过、1既有非有限投影warning**，无失败、skip或xfail。

## 唯一固定串行比较

测量冻结647文件，SHA256：
`383a60e491282611021da5ad29a712f87cc26c687d31c02ed7c8322b28c88d61`。
事前另准备1973文件回放freeze及固定13例计划，但性能失败后**未执行回放**。

中心75、默认40、正横偏80决策完整序列；每版/例预热1次，4轮固定AB/BA，轮0/2为AB、
1/3为BA；六局部每版8轮。OpenCV1线程，其余线程/调度不调整，主机非独占。每遍新
控制器、完整原顺序和时间，只有公共任务/RGB进入控制器，不用真值/结果/audit来选点。
只计内存控制器调用，不计解码、序列化比较、报告IO、传输或物理。

**1950条控制器记录全部保留：1560计时、390预热，全部完整输出精确。** 所有局部样本
也保留。没有删慢帧、轮次、换门槛、追加比较/剖析，计时期间没有并行测试或回放。
numpy1.26.4、OpenCV4.10.0，结束后OpenCV线程恢复。

| 条件 | 决策/遍 | 总wall中位旧→候选(s) | 中位改善 | 总wall均值旧→候选(s) | 均值改善 |
|---|---:|---:|---:|---:|---:|
| 中心 | 75 | 36.761172→36.649520 | +0.304% | 36.750472→36.482800 | +0.728% |
| 默认 | 40 | 18.897595→19.053954 | -0.827% | 19.019143→19.028349 | -0.048% |
| 正横偏 | 80 | 40.327595→40.444805 | -0.291% | 40.384226→39.866893 | +1.281% |

正数为本次样本改善，负数为退化；门槛false。CPU统计在[完整分析](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/TIMING_ANALYSIS.json)，不能替代wall门槛。
同一已暴露序列重复不是独立新场景，不给逐帧伪独立置信区间，不拿历史绝对耗时相减，
也不把“噪声”、主机调度或某个局部操作臆断为退化根因。

| 条件 | 轮0旧→候选(s) | 轮1旧→候选(s) | 轮2旧→候选(s) | 轮3旧→候选(s) |
|---|---:|---:|---:|---:|
| 中心 | 37.005977→36.600290 | 36.473568→36.856842 | 36.834622→36.698749 | 36.687722→35.775319 |
| 默认 | 19.025612→18.428759 | 18.709096→18.672981 | 18.769578→19.434927 | 19.572285→19.576730 |
| 正横偏 | 42.177796→41.294284 | 38.073085→39.869117 | 42.808630→41.020494 | 38.477395→37.283678 |

单控制器wall统计（ms，p95线性插值，不含预热）：

| 条件 | 样本/版 | p50旧→候选 | p95旧→候选 | 最大旧→候选 | >100ms旧→候选 |
|---|---:|---:|---:|---:|---:|
| 中心 | 300 | 518.496→514.167 | 722.015→711.527 | 1158.014→1094.101 | 272→272 |
| 默认 | 160 | 540.167→552.254 | 716.736→729.479 | 763.540→825.146 | 132→132 |
| 正横偏 | 320 | 532.213→523.796 | 691.849→681.575 | 956.527→989.237 | 292→292 |

两版均696/780条超过100ms，**10Hz未达到**，不是端到端或制动预算验收。
局部数据仅作完整记录，不能替代完整门槛：

| 局部例 | 中位改善 | 均值改善 |
|---|---:|---:|
| random | +4.670% | +4.702% |
| flat | +13.294% | +2.138% |
| repeated | +8.908% | +6.190% |
| blur | +1.812% | +2.468% |
| shift_padding | +0.426% | +3.884% |
| rectified | +1.936% | +2.121% |

## 恢复、回归与独立核对

1. gate失败后先核对生产源码等于实测candidate快照，再恢复before原字节。
   [恢复记录](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/RESTORATION.json)确认1211个旧文件全部相同。新79项数值测试、12项测量合成测试
   及独立离线基础设施保留；数值测试只显式加载归档，不会重新启用默认候选。
2. 恢复后14文件定向 **459通过、1既有warning**；扩大57文件 **1630通过、33失败、1既有warning**。
   [失败比对](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/FAILURE_COMPARISON.json)确认全部33项class/name/message与上一轮完全相同，均为缺历史
   工件。没有skip/xfail、伪造工件、改配置或改旧断言。扩大集合不是全量current。
3. 标准current按原发现/配置运行，仍缺
   `reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json`而收集阻断，exit2、selected0、
   verification_complete=false，[current结果](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-full-current-20261104/summary.json)不能称全量通过。
4. 分析器核对1950行顺序、AB/BA、完整性、exact及每轮wall/CPU求和。恢复后经实测
   candidate映射核对647测量pin和1973回放pin；只读复算与首个分析结果逐字节一致，
   不运行新控制器。没有REPLAY.json或回放progress，不借上轮回放充当本轮凭据。
5. [独立核对](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/final_check.py)从逐条计时十进制恢复整数纳秒，再以精确分数复核每例总wall中位/
   均值门槛；不靠浮点容差决定gate。还核对原/新源码AST、预检工作量、全部old pin、
   测试/冻结/恢复/current边界，输出[最终核对结果](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/FINAL_VERIFICATION.json)。

默认no_motion、显式bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、.8s租约/
.4s审计截止、模型/相机/标定全部不变。没有新物理/实车验收，S2未通过、S6未完成。
历史gradient_transfer实验任务不因本轮代码实验变成默认策略；全量缺工件问题未修复。

## 工件与只读复算

[全部原始tick](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/benchmark/rows.jsonl)、[三别名对称绑定](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/BENCHMARK_BASELINE_BINDING.json)、
[分析输入pin](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/ANALYSIS_INPUT_PINS.json)、[最终源码pin](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/FINAL_PINS.json)、
[交付清单](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/DELIVERY_MANIFEST.json)、[独立实验说明](/home/sunrise/toponav_project/docs/STRAIGHT_NCC_UNIT_CLIP.md)。

analyze.py及final_check.py接受 `--output` 指向不存在的新文件，仅只读复核原数据/快照，
不重新计时。默认已恢复，不能拿候选freeze重新执行原性能命令。
