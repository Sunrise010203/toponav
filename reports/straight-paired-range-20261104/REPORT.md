# 直行原生补丁灰度跨度优化：门槛失败，默认恢复

2026-11-04，Asia/Shanghai。

**本轮不接入。** 全部输出精确一致，但默认序列总wall中位/均值退化
**6.402% / 2.914%**，正横偏退化 **1.442% / 0.738%**。中心本次样本改善不能抵消
另两例失败。未追加候选计时、剖析或条件回放，不补跑求绿。

唯一生产改动及临时旧测试改动都已恢复before原字节；**1013个预存文件逐字节不变**。
默认保留此前已接受的成对总体复用、局部排除几何、整数积分矩对比度判定。
独立helper和24项测试仅为可复核实验，默认不导入。

## 候选范围与精确性

[独立helper](/home/sunrise/toponav_project/roadvision/native_patch_range.py) 用cv2.minMaxLoc计算11×11 uint8补丁的精确极值。
对N=121、跨度R和总体方差V，有V≥R²/(2N)、V≤R²/4；所以R≥47保证V>9，R≤1保证V<1。
这两种都在既有整数矩证明的远区内；其余跨度继续原integral2和原float32 std边界路径。
不是改变std≥2阈值，不用浮点输出容差掩盖差异。

只改了native_patch_contrast.py的_contrast_gate入口，paired_descriptor_checks.py始终没改。
原补丁提取/float32字节、H/I源、NCC、实际匹配、支持采样和检查、分数后端及归约保持原样。
不减少搜索、RANSAC/求解预算，不合并H/I接受，不变模型/训练/留出。

[初版意图](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/INTENT.json)、[修正意图](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/INTENT-v2.json)与[固定计划](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/PLAN.json)均保留，不覆盖旧工件。

## 开发失败、预检与基线

初次定向 **310通过、1失败**：上一轮支持掩码测试锁定整个contrast文件原字节，
因而拒绝本轮无关的contrast-only实验。原日志/XML保留。计时前备份后临时把该断言限定为
配对源码原字节、contrast除明确改动的_contrast_gate外所有原函数AST相同，并禁止导入
已拒绝的支持helper；所有原支持/回调断言不变。修正后 **311通过、1既有warning**。
[开发修正记录](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/DEVELOPMENT_CORRECTION.json)明确性能失败时测试也恢复；现在已恢复，
**没有永久削弱任何旧测试断言**。candidate保留真实实测版本，旧版保留在before。

[六例预检](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/PREFLIGHT.json)包含random/flat/repeated/blur/shift_padding/rectified，
旧新各显式/隐式四种调用，完整序列化输出精确、RGB不变；逐项匹配实际match、H/I NCC、
支持采样次数及每次采样hash。random仍100次实际匹配，100次积分变为100次充分接受；
flat仍0匹配，100次积分变为100次充分拒绝；blur的100次中92次充分接受、8次原积分；
rectified仍148次匹配、0次范围检查。每次充分决定均现场对照原float32 std和int64精确矩。
省积分次数不是完整控制器已提速的证据。

旧配对源码会导入当前helper，因此仅加载旧配对不是真实旧基线。
[基线绑定](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/baseline.py)显式绑定before contrast的H/I证据入口；旧新共享原H/I模块和filter身份。
原benchmark工具未改，[本轮包装](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/run_benchmark.py)仅替换load_before绑定。

测量freeze **346文件**，SHA256
`acfe56e2373825ba4c048660ff2e3ee5cd03dec0918e6ae995ef9fba583bc4cb`。
计时前另冻结1744个回放输入/工件；因门槛失败，**未执行条件回放**。

## 唯一固定串行比较

六局部例每版8轮；中心75、默认40、正横偏80决策完整序列，每版/例预热1次，
四轮固定AB/BA（0/2为AB，1/3为BA）。OpenCV1线程，其他线程/调度不调整，主机非独占。
只计内存控制器调用，不含解码、比较、报告IO、传输或物理。计时期间没有并行测试、
剖析、回放或物理审计。numpy1.26.4、opencv4.10.0，结束后OpenCV线程已恢复。

**1950条控制器记录全部保留：1560计时、390预热，全部输出精确一致。**
所有局部样本也保留；不删慢帧/轮次、不调门槛、不追加比较。

| 条件 | 决策/遍 | 总wall中位旧→候选(s) | 中位改善 | 总wall均值旧→候选(s) | 均值改善 |
|---|---:|---:|---:|---:|---:|
| 中心 | 75 | 40.289240→39.562403 | +1.804% | 40.647347→40.267709 | +0.934% |
| 默认 | 40 | 17.597323→18.723971 | -6.402% | 18.098957→18.626318 | -2.914% |
| 正横偏 | 80 | 40.345075→40.926906 | -1.442% | 40.609813→40.909534 | -0.738% |

正数表示本次样本改善，负数表示退化。门槛要求三例中位及均值全部严格改善，结果false。
CPU统计在[完整分析](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/TIMING_ANALYSIS.json)中，不能替代wall门槛。

| 条件 | 轮0旧→候选(s) | 轮1旧→候选(s) | 轮2旧→候选(s) | 轮3旧→候选(s) |
|---|---:|---:|---:|---:|
| 中心 | 40.425464→38.231417 | 42.964653→43.714614 | 39.046257→38.240226 | 40.153015→40.884579 |
| 默认 | 19.889643→19.631485 | 17.336909→17.425844 | 17.857737→18.323602 | 17.311539→19.124340 |
| 正横偏 | 42.419601→40.773018 | 41.306436→38.780239 | 39.329502→43.004084 | 39.383713→41.080794 |

单控制器调用wall统计（ms；p95线性插值，不含预热）：

| 条件 | 样本/版 | p50旧→候选 | p95旧→候选 | 最大旧→候选 | >100ms旧→候选 |
|---|---:|---:|---:|---:|---:|
| 中心 | 300 | 559.514→562.512 | 907.998→840.226 | 1163.217→1526.026 | 272→272 |
| 默认 | 160 | 520.778→540.362 | 692.818→750.120 | 870.360→837.782 | 132→132 |
| 正横偏 | 320 | 540.214→535.541 | 690.808→714.198 | 1053.757→827.720 | 292→292 |

两版均 **696/780** 次计时调用超过100ms，**10Hz未达到**。本次序列重复不是新独立场景，
不做逐帧伪独立置信区间，不拿历史轮次基线相减，也不臆断退化的局部原因。

局部完整样例仅供记录，不能挽救完整门槛失败：

| 局部条件 | 中位改善 | 均值改善 |
|---|---:|---:|
| random | -1.092% | +2.789% |
| flat | +6.434% | -0.377% |
| repeated | +0.611% | +1.347% |
| blur | +1.689% | +1.929% |
| shift_padding | +6.738% | +8.821% |
| rectified | -1.512% | -1.309% |

## 撤回与最终验证

1. 核对当前实测源码和旧测试等于candidate快照后，恢复native_patch_contrast.py及
   test_prepared_support_mask.py原字节。[恢复记录](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/RESTORATION.json)确认1013个旧pin一致。
   新helper与24项测试原字节保留；测试从初版就显式加载归档候选，不猴子补丁默认模块。
2. 恢复后10文件定向 **311通过、1既有非有限投影warning**；扩大48文件
   **1375通过、33失败、1既有warning**。无skip/xfail。
   [失败比对](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/FAILURE_COMPARISON.json)确认33项class/name/message与上一轮完全相同，
   均为缺历史工件；没有伪造工件或改配置掩盖失败。
3. 原发现/配置运行标准current，[current结果](/home/sunrise/toponav_project/reports/straight-paired-range-full-current-20261104/summary.json)
   仍因缺 `reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json` 收集阻断，
   exit2、selected0、verification_complete=false。**不能称全量通过**。
4. 只读复算分析到新文件，与初版逐字节相同，不执行控制器。分析器核对1950行全部
   顺序、精确标记、AB/BA、逐轮wall/CPU求和与原gate。测量346及回放1744个pin经
   candidate四项映射核验，恢复默认不会被冒充成实测候选。
5. [最终核对脚本](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/final_check.py)验证旧pin、两个恢复文件、新测试归档绑定、
   默认不导入helper、预检工作量、所有测试边界和current阻断，输出
   [最终核对](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/FINAL_VERIFICATION.json)。没有REPLAY.json/回放进度，不借旧审计充当本轮验收。

默认no_motion、显式bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、
.8s租约/.4s审计截止、模型/相机/标定都不变。没有新物理/实车验收，S2未通过、S6未完成，
历史缺工件问题未修复。保留已接受默认，不将减少积分包装成已实测提速。

## 工件与复核

[全部原始tick](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/benchmark/rows.jsonl)、[基线绑定记录](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/BENCHMARK_BASELINE_BINDING.json)、
[分析源码](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/analyze.py)、[最终源码pin](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/FINAL_PINS.json)、[交付清单](/home/sunrise/toponav_project/reports/straight-paired-range-20261104/DELIVERY_MANIFEST.json)、
[独立helper说明](/home/sunrise/toponav_project/docs/STRAIGHT_PAIRED_RANGE.md)。

分析器及final_check均接受 `--output` 指向不存在的新文件；只读取原记录/归档，不重新计时。
默认已恢复，不能拿候选freeze重新运行原性能命令。
