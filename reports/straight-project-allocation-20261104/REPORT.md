# 直行齐次坐标分配优化：有效门槛通过，保留代码优化

2026-11-04，Asia/Shanghai。

**保留本轮代码优化，但不是导航/实时/物理验收。** 有效的对称包装比较中，中心、默认、
正横偏三条完整序列的总wall中位/均值全部严格改善，所有序列化输出精确一致。
只修改了 [corridor_rgb_probe.py](/home/sunrise/toponav_project/roadvision/corridor_rgb_probe.py) 中的 `_project` 首个拼接语句；
其余1091个预存文件原字节不变，旧测试没有任何修改。此前已接受的成对总体复用、
局部排除几何及整数矩对比度优化保留；未接回任何已拒绝候选。

## 实现及精确范围

原路径每次执行 `np.column_stack((points,np.ones(len(points)))) @ matrix.T`。
本轮仅在 exact ndarray、float64、N×2、C连续时，直接分配C连续float64的N×3数组：
前两列逐位复制points，第三列填精确1，然后仍执行原 `@ matrix.T`。

原/新矩阵乘法操作数的shape、dtype、strides和全部字节相同，不合并行、不变算术顺序。
原分母abs<1e-9检查、除法、全量isfinite检查及None/异常路径保持。其他dtype、布局、
数组子类、形状等全部走原column_stack。没有投影缓存、批量投影、求逆复用、精度变化，
也没有少做证据、匹配、搜索、RANSAC/求解器或制动/恢复预算。

新增46项数值测试验证原/新完整输出字节、实际matmul操作数、signed zero/NaN payload、
C/F/reverse/strided、空及极端值、非适配输入结果或异常。新增11项绑定测试见下节。
测试显式读取candidate实测快照和before旧源，不以容差或只看接受bool代替精确验证。
原模块除_project外的全部AST、_project原首语句后的所有检查AST经最终脚本核对一致。

[事前意图](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/INTENT.json)、[原计划](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/PLAN.json)和before原字节备份均在生产编辑前保存。

## 计时公平性修正：首批保留但不准入

首批实际原始gate为true，全部1950输出精确。但审查
[首版基线绑定](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/baseline.py)发现旧版local_before在整个控制器已切换投影别名之后，
每次配对调用又重复进入一次投影别名context；候选没有这层重复开销。该开销在旧版
控制器计时范围内，可能偏向候选，**不能用首批数值准入或宣称算法收益**。

首批原始比例完整保留，仅供审计，**不是本轮采用的提速结果**：

| 首批无效条件 | 原始中位改善 | 原始均值改善 |
|---|---:|---:|
| 中心 | +7.173% | +5.450% |
| 默认 | +1.114% | +2.249% |
| 正横偏 | +5.171% | +4.744% |

[修正声明](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/MEASUREMENT_CORRECTION.json)在修正比较前固定：源码候选不变、原门槛不变，
仍四轮AB/BA、一次预热、六局部八轮，仅进行一次修正比较；所有首批记录保留，不与
修正批合并，不删除慢帧、轮次或补跑求绿。首批本来已为true，这不是为求true而重测。

[对称绑定](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/baseline_v2.py)让新旧使用同一Variant.__call__。完整控制器遍的11个投影
别名切换都在timer外，遍内配对包装只透传原CURRENT_PAIR；新旧计时内的包装代码相同。
局部调用两版都使用同一context代码。仍预加载所有支持的控制器、按原函数身份发现
全部投影别名；旧版覆盖**整个控制器**，不是只替换配对内部投影。

原H/I模块及filter_h/filter_i对象始终相同，避免显式filters身份回退。原benchmark工具
未修改，[包装脚本](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/run_benchmark_v2.py)仅提供新旧投影绑定、对称dispatcher和真实配对别名发现。
11项合成测试验证无计时内重复context、两版同类包装、局部/控制器范围、正常/异常恢复。
[修正预检](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/PREFLIGHT-v2.json)固定六例×两版×显式/隐式×局部/控制器scope，48次完整输出精确，
实际匹配次数一致、数组和别名恢复。未运行新的阶段/cProfile剖析。

原[数值/工作量预检](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/PREFLIGHT.json)另外逐次对照原投影字节、所有投影输入输出、实际match、
H/I NCC、支持采样次数与每次采样hash，新旧四种dispatch全部一致，RGB/region不变。
首版定向357通过；补充投影调用方107通过；对称包装测试加入后368通过，均无skip/xfail。
定向保留1个既有非有限投影warning，调用方107项无warning。

## 有效固定比较

仅以下修正批用于性能门槛。中心75、默认40、正横偏80决策完整序列，新旧各预热1次，
各计时4轮，轮0/2为AB、1/3为BA；六局部例每版8轮。OpenCV1线程，其余线程/调度
不调参，主机非独占。只计内存控制器调用，不计解码、比较、报告IO、传输或物理。
每遍新控制器，原顺序/时间/公开任务，所有配对与投影别名正常/异常退出均恢复。

有效批 **1950条控制器记录=1560计时+390预热**，全部完整序列化输出精确；所有局部
样本保留。首批也保留同样1950条，**本轮共保留3900条**，只有修正批进入准入统计。
两批都逐行核对顺序、AB/BA、完整性和exact，绝不混合。numpy1.26.4、OpenCV4.10.0。

| 条件 | 决策/遍 | 总wall中位旧→候选(s) | 中位改善 | 总wall均值旧→候选(s) | 均值改善 |
|---|---:|---:|---:|---:|---:|
| 中心 | 75 | 33.449138→32.636017 | +2.431% | 34.183350→32.540429 | +4.806% |
| 默认 | 40 | 17.074282→16.721719 | +2.065% | 17.197815→16.914568 | +1.647% |
| 正横偏 | 80 | 35.398752→34.961533 | +1.235% | 35.495676→35.047843 | +1.262% |

六项全部严格为正，原门槛true。CPU明细在[有效完整分析](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/TIMING_ANALYSIS_CORRECTED.json)，不能替代wall门槛。
这只是同一已暴露三序列、同一主机的重复，不能声称普遍/统计显著收益；不同批或历史
绝对耗时不相减，不用逐帧伪独立置信区间。只有以下同一有效批内新旧比较有准入意义。

| 条件 | 轮0旧→候选(s) | 轮1旧→候选(s) | 轮2旧→候选(s) | 轮3旧→候选(s) |
|---|---:|---:|---:|---:|
| 中心 | 36.443270→33.283440 | 33.427990→33.202384 | 33.391855→32.069650 | 33.470286→31.606242 |
| 默认 | 16.891004→16.671925 | 17.105769→16.771513 | 17.751692→17.720533 | 17.042795→16.494303 |
| 正横偏 | 35.443342→34.931321 | 35.354162→34.327966 | 35.208757→34.991744 | 35.976443→35.940342 |

单控制器wall（ms，p95线性插值，不含预热）：

| 条件 | 样本/版 | p50旧→候选 | p95旧→候选 | 最大旧→候选 | >100ms旧→候选 |
|---|---:|---:|---:|---:|---:|
| 中心 | 300 | 485.830→460.163 | 661.772→628.050 | 1116.203→951.264 | 272→272 |
| 默认 | 160 | 488.205→488.472 | 637.263→648.301 | 888.069→888.080 | 132→132 |
| 正横偏 | 320 | 474.595→467.923 | 619.489→597.951 | 665.228→647.667 | 292→292 |

两版均696/780个计时调用超过100ms，**10Hz仍未达到**。不是端到端延迟、制动预算或实车SLA。
局部数据仅作完整记录，不能替代三完整序列门槛：

| 局部条件 | 中位改善 | 均值改善 |
|---|---:|---:|
| random | -3.581% | +1.891% |
| flat | +2.948% | -18.026% |
| repeated | +4.293% | +2.788% |
| blur | +6.194% | +2.174% |
| shift_padding | +3.259% | +8.071% |
| rectified | -1.181% | +0.300% |

## 后处理失败与修正（未重新计时）

1. analyze_v2.py更新了修正summary路径，但遗漏rows路径，尝试把首批rows同修正summary
   相比，触发断言 `(38.6522766825,33.449138104)`，没有产生成功分析文件。
   原脚本不覆盖，新analyze_corrected.py只读取修正rows/summary，重新只读分析成功。
2. 首次启动回放脚本时缺仓库PYTHONPATH，ModuleNotFoundError: tools，exit1，**尚未执行
   任何控制器调用**，无回放progress/REPLAY输出。仅在子进程加仓库根PYTHONPATH，原回放
   源/输入/次数不变，此后完成一次固定13例实际回放。首个失败log/exit完整保留。

[修正记录](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/POSTPROCESS_STARTUP_CORRECTION.json)说明上述失败，没有修改候选或任何性能数据，也没有再跑性能比较。

## 条件回放、回归及最终核对

- 有效门槛通过后，执行事先固定13例旧记录回放：**581条决策、39条终止后帧**全部
  精确，旧120Hz物理trace审计errors=[]，反馈投递0。输入含旧baseline/reference/
  gradient_transfer/gradient_history、缺失/冻结、无许可/重复纹理拒绝。见
  [回放结果](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/REPLAY.json)。仅核对旧记录，**没有新仿真世界、物理闭环或执行器**。
  导入pybullet打印build banner不是启动了新物理运行。
- 最终12文件定向 **368通过、1既有warning**；扩大55文件 **1539通过、33失败、1既有warning**。
  [失败比较](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/FAILURE_COMPARISON.json)确认33项class/name/message与上一轮完全相同，都是历史
  工件缺失；无skip/xfail、伪造工件、旧断言修改或删除。扩大不是全部current。
- 原发现/配置的标准current仍因缺
  `reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json`收集阻断，exit2、selected0、
  verification_complete=false。[current结果](/home/sunrise/toponav_project/reports/straight-project-allocation-full-current-20261104/summary.json)不等于全量通过。
- 两版测量freeze分别646/681文件，回放freeze分别1911/1946文件均核对原字节。有效测量
  SHA256为 `adfa1d1ef2fa9888642861d3183d7f36ca87565208dc62a98a27ce9aef8cc896`。
  原/修正两批分析只读复算均与对应首个成功结果逐字节一致，无新控制器调用。
- [最终独立核对](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/final_check.py)还用十进制还原逐条整数纳秒，再用精确分数比较每例总wall中位/
  均值，六项严格改善，不依赖浮点容差决定gate；验证旧source备份、1091旧pin不变、
  唯一生产改动、旧检查AST、测试/回放与所有冻结边界。见[最终核对结果](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/FINAL_VERIFICATION.json)。

默认no_motion、显式bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、.8s租约/
.4s审计截止、模型/相机/标定不变。历史gradient_transfer任务不因本轮代码优化成为
默认导航策略。**S2未通过、S6未完成，没有新物理/实车验收，全量验证仍阻断。**

## 工件与只读复核

[有效逐tick原始记录](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/benchmark-corrected/rows.jsonl)、[首批保留记录（无效准入）](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/benchmark/rows.jsonl)、
[对称基线绑定](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/BENCHMARK_BASELINE_BINDING-v2.json)、[分析输入pin](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/ANALYSIS_INPUT_PINS.json)、
[最终源码pin](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/FINAL_PINS.json)、[交付清单](/home/sunrise/toponav_project/reports/straight-project-allocation-20261104/DELIVERY_MANIFEST.json)、
[实现说明](/home/sunrise/toponav_project/docs/STRAIGHT_HOMOGENEOUS_PROJECTION.md)。

analyze_corrected.py与final_check.py接受 `--output` 指向不存在的新路径，进行只读复算，
不启动控制器。首批不用于性能准入，不能重新混入；冻结性能命令也不是物理验收命令。
