# 直行 I 支持掩码准备：性能门槛失败，默认恢复

2026-11-04，Asia/Shanghai。

**本轮不接入。** 候选输出完全精确，但默认序列的总wall中位/均值退化
**0.891% / 3.029%**，不满足事前三例全部严格改善的门槛。中心、正横偏本次样本的
改善不能抵消默认失败，也不是普遍收益或统计显著性证据。

两个生产文件已恢复修改前原字节，942个预存文件全部不变。此前已接受的成对总体
复用、精确局部排除几何、原生补丁对比度快速判定均保留；没有追加候选计时、
剖析或条件回放，也未重启已拒绝的类型替换、patch/NCC、scratch、求逆或归约方案。

## 候选做了什么

I通道每次支持采样原先都将128×128掩码转为float32。候选仅在同次原生区域中
**惰性准备静态掩码采样画布**，复用一次原 `astype(np.float32)` 的结果。

- 新独立helper：[源码](/home/sunrise/toponav_project/roadvision/prepared_support_mask.py)。只保存私有区域拥有、
  调用期间不变的源掩码及惰性转换；最多2张float32画布，共131072字节数组数据，
  另有Python对象开销。不是零分配，也不是跨区域/帧或控制器状态。
- 成对模块为原生I证据准备两份包装，原掩码仍用于支持侵蚀；对比度模块识别包装，
  但**每次有效补丁仍执行原getRectSubPix、原坐标、原>=1.-1e-6检查**。
- 新包装不得进入整流/custom过滤器路径；自定义证据/支持回调获得原掩码对象。
  无搜索器的独立I入口也解包，保留原独立搜索路径。拒绝的灰度补丁不触发转换。
- 没有补丁/NCC缓存，没有合并H/I接受，没有少做支持采样、有限性检查、逻辑搜索
  或实际完整匹配。分数dtype、后端、全画布/竞争范围、max/count、role/generation、
  逐轨迹清空、模型/训练/留出均不变。原H/I源文件未编辑。
- 精确性依据是同一不变源数组的同一个astype，其数值、形状、步长均相同；仍由原
  OpenCV函数逐次采样，而不是用“全支持”猜测或近似支持决定。

[意图](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/INTENT.json)和[比较计划](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/PLAN.json)在测试/正式计时前固定。
两个旧生产文件的原字节备份位于before；原候选、helper和原测试均在candidate归档。

## 基线绑定与正确性

本轮改了配对及对比度helper两个文件。如果仅加载旧配对源码，它会误导入当前新
helper，不能当作真实旧版。为此沿用原benchmark加载器，并在独立
[基线绑定](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/baseline.py)中显式把旧配对H/I入口绑定至旧helper快照。
旧/新仍共享同一原H/I模块及filter_h/i身份，原benchmark工具本身未修改。

[预检](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/PREFLIGHT.json)核对6局部样例的显式/隐式旧新四种调用：

- 全部完整序列化输出与旧版一致，RGB输入不变。
- 支持采样次数及每次结果hash、H/I NCC次数、实际匹配次数逐项一致。
- random的25轨迹仍100次匹配、50次支持采样；候选50次准备请求仅转换**2次**，
  旧helper未触发候选准备。flat和整流分支均0次候选转换。
- 初始九文件定向 **286通过，1个既有非有限投影warning**，无失败或skip/xfail。
  新24项覆盖布局/掩码值/边界、采样字节与次数、惰性拒绝、非适配对象、自定义回调、
  无搜索器路径、对象释放、正常/异常退出的两次转换与区域释放。

[测量冻结](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/FREEZE.json)338文件，SHA256
`c5bcc765de9b07ef4d99cf7bf1511a522c4b7f62247fdde4f355778567e7f067`。
[回放计划](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/REPLAY_PLAN.json)及1666文件回放freeze也在计时前准备，但门槛
失败后没有执行回放。冻结包含所有既定公共输入、源码、工具、计划及快照。

## 唯一固定串行比较

复用原完整benchmark，仅增加上述双文件基线绑定。每版每条完整序列预热一次，
四轮固定AB/BA交替；六局部样例每版8轮。OpenCV单线程，其他线程/调度不调整，
主机非独占。只计内存控制器调用，不计解码、比较、写报告、传输或物理。
计时期间没有并行测试、其他性能/剖析、回放或物理审计。

**1950条控制器记录全部保留：1560计时、390预热，完整输出全部精确。**
另保留所有局部计时记录。没有删慢帧、慢轮次、改门槛或补跑求通过。

| 条件 | 决策/遍 | 总wall中位旧→候选(s) | 中位改善 | 总wall均值旧→候选(s) | 均值改善 |
|---|---:|---:|---:|---:|---:|
| 中心 | 75 | 42.874646→41.336568 | +3.587% | 42.072011→41.125517 | +2.250% |
| 默认 | 40 | 18.812774→18.980435 | -0.891% | 18.900872→19.473384 | -3.029% |
| 正横偏 | 80 | 40.705830→40.185898 | +1.277% | 42.213733→40.732199 | +3.510% |

正数表示本次样本改善，负数表示退化。CPU不能替代wall门槛；全CPU数据保存在
[完整分析](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/TIMING_ANALYSIS.json)。四轮总wall如下，轮0/2为AB，1/3为BA：

| 条件 | 轮0旧→候选(s) | 轮1旧→候选(s) | 轮2旧→候选(s) | 轮3旧→候选(s) |
|---|---:|---:|---:|---:|
| 中心 | 35.172311→39.813909 | 43.391939→43.018654 | 42.357352→42.859227 | 47.366443→38.810279 |
| 默认 | 18.988169→17.834508 | 18.081517→17.236845 | 18.637379→20.126361 | 19.896425→22.695820 |
| 正横偏 | 42.182735→40.892948 | 39.228924→38.964215 | 38.521808→39.478848 | 48.921464→43.592783 |

这只是同一已暴露序列、同一主机的重复，不是新独立场景。不用逐帧伪独立置信区间，
不与历史轮次基线相减，不从结果臆断某一局部操作就是默认退化的原因。

单控制器调用wall统计（ms；p95线性插值；不含预热）：

| 条件 | 计时样本/版 | p50旧→候选 | p95旧→候选 | 最大旧→候选 | >100ms旧→候选 |
|---|---:|---:|---:|---:|---:|
| 中心 | 300 | 565.523→587.318 | 972.556→858.123 | 1643.999→1510.161 | 272→272 |
| 默认 | 160 | 553.988→536.239 | 734.859→794.818 | 772.134→1445.391 | 132→132 |
| 正横偏 | 320 | 546.666→539.294 | 731.293→721.274 | 1347.609→1094.004 | 292→292 |

两版均696/780条超过100ms，**10Hz未达到**，实时/制动预算没有因此验收。
局部完整样例的wall改善如下，不能替代三例完整序列门槛或用于挽救失败：

| 局部条件 | 中位改善 | 均值改善 |
|---|---:|---:|
| random | +2.420% | +0.133% |
| flat | -7.281% | -8.496% |
| repeated | +0.731% | -4.682% |
| blur | -8.085% | -6.645% |
| shift_padding | -2.804% | -2.714% |
| rectified | +2.329% | +2.055% |

## 撤回、测试归档与最终验证

1. 原门槛失败后，先核对实测源码/测试与candidate快照一致，随后两个生产文件恢复
   before原字节。[恢复记录](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/RESTORATION.json)核验全部942个旧pin一致。
2. 独立helper保留，但**默认配对和对比度模块都不导入它**。新测试原24项断言的函数
   AST完整不变，仅将测试模块变量显式绑定至归档候选和归档helper，不猴子补丁默认
   模块，并增加默认两源码仍为before字节的断言。原实测测试源保留在candidate；
   [测试范围说明](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/FINAL_TEST_SCOPE.json)记录这项收尾改动，不假装最终测试
   源与原测量测试源相同，也不将归档候选测试冒充默认推广。
3. 最终九文件定向 **287通过、1warning**；扩大47文件 **1351通过、33失败、1warning**。
   [失败比对](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/FAILURE_COMPARISON.json)确认33项class/name/message与上一轮
   完全相同，都是既有缺历史工件问题。没有skip/xfail、伪造工件或削弱旧断言。
4. 标准完整current运行保持原发现/配置，独占输出到
   [完整current结果](/home/sunrise/toponav_project/reports/straight-paired-support-full-current-20261104/summary.json)。
   仍缺历史manifest而收集阻断，exit2、selected0、verification_complete=false，
   **不能称为全量通过**。没有改变配置来掩盖该阻断。
5. 分析器核对1950行全部顺序/精确标记、AB/BA、逐轮wall/CPU求和与原gate；恢复后
   经实测归档映射核验338个测量pin。只读复算与初版逐字节相同，无新控制器/计时。
   [最终核对](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/FINAL_VERIFICATION.json)还验证原24项测试AST、默认不导入helper、
   预检工作量及采样hash、942旧pin、测试和current边界。
6. 1666回放pin经实测快照映射核验，**未执行13例条件回放/旧物理审计**；没有
   REPLAY.json或进度记录，不借上一轮回放充当本轮凭据。

默认no_motion、显式bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、
.8s租约/.4s审计截止、模型/相机/标定都不变。实验策略未推广，没有新物理/实车
验收；S2未通过、S6未完成，既有全量阻断也未修复。保留已接受默认，不将减少转换
包装成“已经提速”。

## 工件与只读复算

[原始每tick](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/benchmark/rows.jsonl)、[基线绑定记录](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/BENCHMARK_BASELINE_BINDING.json)、
[分析源码](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/analyze.py)、[最终核对源码](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/final_check.py)、
[最终pin](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/FINAL_PINS.json)、[交付清单](/home/sunrise/toponav_project/reports/straight-paired-support-20261104/DELIVERY_MANIFEST.json)。

分析器接受 `--output` 指向不存在的新文件，只读取原数据与实测快照，不执行控制器。
当前默认已恢复，不能用候选freeze重新运行原性能命令。实验机制见
[独立helper说明](/home/sunrise/toponav_project/docs/STRAIGHT_PAIRED_SUPPORT.md)。
