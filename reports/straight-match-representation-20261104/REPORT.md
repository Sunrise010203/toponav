# 直行模板匹配：输入类型替换的精确等价检查

2026-11-04，Asia/Shanghai。

**结论：不接入“直接把float32匹配输入改为uint8”的无条件替换。**
当前IPP开启时，固定1106张完整分数图全部逐位一致；但同一批合成样本在非IPP回退
路径出现6张反例、34424个不同分数。有限样本通过不是通用精确性证明。
本轮仅新增隔离数值检查、测试和报告，**897个预存文件原字节不变**，默认实现不变。

## 假设与事前边界

[原计划](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/PLAN.json)在数值执行前固定：

- 目标为128×128、模板11×11，使用原 `TM_CCOEFF_NORMED`；只有float32数组能够
  **字节精确往返uint8**时才比较替代输入，不把浮点小数截断当成同一输入。
- 种子98417，10种合成样本；复用6种已有完整局部样例，每种隐式/显式原过滤器各一次。
- 对比完整float32结果的每一位，包括有符号零、NaN payload；不使用误差容限。
- 真实局部过滤器始终收到**原float32匹配结果**；uint8结果只做旁路比较，不进入H/I证据。
- 所有固定样本保留，不搜种子、不删不一致样本。任何位差都阻止无条件替换；
  即使全通过，也不据有限样本推广。普通计时、完整控制器序列、回放和物理执行均0。

两个数值检查都为单次固定执行。后续非IPP检查是源码审查触发的**新增对抗性检查**，
不是原计划中预先包含的模式，事前另存[回退计划](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/FALLBACK_PLAN.json)与
[回退冻结](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/FALLBACK_FREEZE.json)；不覆盖初轮计划或结果，不补样求通过。

## 初轮：当前IPP开启，1106张逐位一致

OpenCV4.10.0、NumPy1.26.4，OpenCV单线程，`useOptimized=true`、`useIPP=true`。
[结果](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/numerical/SUMMARY.json)：

- 10张合成图 + 完整局部样例1096次实际匹配，共 **1106张、15399944个分数**。
- 全部输入满足精确表示要求，完整分数图**0个位差**。
- 6局部样例×2种dispatch的12组完整序列化输出与未观察运行相同，输入RGB/region不变；
  flat样例原本不匹配，仍为0次；未跳过或补做其工作。
- 回调和线程恢复；没有将uint8图返给过滤器，故这些完整输出相同不等于候选管线已验收。

这些结果没有证明IPP内部对所有合法输入都精确等价，也没有普通时延或收益结论。

## 源码审查与非IPP反例

只读下载[OpenCV4.10.0上游源码](https://github.com/opencv/opencv/blob/4.10.0/modules/imgproc/src/templmatch.cpp)，
固定其原字节和SHA；副本及build information在
[源码审查目录](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/source-review/SCOPE.json)。
版本标签不等于已安装二进制的构建来源证明，没有下载或执行替代库。

源码中 `crossCorr` 根据输入深度选择DFT精度：uint8对应float32，float32对应float64。
IPP也根据深度调用不同入口，且IPP失败可回退通用路径。因而**不能只以输入整数值
相同或useIPP开关为真，就断言结果始终相同**。

新增对抗性检查仅在独立检查进程中临时关闭IPP，使用原先同样10个合成样本、
同样尺寸和种子各一次，未运行控制器或性能测试；finally恢复原IPP/线程设置，
从未改变部署配置。[结果](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/fallback-numerical/SUMMARY.json)：

| 样本 | 不同分数数 | 最大绝对差 |
|---|---:|---:|
| constant_0 | 0 | 0 |
| constant_70 | 0 | 0 |
| constant_255 | 0 | 0 |
| random_full | 8480 | 1.668930054e-06 |
| random_low | 8294 | 0.001504611224 |
| ramp_x | 1888 | 0.0001032948494 |
| ramp_xy | 645 | 0.0002065896988 |
| checker | 0 | 0 |
| periodic | 6962 | 4.507601261e-07 |
| fractional_extraction | 8155 | 2.056360245e-06 |

合计10张139240个分数，其中 **6张、34424个分数不一致**，最大绝对差约0.0015046112。
这不是仅有符号零或误差显示差异。6个反例均保存输入、模板、原结果和替代结果的完整
`.npz`工件，记录第一处不同的float32位模式和全部数组hash，可独立复核而不重跑匹配。

该反例否定**跨现有后端的无条件输入类型替换**，但不证明所有IPP专用实现均不可能
优化。当前没有IPP全域精确性及失败回退证明，因此也没有接入任何条件化IPP候选。
不通过设置误差容差、修改接受阈值或强制更改部署后端来“修复”此失败。

## 代码、测试与独立核对

- 检查代码：[初轮脚本](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/check.py)、[回退脚本](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/fallback_check.py)。
  均独立保存在报告目录，不被默认入口导入。旧生产、工具、测试和报告均未编辑。
- 初轮 **198通过、1既有非有限投影warning**；回退设置恢复专项 **4通过**。
  合计202项通过（27项新检查器测试、175项既有回归），没有skip/xfail。
  覆盖一ULP差、零符号、NaN payload、整数往返资格、非连续输入、失败关闭、
  原结果返回身份、反例保存、修改输入pin检测、成功/异常及原IPP开/关时设置恢复。
- [独立核对](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/INDEPENDENT_CHECK.json)不执行match或控制器：重算6个反例全部
  位差、输入往返、hash、最大绝对差及第一处位模式，核对1116行记录与统计、固定顺序、
  12组局部工作量和完整输出标记，以及所有源码/输入/分析pin。
- [原始冻结](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/FREEZE.json)903文件，[回退冻结](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/FALLBACK_FREEZE.json)
  920文件、39个分析输入pin均未变；897个预存文件全部原字节不变。没有源码需要撤回，
  因为本轮没有把候选接入默认。

## 决策和保留边界

**该无条件替换不再进入性能门槛；没有计时、条件回放或推广。** 当前成对总体复用、
精确局部排除几何、原生补丁对比度快速判定三项已接受优化保持不变；已拒绝旧方案
未重新接入。后续候选仍需精确输出和中心/默认/正横偏总wall中位、均值全部严格改善。

没有重跑全量/current；此前33项缺历史工件失败及缺历史manifest导致的收集阻断仍在，
本轮定向通过不能冒充完整验证。默认no_motion、显式bounded_probe、安全阈值、
初始净空、速度、制动/恢复预算、.8s租约/.4s审计截止、模型/相机/标定均不变。
没有新物理/实车验收；**10Hz仍未达到，S2未通过，S6未完成**。

原始记录：[IPP开启](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/numerical/rows.jsonl)、[IPP关闭](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/fallback-numerical/rows.jsonl)。
执行命令：[初轮](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/COMMANDS.json)、[回退](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/FALLBACK_COMMANDS.json)。
重新执行需新建输出目录；不会覆盖本轮结果，也不属于本轮性能样本。
[交付清单](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/DELIVERY_MANIFEST.json)。
