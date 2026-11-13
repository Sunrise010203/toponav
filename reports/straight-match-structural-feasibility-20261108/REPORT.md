# 直行优化收尾：模板匹配结构性可行性评估

2026-11-08，Asia/Shanghai。最多两轮计划的**第一轮已完成，第二轮不启动**。

## 结论

**停止当前这条零碎优化路线，保留全部已接受优化。**
本轮找到值得另立项目研究的内部复用位置，但没有形成同时具备完整精确性依据和
实质收益依据、可以接入当前默认实现的候选。不是证明“再也无法优化”，也不是候选
性能门槛失败；本轮没有候选，没有运行性能比较。无需为了凑满两轮再做一次试改。

只新增本目录的报告和只读审计脚本。2455个预存文件逐一SHA256核对相同，上一轮
8个最终pin也全部相同。没有生产、工具、测试、配置或旧报告修改，不需要恢复。
成对分数图/总体精确复用、局部排除几何、整数矩对比度判定、齐次投影分配均保留。

## 依据：不重复剖析，检查最大热点内部的复用边界

已有完整正横偏诊断每遍126105次实际匹配，其中114756次位于成对描述子内部；
模板匹配排他占比46.97%。这是历史仪器遍的归因，不是本轮时延，也不是新收益。
H/I准备合计约0.67%，继续在那里做零碎替换缺乏优先级依据。

检查当前配对实现后确认：画布float32准备已经区域内复用；相同逻辑角色的完整分数图
已在单轨迹内精确共享，轨迹结束清理；相同总体也已共享归约。
不能把已完成的共享再次列作新方案，也不重新开启已拒绝的补丁/NCC缓存、scratch、
输入类型替换或其他旧候选。

本轮只读检查归档的OpenCV 4.10.0上游源码及当前安装元数据：

| 结构性方向 | 源码依据与潜力 | 当前不进入第二轮的原因 |
|---|---|---|
| 同一画布的积分图、窗口矩复用 | `common_matchTemplate`第931行每次创建sum/sqsum；第980–1029行窗口矩参与归一化。同一画布、模板尺寸相同时有不依赖模板的量 | Python `matchTemplate`不能传入预备积分图。要拆开相关与归一化或修改原生实现，需证明运算顺序、特殊分支、舍入和后端回退均与现有库一致；目前没有完成这一证明，也没有单独耗时占比，不能把整个匹配46.97%都算作可省部分 |
| 同一画布的DFT准备复用 | 通用`crossCorr`第566行起，每次创建变换缓冲，后续准备模板和图像DFT；固定尺寸画布存在共享图像准备的可能 | 当前库启用IPP，相关优先走IPP，失败可回退通用路径。不能把通用DFT路径当作所有调用的实际路径，也不能为方便复用强制切后端；现有Python接口不暴露这些内部工作区 |
| 多模板批处理 | 有机会减少重复准备与Python边界开销，但收益未测 | 按通道堆叠会改变输出语义/分派；拼接或分块会改变边界及变换布局，不能直接推导为每张118×118分数图逐位相同。保留原调用的包装本身又不消除内部重复准备；需要真正的原生批处理设计 |

以上复用若将来实现，仍须执行所有原逻辑匹配、搜索位置和证据检查，只复用不变的
内部准备；不能缓存掉实际匹配或缩小搜索范围来获取收益。

### 为什么不能直接把归一化改写成NumPy公式

上游实现不只是相关系数的数学公式。它保留float32原始相关结果，以double依次计算
窗口和模板矩，包含低方差判定、`fabs(num) < t`、`t*1.125`饱和/零分支，最终写回float32。
代数等价不保证逐位等价；向量化sqrt、表达式重排、融合运算或不同后端也需单独论证。
使用`TM_CCORR`取得原始相关再做预备归一化，是**可能的后续研究设计**，不是已证明
或已测速的候选。本轮不临时写一个近似实现，再用少量样本通过冒充完整精确性依据。

已有float32→uint8反例也表明不能仅凭整数像素值相同或IPP开启推断全域精确；不重测
该拒绝路线，不用容差、接受bool或最终控制器状态替代完整分数图精确要求。

## 部署边界核查

- 当前Python运行时为OpenCV **4.10.0**、NumPy **1.26.4**；默认`pkg-config opencv4`
  返回 **4.5.4**。因此直接按系统开发库构建扩展，不等于接入当前Python库的同一实现。
- 当前构建信息显示`Built as dynamic libs?: NO`，使用隐藏可见性；Python公开入口只有
  `matchTemplate(image, templ, method[, result[, mask]])`，没有预备积分图/DFT或批模板参数。
  这不证明任何原生扩展都不可能，只说明没有现成的同构建内部复用接口。
- 上游版本标签的源码**不是安装二进制来源证明**；本轮记录二进制SHA256，不混用
  不同库版本来声称精确性。
- 只读查询到线程数12、IPP/optimized开启；未调用任何设置接口。没有计时，所以没有
  为审计改成单线程；后续若有正式比较仍必须只将OpenCV设为1线程并在退出恢复。

## 建议的后续工作：另立原生匹配项目，不再追加微调轮次

1. **先解决可复现构建与同库调用边界**：记录当前安装包、构建参数和IPP依赖来源；
   不把系统4.5.4当成当前4.10.0的可替换链接库，不直接覆盖运行环境。
2. **限定第一项原生设计**：同画布、同尺寸的积分/窗口矩准备复用，必要时增加批模板
   入口；保留原始相关调用、原标量归一化次序、特殊分支和后端失败回退语义。
   DFT共享仅在证明适用的原路径内评估，不强制后端、不改变线程或调度。
3. **先精确性，后性能**：事前冻结支持范围与失败回退，逐张核对完整分数图位模式、
   完整H/I证据及独立拟合输出，再决定是否具备进入完整计时的依据。有限样本相同不
   代替实现层面的等价论证；不适用输入必须保留原实现。
4. **门槛不变**：若将来形成候选，中心/默认/正横偏三条完整序列总wall中位和均值
   六项必须全部严格改善；仍是一次固定串行AB/BA比较，保留慢帧，失败即恢复且不补跑。
   通过后才做固定13例旧记录回放；旧记录回放不等于物理验收。

这是需另行确认范围的架构工作建议，不是在本次最多两轮承诺内已经启动的改造，
没有承诺其收益或完成轮数，也不承诺能达到10Hz。

## 本轮验证与验收状态

- 本轮控制器/数值候选/剖析/计时/回放/物理运行均为0；测试运行也是0。没有新生产
  改动，所以使用源文件pin核对交付，不复跑旧失败集合来制造“进展”。
- `AUDIT.json`和独立再次运行的`FINAL_VERIFICATION.json`记录静态定位、安装元数据、
  全部预存pin及上一轮最终pin检查；这不是数值等价或性能测试。
- 沿用而非新测的最近状态：定向459通过，扩大1630通过、33项既有缺工件失败；
  标准current仍因缺`reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json`
  而exit2、selected0、verification_complete=false。本轮没有补造或修复这些工件。
- **10Hz未达到，S2未通过，S6未完成，没有新实车/物理验收。** 默认no_motion、显式
  bounded_probe、安全阈值、初始净空、速度、制动/恢复预算、.8s租约/.4s审计截止、
  模型、相机、标定、训练/留出均不变。

## 可复核工件

- [固定范围](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/PLAN.json)
- [只读审计源码](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/audit.py)
- [审计结果](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/AUDIT.json)
- [最终再核对](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/FINAL_VERIFICATION.json)
- [预存文件pin](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/BEFORE_PINS.json)
- [交付清单](/home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/DELIVERY_MANIFEST.json)

复核命令（输出必须使用不存在的新路径，不覆盖已交付结果）：

```sh
/home/sunrise/toponav_project/.venv/bin/python \
  /home/sunrise/toponav_project/reports/straight-match-structural-feasibility-20261108/audit.py \
  /absolute/new/audit-output.json
```

历史依据：[完整热点诊断](/home/sunrise/toponav_project/reports/straight-current-workload-20261104/REPORT.md)、
[输入表示反例](/home/sunrise/toponav_project/reports/straight-match-representation-20261104/REPORT.md)、
[最近一次拒绝与恢复](/home/sunrise/toponav_project/reports/straight-ncc-unit-clip-20261104/REPORT.md)。
