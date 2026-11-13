# 项目检查：第二批修复——结构与通行权限分层

日期：2026-10-29（Asia/Shanghai）。承接第一批修复，不启动长期实验、不补签历史验收。

## 问题与改动

旧`map_exits()`将禁行边和禁止逆行的边从相对出口签名中移除；理想结构传感器也读取通行标志来删除观测。
这把“看见一个物理通道入口”与“允许驶入该通道”混为一谈。Gazebo中C02阻挡位于路段中部，入口仍然可见，旧模型闭环因此在N02之前结构不匹配并停车。

- `simulation/structural_localization.py`：`map_exits()`只预测物理开口，保留两端的物理结构；第三个`disabled_channels`参数为兼容旧调用保留，但不改变结构结果。新增`traversable_exits()`，在运动假设传播中继续过滤禁行、停用及旧版地图的单向限制。公共规划器和动作安全检查不修改。
- `simulation/structural_navigation.py`：理想传感器仅按独立场景通道端点预测结构，不读取规划禁行状态。执行逻辑不修改，直发禁行命令仍失败、位姿不变。
- 真实模型概率、0.6阈值、模型权重、几何守卫、运动许可、ROS MotorGate及超时均不改变。观测真的与物理签名不符时，仍返回`observation_mismatch`并停车，不将遮挡伪造为可见。

这是一种拓扑层的“通道入口”模型，不证明中远段可通行，也不能完整模拟阻挡恰好封住入口时的RGB。后者若造成观测冲突必须拒绝，不能由禁行标志替代感知。

## 回归与诊断范围

新增`tests/test_review_structure_permissions.py`，覆盖持久/临时禁行、四朝向、全部64种禁行子集、v2及legacy结构传感器、旧版单向边、禁止转移传播、旧路由失效、不可达停车、直接禁行命令拒绝、离散及三帧时序绕行。
模型控制器测试使用明确标注的假预测，仅验证真实`_matches`/`_authorize`方法和公共规划的接口，不验证模型精度或连续驾驶。

修改前，选定的19项新回归测试全部失败；修复后结果与全仓日志存于：
`/home/sunrise/toponav_project/reports/project-review-structure-20261029-g56ldqx6`。
修改前源码及哈希：`/home/sunrise/toponav_project/.backups/review-structure-20261029-192209`。

### 本轮结果

- 新增185项测试全部通过（独立运行6.11秒，退出码0）。最初181项通过后，又补充4项直接禁行执行拒绝测试；两轮日志均保留。
- 全仓：**2300通过、2失败、18条依赖警告**（111.11秒）。JUnit按来源拆分：当前`tests/`为**2128项全部通过**；历史报告测试为172通过、2失败。没有修改收集配置、跳过或xfail测试。
- 两个失败仍是第一批已记录的`7d-recovery`与`7d-navonly`历史候选对当前源码的文本对照；本批没有新增全仓失败，不据此宣称全仓全绿。
- 地图/场景校验及公共Dijkstra的C02禁行绕行检查通过；权重SHA256仍为`1188f35dd20d0773b780b73f2000daa25fede9b2db098b02dc0746ad0de6b03b`。

复核命令（工作目录为项目根目录；输出日志请另选新路径，勿覆盖本轮记录）：

```bash
.venv/bin/python -m pytest -q tests/test_review_structure_permissions.py
.venv/bin/python -m pytest -q tests
# 包含两项已知历史文本对照失败：
.venv/bin/python -m pytest -q
.venv/bin/python -m simulation.topology
.venv/bin/python -m simulation.dijkstra --disable C02
```

### 旧失败图像的只读开发诊断

`inspect_recorded_structure.py`按记录顺序选择旧C02失败运行中第一张原本被接纳为`exits_1101`的图像（tick159），验证原像素哈希后，用真实冻结模型及原采集时间重新推理。
只在显式给定的假设`N02/0°`上比较结构签名：旧代码缺少front并拒绝该候选，新代码保留front/left/back，公共规划仍选择left经N05绕行、不包含C02。

此诊断**没有推断车辆已到N02，没有发运动命令，不是逐项历史决策重放，也不是新独立图像评测**。
原运行仍是未完成、`model_evidence_hold_timeout`；原报告、RGB、JSONL及freeze原样保留。旧freeze与当前源码不一致仍应拒绝精确重放，不更新清单强行通过。

## 未解决与验收边界

- 尚无新的真实Gazebo连续C02绕行成功证据；理想离散/时序绕行不等于像素驱动任务完成。
- 直走启动可观测性、近场扫掠、普通任务完成率问题不变。
- unknown/遮挡误接纳问题不变，未重训、未降低拒识门槛。
- 第一批全仓测试中的两个历史候选文本对照冲突不通过改写历史文件或跳过测试解决。
- 缺失的S2/S6历史原始输入不补造，S2未通过、S6未完成状态不变；未做ROS/DDS或实车验收。
