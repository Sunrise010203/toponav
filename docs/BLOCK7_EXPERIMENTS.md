# 第七块实验与成果：P/N阶段交付

日期：2026-10-19（Asia/Shanghai）。**7A—7C完成，7D原子离散N实验及复核完成，7E现有P/N阶段成果交付完成；连续/ROS未授权未执行，因此7D整体与整个第七块未完成。** 第六块继续停止，S2未通过、S6整批独立评估未完成，不补签安全验收。

## 1. 已实际完成的范围

| 轨道 | 验证 | 最终测试 | 状态 |
|---|---:|---:|---|
| P冻结17类感知 | 504包 | 2016包 | 原始记录、统计和复算完整 |
| N无逐点参考照片原子导航 | 1584次 | 3168次 | 4752次全审计、0运行错误/未运行 |
| 连续控制 | 仅预算24条件 | 尚无精确冻结套件 | 未授权/未执行 |
| ROS | 仅预算8条件 | 尚无精确冻结套件 | 未授权/未执行 |

P的一个四视图包是一个观测，不按4个独立场景计数。18个布局族按validation6/test12隔离，含3个节点规模档。N各方法均使用公共拓扑、起点/目标/初始朝向；不包含等价“无拓扑”对照。历史连续轨采用参考RGB先验，与本N轨不同。

## 2. 主要结果（最终测试）

- 感知已知准确率1526/1536=99.35%；布局宏固定17类F1为94.54%。但unknown错误接受124/480=25.83%，其中遮挡90/96=93.75%，不能称为可靠拒识或安全泛化。缺视图96包由输入守卫拒绝，另1包原帧龄拒绝保留。
- M0（1帧/无恢复）正常成功172/180；M1（3帧/恢复）180/180；M2（1帧/恢复）180/180；M3（3帧/无恢复）164/180。
- 预声明主比较M1−M0：**+4.444个百分点，95%配对成簇CI[1.667,7.222]**；仅支持本离散工作域正常任务的机制组合改善。M1−M2为0，不能据此证明三帧确认的独立额外收益。
- 并非所有条件均占优：外观M1=68/72、M2=72/72；错误地图M1/M2均24/72；执行状态不确定时所有方法0/72。临时封路M1=72/72、M2=2/72，受首3采样槽故障与确认时序耦合限制，不推广为任意物理封路能力。
- 正常任务采样槽中位数M1/M3为12，M0/M2为4；成本与成功率同步披露。会话墙钟不含模型初始化，纯前向/预处理/采样链路、模型加载单列，非硬实时保证。
- 本批N全审计观察误到达0、错位姿发令0，不意味着零风险或连续物理安全。恢复“可靠重新确认”不等于任务到达、环境修复或执行不确定消失。

CI依原协议按12个最终测试布局族、3档分层、10000次bootstrap、seed7101801；不把帧/种子作独立场景，小簇数、退化区间和探索性多重比较限制保留。池化计数与布局宏值分开。

## 3. 交付入口

- [阶段实验报告](/home/sunrise/toponav_project/reports/block7/7e-20261019/EXPERIMENT_REPORT.md)
- [7E验收与限制](/home/sunrise/toponav_project/reports/block7/7e-20261019/ACCEPTANCE.md) · [7D离散最终复核](/home/sunrise/toponav_project/reports/block7/7e-20261019/7D_N_REVIEW.md)
- [汇总与分母](/home/sunrise/toponav_project/reports/block7/7e-20261019/summary.csv) · [逐族数据](/home/sunrise/toponav_project/reports/block7/7e-20261019/family_metrics.csv) · [CI与配对比较](/home/sunrise/toponav_project/reports/block7/7e-20261019/confidence_intervals.json)
- [成本与计时](/home/sunrise/toponav_project/reports/block7/7e-20261019/costs.csv) · [恢复及删失](/home/sunrise/toponav_project/reports/block7/7e-20261019/recovery_diagnostics.csv)
- [全量诊断目录](/home/sunrise/toponav_project/reports/block7/7e-20261019/FAILURE_CATALOG.csv) · [6张图](/home/sunrise/toponav_project/reports/block7/7e-20261019/figures/)
- [只读复现命令](/home/sunrise/toponav_project/reports/block7/7e-20261019/REPRODUCE.md) · [交付SHA清单](/home/sunrise/toponav_project/reports/block7/7e-20261019/DELIVERY_MANIFEST.json)

原始记录分别在[7C](/home/sunrise/toponav_project/reports/block7/7c-20261018/ACCEPTANCE.md)和[7D目录](/home/sunrise/toponav_project/reports/block7/7d-20261018/)。原7D的STATE保留自动流水线结束时“待复核”历史，后续只读复核结论以本次7D_N_REVIEW为准，不改历史文件。

## 4. 复现、保真与未完成项

从原始记录复算的21份P/N产物与旧正式交付逐字节一致；本次独立统计230项交叉核对一致，另在新目录复算全部表/JSON/报告/6张图。最终28项资格测试、完整后回归1611项及46项交付检查通过，129801项旧文件SHA保真，详见验收，不用前测冒充后测。原2040张正式图、模型、源码和旧报告只读保留。

N曾在用户指令下暂停，并于2026-10-19明确续跑，完成前缀与首次生成凭据保留；一例无RGB/结果的模型加载期中断，空现场归档后重启预备进程。该偏离完整披露，不包装为新的未见测试。开发版本与中间检查也保留。

若后续继续连续/ROS，必须先取得明确授权，解决或限定S2/S6资格缺口，冻结精确案例/扰动锚点/参考先验，再做独立物理/停车/租约证据审计。本次没有碰撞/制动/ROS新曲线，不宣称实车、安全或硬实时能力。新候选须新未见测试，不能据此调参后重用旧测试声称独立改善。
