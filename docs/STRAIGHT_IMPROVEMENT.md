# 直走能力改进

执行记录：2026-10-28—2026-10-29（Asia/Shanghai）。按用户要求只设五个大任务，每个完成后汇报；五项均已完成实现与评估交付；第三至第五项实验未推广，第五项不启用新的主动运动。工作交付不等于整体导航验收通过。

| 大任务 | 状态 |
| --- | --- |
| 1. 细分失败原因并审查现有许可链路 | 已完成 |
| 2. 补强启动许可和制动／恢复边界 | 已完成交付（检查未全通过） |
| 3. 分别验证自适应历史与控制调度 | 已完成交付（保留基线，实验未推广） |
| 4. 扩展轨迹相关的通行检查 | 已完成交付（覆盖验收未通过，实验未推广） |
| 5. 评估后向融合与主动恢复 | 已完成交付（主动动作未启用，实验未推广） |

计划：[五个大任务](/home/sunrise/toponav_project/docs/plans/2026-10-28-straight-improvement.md)。

第一任务未修改生产算法、阈值或17类模型；第二任务只改许可/停车契约，不改模型，旧验收结果不覆写。2026-10-18磁盘清理已删除历史图片和逐帧JSONL，因此诊断需在新目录复现已知旧条件，再核对保留的逐帧像素哈希、采集时间/身份及结果；不把这些旧条件当成新的独立测试。

## 2026-10-29 项目检查后增量：直走数值校验

本增量不是下方五项冻结实验的重做，也不修改它们的失败证据。
修复启动运动估计NaN/布尔值漏检和畸形前表面区间未验证的问题；停车确认使用同一数值检查。
启动纹理诊断新增逐帧`failed_checks`及原生/整流支持数量，门槛仍为九个原生单元全部支持、
至少六个整流单元及三前向/三横向分箱覆盖。已知默认夹具仍拒绝起步，四向水平相机近场盲区不变。
方法、验证及未解决部分见[第三批修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STRAIGHT_FIXES.md)。

## 第一个大任务交付

已查明：重复纹理的12条非零指令均由启动探测授权产生；正横偏/组合偏差被侧墙航向不确定度阻断，负横偏首先缺少地面跨视图空间支持，宽走廊/涂色主要受采样地面的唯一性和匹配门槛阻断。停车后的低视差常是后续恢复困难，不等于首发原因。独立S2、ROS与Gazebo的许可/看门狗边界已分别审查。

20个已知旧条件重新记录1534次决策；1532张解码RGB及新旧采集清单一致，各案最终控制与物理审计一致。最终完整回归1803通过，新增24项诊断/契约测试；236732项旧文件与205项历史冻结保真。没有修复生产控制器，也没有改变S2未通过或S6未完成的验收结论。

[完整报告](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task1/REPORT.md) · [许可链路审查](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task1/PERMISSION_AUDIT.md) · [验证记录](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task1/VERIFICATION.json)

## 第二个大任务交付

独立S2默认禁止运动，需显式受限探测授权、初始空间先验和持续RGB可观测性支持；增加0.14m探测预算及0.1598m条件停车余量。证据退化后先停车，至少0.1秒间隔的视觉时间窗口确认静止，再用连续三次新证据决定是否恢复；限制恢复次数与等待时间。共享MotorGate增加接受速度的保持超时，不能靠下一帧续旧速度。

最终冻结边界检查8/11满足、整体未通过；11案均安全终停、零碰撞、8次制动无提前恢复，三个普通正例有跟随进展但未完成导航，几何任务完成0案。两项故障条件未通过启动门控而未被实际触发，一项预算条件未进入跟随；原失败保留。当前tests目录1684项通过；全仓1857通过、1项历史生产策略文本对照失败（未改写/跳过历史测试）。S2未通过/S6未完成结论保持。

[第二任务报告](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task2/REPORT.md) · [验证明细](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task2/VERIFICATION.json) · [最终冻结边界检查](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task2/frozen-checks-v2/ASSESSMENT.json)

## 第三个大任务交付

新增显式H历史、C控制、HC组合实验，默认B不变；先以训练轨迹选择真实历史窗，再作一次原几何/通行检查，控制采用航向优先、中心带与平滑回中。许可、停车/恢复边界与模型不改。第二任务最终11案全部逐决策精确回放。

12次已知条件开发对照、44次冻结新条件对照及公共RGB回放完成。新条件四个普通直走：B 基线 0/4完成、69条跟随；H 自适应历史 0/4完成、143条跟随；C 平滑控制 0/4完成、38条跟随；HC 组合 0/4完成、122条跟随。未形成稳定任务完成收益，实验不推广；44次新条件边界检查全部满足，缺帧/冻结均在运动后实际注入并停住，近前表面停车及预算终止均实际覆盖；这些不混同导航完成。

当前tests目录1753通过；全仓1926通过、1项既有历史策略文本对照失败；新增69项测试。S2未通过/S6未完成结论保持，第四/第五项未开始。

[第三任务报告](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task3/REPORT.md) · [对照明细](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task3/independent-comparison.json) · [验证记录](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task3/VERIFICATION.json)

## 第四个大任务交付

2026-10-29交付。新增完整车体/轮子与相机外参声明、含延迟/不确定度的制动/纠偏扫掠包络、侧墙/前表面区间检查及RGB补丁实际投影的完整区域覆盖守卫，未知不作空闲。S实验仅约束跟随，原显式受限探测仍靠已有初始扫掠空间先验；默认B、H/C/HC及模型均保留。

完成4次已知条件和24次冻结新条件轮驱动对照/公共RGB回放。新条件安全审计零碰撞、零提前恢复；S边界测试6/12满足，四个普通正例无跟随，几何完成B/S均0/4。查明近场扫掠超出当前完整RGB补丁可视范围，代表案868个单元全未知；远处九个点不能充当区域覆盖。功能/覆盖验收未通过，S不推广。

新增52项测试，当前1805通过；全仓1978通过、1项既有失败。55条前任务运行1125次决策精确回放不变，243609项既存文件保真；S2/S6结论保持；当时第五项未开始，现见下方记录。

[第四任务报告](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task4/REPORT.md) · [验证记录](/home/sunrise/toponav_project/reports/straight-improvement-20261028-task4/VERIFICATION.json)

## 第五个大任务交付

2026-10-29交付。新增后向RGB质量/同一运动的留出预测一致性守卫，以及默认关闭的后向原生补丁覆盖比较；拒绝无条件投票、平均和盲区填充。主动观测只审查有限前移提案的停车、时间、距离、恢复次数及同帧扫掠前提，不连接速度执行器；未通过的提案保持停车并按原期限退出。既有多候选定位、默认B、MobileNetV3-Small和历史验收保留。

完成8次已知条件开发及56次冻结新条件四版闭环/公共RGB回放，共855条决策、192张实际终态帧、49次制动审计；零碰撞、零提前恢复。普通几何完成B/S/R/RA均0/4；R/RA无跟随，完整边界及研究检查仅6/14满足，不推广。四向完整补丁仍看不到车旁近场；候选0.2秒动作加保守保持/制动需1.17秒，无法装入原1秒恢复预算，未放宽该预算。RA去重54个当前帧提案审查，实际主动执行0次，不能称作主动恢复成功。

新增67项测试，tests目录1872通过；全仓2045通过、1项既有失败，无跳过。83次前任务运行1594条决策精确回放一致，245180项既存文件保真。五项工作完成交付，但原S2未通过/S6未完成状态仍在；不自动添加第六个大任务。

[第五任务报告](/home/sunrise/toponav_project/reports/straight-improvement-20261029-task5/REPORT.md) · [验证记录](/home/sunrise/toponav_project/reports/straight-improvement-20261029-task5/VERIFICATION.json) · [冻结比较](/home/sunrise/toponav_project/reports/straight-improvement-20261029-task5/independent-comparison.json)
