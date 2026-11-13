# 直行持续证据/恢复：组合实验及完整链路剖析

日期：2026-11-03（Asia/Shanghai）。**没有获得普通直行完成收益，未推广新策略。** S2未通过，S6未完成。

## 本轮目标与记录顺序

1. 先冻结未修改生产策略的开发复现计划/源码（DEVELOPMENT_PLAN、BEFORE_PINS），在已知4m/seed31013条件录制baseline/reference/gradient_transfer的完整连续RGB、每次决策及独立物理轨迹。三个最终几何完成均否，位移0.6366/0.6366/0.7707m；未把重建开发条件称为未见测试。
2. 使用既有tools/diagnose_straight_task1只读公共决策诊断。默认/reference先在tick33因竞争匹配停车，tick47缺地面跨视图支持，tick55后走廊区间过宽；停车至tick62后位移基线不足。gradient先在tick42流检查失败，tick54/64地面留出失败。不是所有首发失败都源于低视差。
3. 只读gradient录制的20个waiting帧，使用原训练轨迹历史选择器先选一个5/9/13/17窗口，再执行一次原宽侧向几何+梯度通行。原九帧联合支持12/20，所选窗口14/20；这只是固定录制上的反事实检查，不是新闭环或恢复成功，不能用它预告功能收益（WINDOW_PROBE及progress原包保留）。
4. 冻结最初组合意图CANDIDATE_INTENT，新增显式gradient_history。候选契约测试后细化固定10次开发比较运行矩阵CANDIDATE_PLAN并冻结CANDIDATE_PINS，随后串行运行/回放/独立物理审计。不根据比较结果改参数、重跑到通过或删除失败。

## 代码边界

新模块roadvision/gradient_history_straight.py继承原gradient_transfer，仅将缓冲扩为17并接上既有训练轨迹选窗；不重做模型或描述子，也不读取最终/留出接受结果来挑窗口。
无合格窗口就未知，选中后原一次拟合拒绝就停车，没有九帧回退、旧结论重用或同帧寻找通过者。
恢复静止确认仍三张零请求后真实采集、至少.1s间隔；1s等待、三次重新确认、恢复次数、探测、速度、制动及0.8s/0.4s门控不变。
默认StraightController、原gradient/宽侧向/历史选择器的源码均未改变；已有文件只修改运行器/公共精确回放的显式变体选择和离线profiler选择/绑定。
新变体不能混入转角/路线，任务与回放强绑定；所有输出promoted=false。它改变历史和接受集合，不是等价计算提速。新增25项契约测试；输出的选窗记录/策略无外部可写别名。

## 冻结后开发比较（不是正式导航验收）

| 条件/版本 | 决策数 | 跟随指令 | 位移m | 几何完成 | 终止原因 |
|---|---:|---:|---:|---|---|
| known_center--gradient_transfer | 75 | 46 | 0.77070 | 否 | evidence_recovery_timeout_geometry_or_free_path_unverified |
| known_center--gradient_history | 39 | 8 | 0.20496 | 否 | evidence_recovery_timeout_awaiting_stationary_and_fresh_rearm |
| known_default--gradient_transfer | 40 | 9 | 0.22435 | 否 | evidence_recovery_timeout_geometry_or_free_path_unverified |
| known_default--gradient_history | 25 | 1 | 0.10513 | 否 | evidence_recovery_timeout_awaiting_stationary_and_fresh_rearm |
| known_offset_plus--gradient_transfer | 80 | 55 | 0.88799 | 否 | recovery_count_exhausted |
| known_offset_plus--gradient_history | 83 | 52 | 0.84424 | 否 | evidence_recovery_timeout_awaiting_stationary_and_fresh_rearm |
| fault--missing | 13 | 4 | 0.14988 | 否 | missing_frame |
| fault--freeze | 15 | 6 | 0.17994 | 否 | commanded_motion_without_rgb_response |
| refusal--no-permission | 1 | 0 | 0.00003 | 否 | motion_not_authorized |
| refusal--repeated | 3 | 0 | 0.00004 | 否 | startup_texture_unverified |

三组普通条件两版均0/3完成。组合版三个条件的行进距离全部更低；不能推广，不能把“停止更早”算功能增强。
窗口选择虽然只看训练证据，仍可能选出最终描述子/留出几何不支持的窗口；完整流水线必须拒绝。
组合中心案例先在tick14留出失败，之后间歇恢复；末次等待中tick32—35无合格窗口，tick36/37仅两次新好证据，tick38按原1s期限退出。没有延长超时来凑足第三次确认。
这说明离线14/20高于12/20不能预测闭环改善，训练分数不等于最终证据可靠性，增大历史缓冲也不等于增加有效运动基线。

10次开发运行全部独立物理审计errors=[]、0接触、最终静止；最大完整制动0.166667仿真秒/0.012183m。
374条公共RGB决策、最终结果及30张真实终态好帧精确回放；missing在运动后tick12即时零并终止，freeze在运动后实际注入并以commanded_motion_without_rgb_response终止。
未授权和重复纹理0运动命令拒绝，物理静态沉降约3—4e-5m不伪装成精确0位移。
这些是限定场景的仿真制动/故障记录，不是实车反应时间、OS活性或普遍重复纹理保证。到视觉停止也不代表节点到达；本轮普通案例甚至没有视觉完成。

## 完整控制器剖析

对中心父版75次/组合39次完整顺序输入，独立执行普通计时、阶段剖析、cProfile各一遍；342次输出逐项精确一致。输入及源码见PROFILE_PLAN/PROFILE_FREEZE；不读取result/物理真值来选择控制。
普通计时仅controller调用，图像已解码；输出比较在计时外。OpenCV单线程并结束还原，BLAS未调参。无预热、每序列一轮，以下不是性能准入、前后同输入提速或部署SLA。

| 完整序列 | 普通调用分母 | 中位ms | p95 ms | 最大ms | 超0.1s |
|---|---:|---:|---:|---:|---:|
| gradient_transfer | 75 | 564.51 | 774.27 | 1121.79 | 68/75 |
| gradient_history | 39 | 597.05 | 790.06 | 818.18 | 32/39 |

两序列的输入/长度不同，不能用组合总耗时更短宣称提速。普通0.1s目标仍明显未满足。
阶段与cProfile存在仪器开销，不能与普通计时混用：父版阶段exclusive中模板匹配18.07s、配对描述子本体17.50s最突出，cProfile显示117962次实际matchTemplate。
下一优化应优先审查这一重复描述子计算链，而非继续堆叠策略、减少留出检查或放宽恢复预算；选点/支持/评分/拒绝门槛必须保留。

## 软件回归及兼容性

- 候选相关4文件170项通过，含25项新增；不是额外170个独立场景。
- 扩大到34个明确列出的直行/通行/历史/描述子测试文件：1038通过、33失败、1warning，无skip/xfail。33项与上一轮失败节点及错误消息完全相同，均缺失历史multi-node/control-safety报告文件（expanded-command/log/junit）。这不等于完整current或all范围通过。
- 为避免已知ROS launch自动插件把未选择的整个tests提前导入，以上显式子集命令仅禁用launch_testing与launch_ros，未更改pytest配置/默认发现行为或历史断言。
- 仓库标准tools/run_project_tests.py --scope current另按默认插件执行；原始日志/审计位于reports/straight-evidence-recovery-full-current-20261103，缺历史封存文件仍阻断收集，verification_complete=false；不将失败白名单化。
- 新增变体后，对最初三条baseline/reference/gradient_transfer完整录制作当前源码精确回放207条决策/9张终态好帧，并独立复算物理审计。旧默认/已有实验行为不变（OLD_CURRENT_REPLAY）。这只是本轮当前录制的兼容性回放，不是已删除历史验收数据的恢复。
- CANDIDATE_PINS在比较、剖析、回归结束后再次相符；原roadvision文件、历史选择器、配置及fixture不改。before目录保留三份修改前集成源码，SOURCE_DIFF为增量。

## 入口和限制

[使用/实现说明](/home/sunrise/toponav_project/docs/STRAIGHT_GRADIENT_HISTORY.md)。所有实验运行目录独占创建；完整条件见CANDIDATE_PLAN，勿改旧pins或覆盖旧记录。
组合实验**仅为复现保留，默认不启用、不推荐替换原梯度版本**。
旧四普通条件0/4、近场盲区、窄障碍覆盖、受信pending/曝光来源和旧IPC故障均未修复；不动2AB等离线协议，不接生产native/Gazebo/设备。
本轮完成的是一项未获功能收益的开发验证和完整链路定位，不是整个直行任务完成。
