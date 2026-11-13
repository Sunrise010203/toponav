# 项目检查：第一批代码修复

日期：2026-10-29（Asia/Shanghai）。仅修复已复现的四类代码/入口问题，不将单元测试通过当作连续导航或安全验收。

## 修改范围

1. **ROS异常停车**：区分“控制器已中止”和“桥接层已发出中止零速”。感知抛错前已锁存aborted时也会发零速；重复桥接中止不重复发令。所有零速请求先发布，再写日志和诊断；非零命令仍先写决策记录。独立MotorGate租约不改。
2. **显式直走授权贯通入口**：Gazebo CLI/工厂、原生Qt界面、浏览器API/CLI/界面及公开RGB重放均传递 `motion_permission`。默认 `no_motion` 不变；受限探测必须单独选择，不能仅靠初始净空勾选自动授权。直走专用许可不能用于转角或路线。新的Gazebo直走记录绑定公开任务哈希，重放检查同一授权。
3. **终态锁存**：CornerController及ContinuousRouteController在检查后续包格式/时钟前先返回原终态零速，不让坏包覆盖完成/中止结果。
4. **帧龄校验**：NaN/Inf、布尔、非数值、负帧龄和超过1秒的帧龄均输出未知观测、置信度0；原有效时间边界、模型权重、0.6阈值和类别映射不变。

## 证据与限制

- 修改前源码快照：`/home/sunrise/toponav_project/.backups/review-fixes-20261029-190902`，含逐文件SHA256。
- 新增测试：`tests/test_review_safety_regressions.py` 与 `tests/test_review_motion_permissions.py`。
- ROS测试执行实际Navigation方法、实际连续控制器和MotorGate，只替换ROS传输构造器；不是DDS/三进程集成验收。覆盖感知抛错立即零速、写盘失败零速、重复中止、非零命令先记账。
- Qt测试使用真实控件，检查实际QProcess命令参数，但不启动Gazebo。浏览器API使用真实本地HTTP请求，子进程启动被替换；不将此当作实际驾驶成功。
- 历史报告/冻结套件/旧失败结果不修改。当前源码已演进，旧哈希不同仍应拒绝精确重放，不能更新旧清单强行通过。

本次运行日志、JUnit及验证记录存于：
`/home/sunrise/toponav_project/reports/project-review-fixes-20261029-QCOuCu`。

## 本次验证结果

- 新增71项回归测试全部通过，覆盖四类修复、HTTP授权传递、Qt真实控件/启动参数和公开任务哈希检查。
- 独立运行当前`tests/`：**1943通过、18条警告，退出码0**（84.87秒）；记录见`current-tests.log/xml/exit`。
- 全仓运行：**2115通过、2失败、18条既有依赖警告**。其中`tests/`的1943项均通过；报告目录174项中172项通过。没有跳过、xfail或修改收集配置来隐藏失败。
- 两项失败均为2026-10-19候选对当前live源码的文本对照：`7d-recovery`的StraightPolicy差异是修改前已存在的失败；`7d-navonly`的转角/路线片段对照因本次终态锁存修复新增失败。历史候选文件和断言原样保留，不据此宣称全仓回归全绿。详细日志见`regression.log/xml`。
- 已知默认API夹具（6m、seed313）的两次三tick物理短检查均终停、无接触、终态好帧保持锁存。`no_motion`返回`motion_not_authorized`；`bounded_probe`到达启动可观测性守卫后返回`startup_texture_unverified`，均未发运动指令。首轮脚本错误预期“显式授权必定产生非零运动”，其断言失败、退出码1和原日志保留；`smoke-summary.json`明确区分入口贯通与视觉守卫拒绝，没有放宽门槛或更换种子重跑成成功。
- 地图校验、公共Dijkstra和现有依赖一致性检查通过。未运行新的Gazebo/DDS完整导航、未进行正式S2/S6验收。

复核命令（工作目录为项目根目录；新JUnit必须另选路径，勿覆盖以上记录）：

```bash
.venv/bin/python -m pytest -q tests/test_review_safety_regressions.py tests/test_review_motion_permissions.py
.venv/bin/python -m pytest -q tests
# 含历史候选源码对照，当前预期仍显示上述两项失败：
.venv/bin/python -m pytest -q
```

## 尚未解决

- 普通直走任务完成率与车旁近场完整扫掠可观测性不足；后向实验和主动提案不推广。
- 模型对遮挡/unknown的错误接受率偏高，未重训或改阈值。
- 第一批结束时视觉出口结构与规划禁行状态仍需分层；第二批已完成代码分层，见[第二批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STRUCTURE_FIXES.md)。Gazebo模型闭环C02绕行仍未获得新的完整通过证据。
- 默认全仓pytest中的旧候选文本对照失败未改写或跳过；历史验收检查与当前回归尚未分层。
- S2/S6旧图像与逐帧日志缺失不能靠汇总/重新生成补回，S6正式审核未完成。

本次不启动长期实验流水线，不补签S2/S6、ROS2或实车验收。
