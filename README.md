# 先验拓扑视觉导航：统一系统 v2

> **项目整理快照：2026-11-13。** 本仓库是研究/仿真项目，不是已完成实车验收的导航产品。
> 当前 **S2 未通过、S6 未完成、10Hz 未达到**；下方历史通过数不能替代本次验证结果。

## 项目入口

- [项目总览与模块地图](docs/PROJECT_OVERVIEW.md)：架构、目录职责、推荐阅读顺序及当前边界。
- [本次整理与验证记录](docs/REPOSITORY_STATUS_2026-11-13.md)：上传范围、缺失工件、测试结果和推送前置条件。
- [测试范围与严格审计规则](docs/TESTING.md)：区分当前回归、历史快照和正式导航验收。
- [最近一次结构性可行性审计（2026-11-08）](reports/straight-match-structural-feasibility-20261108/REPORT.md)：只读结论，不是新增性能收益。

### 最小运行（已有依赖环境）

在项目根目录执行；纯拓扑/结构离散示例不需要启动 Gazebo 或 ROS：

```bash
.venv/bin/python -m simulation.topology
.venv/bin/python -m simulation.dijkstra
.venv/bin/python -m simulation.structural_navigation
```

环境基线是 Ubuntu 22.04 / Python 3.10；依赖安装和仿真准备见[项目总览](docs/PROJECT_OVERVIEW.md)。
现有 `datasets/`、冻结模型和 `reports/` 纳入版本控制；虚拟环境、缓存、仿真运行时和 ROS 构建产物不上传，留在本地。
本次仅整理文档和版本控制边界，不改导航算法、安全阈值、冻结报告或测试发现规则。

### 本次验证摘要

- 标准 `current`：退出 **2**，缺失 `reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json` 导致收集失败。
- 标准 `all`：退出 **2**，还存在历史候选快照的模块重名及缺失路径；**不能宣称全仓通过**。
- 完整日志和独立的小范围 smoke 结果见[本次状态记录](docs/REPOSITORY_STATUS_2026-11-13.md)。

---

以下保留原开发记录。它们按各自日期和环境解读；旧记录中的本机绝对路径、已缺失的报告引用与历史测试数量不表示本仓库具备完整历史归档。

## 2026-11-01 普通直走增量：联合参考选点与标量接口

新增显式`--straight-variant reference`，在原1cm候选范围内以同一参考RGB联合选择原生/整流支持，不降低数值门槛、不查看目标帧重试。默认启动策略不变；四个冻结后生成的普通条件跟随指令9→48，但两版几何完成仍均0/4，不推广实验版本。
另修S2前表面区间的NumPy float16/32/64到等值Python float接口，严格消费端、NaN/布尔拒绝和制动门槛不放宽。近场完整扫掠仍缺证据；模型、安装与租约不变，S2未通过、S6未完成。方法、运行与证据见[本轮说明](/home/sunrise/toponav_project/docs/STRAIGHT_REFERENCE_FOLLOWUP.md)。

## 2026-10-31 第三十六批：订阅注册失败时拒绝启动

概览、统计、时钟、姿态和五个接触通道的注册返回值不再被忽略；任一失败即在创建执行器和步进助手前拒绝启动，保留明确失败topic。
原接收状态/相机订阅门槛及回调、消息类型、顺序不变；注册成功不是传感器持续健康、消息完整或无碰撞证明。模型、租约及正式验收状态不变。
详见[第三十六批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_SUBSCRIPTIONS.md)。

## 2026-10-31 第三十五批：空异常文本不能绕过故障门控

相机回调对空异常文本使用异常类型名；取帧、非零命令及最终完成检查以`error is not None`判定故障，不再依赖文本真值。
保留首故障、主异常优先、零速/STOP与新鲜停稳事实；合成`MemoryError()`验证不代表真实内存耗尽恢复能力。模型、租约、阈值和正式验收状态不变。
详见[第三十五批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_CALLBACK_ERRORS.md)。

## 2026-10-31 第三十四批：相机输入预校验与故障拒绝门控

相机在获取像素载荷、创建NumPy视图/副本之前校验尺寸、格式和步长；方向128、上下256、概览384×384或768×448的既有契约不变，通用解码限定最大宽768、高448。
载荷长度校验后才复制像素；回调保留首故障并通知等待者，概览出错清除旧图。已知I/O故障拒绝后续非零提交及任务完成，零速/STOP和独立的新鲜停稳事实保留。
这是解码入口约束，不是原始传输、protobuf分配、磁盘或全进程资源有界；模型、租约和验收门槛不变。
详见[第三十四批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_IMAGE_INGRESS.md)。

## 2026-10-31 第三十三批：独立执行器防重放历史容量上限

Python独立执行器每会话最多保留16384个已接受帧引用，不淘汰；耗尽后在arm/accept和新记录分配之前拒绝命令并锁存零速，保留上一授权元数据及期限。
包括终止命令在内的新帧证据不能越过容量门控；零速锁存及无需帧的STOP/shutdown仍可用，不把安全停车等同于接受终止证据或导航完成。
原生接收端仍保持4096帧上限；Python容量兼容原有8193次零速IPC审计负载。失败客户端仍不可恢复；原独立停车观察只证明新鲜停稳，不覆盖普通清理失败。模型、租约、队列、超时和验收门槛不变。
详见[第三十三批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_ACTUATOR_FRAMES.md)。

## 2026-10-31 第三十二批：接触审计历史容量上限

生产接触审计最多保留65536条观察到的墙体接触记录，不滚动淘汰；达到上限后，下一条接触输入触发粘性容量故障，不再解析名称/时间戳或分类该输入。
耗尽后拒绝新非零命令和任务完成，零速/STOP及原停车流程保留；省略计数是未校验条目数，不能当作实际墙体接触数。
接触审计不完整不抹掉独立的新鲜停稳事实；仍不认证无碰撞，也不是原始消息/名称长度、帧、传输、磁盘或全进程资源有界。
详见[第三十二批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_CONTACT_HISTORY.md)。

## 2026-10-31 第三十一批：执行审计事后读取容量上限

审计摘要不再整文件读取：总输入16MiB、单个LF块16KiB、最多65536条逻辑记录；超限拒绝完成，不淘汰前缀或用尾部补证。
只在事后路径读取普通文件，拒绝符号链接/FIFO；原发布循环、队列、超时、租约与停车策略不变。
这不是写入/磁盘/传输或全进程资源有界，也不解决旧自然压力故障；正式验收未恢复。
详见[第三十一批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_AUDIT_SUMMARY_BOUNDS.md)。

## 2026-10-31 第三十批：审计故障补充文件纳入完成拒绝门控

退出后发现合法、无效或不可读的`.failure.json`时，即使执行日志完整且末态绑定，也拒绝任务完成；不存在补充文件不额外授予健康证明。
只在事后摘要最多读取1025字节，不跟随符号链接、不等待FIFO；发布循环、队列、超时、租约与停车策略不变。
预先限定的4轮自然IPC压力均未复现旧故障，旧根因仍未知；受控退出延迟只用于验证新拒绝门控，正式验收未恢复。
详见[第三十批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_AUDIT_FAILURE_GATE.md)。

## 2026-10-31 第二十九批：IPC审计首故障分因诊断

审计失败现在区分队列满、socket背压、确认超时、坏确认及写进程退出等首个原因；只保存固定小摘要，不在发布/租约循环写诊断文件。
原生worker退出并完成零速锁存及有界回收后，才另存`.failure.json`补充诊断；它不构成授权、完整审计或停稳证明。
64项队列、0.4秒审计超时、0.8秒租约和所有拒绝策略不变；上批未记录的压力故障根因不能被新诊断倒推，仍待定位。
详见[第二十九批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_AUDIT_DIAGNOSTICS.md)。

## 2026-10-31 第二十八批：Gazebo姿态历史容量上限

生产姿态日志最多保留65536条robot姿态的完整前缀，不滚动淘汰；下一条匹配输入触发容量故障，停止解析和留存该姿态。
耗尽后拒绝非零提交，零速请求及原停车清理仍可执行；旧姿态不能再证明新鲜停稳或任务完成，摘要明确披露不完整性。
只限制姿态前缀，不等于接触/帧/传输/磁盘或整个进程有界；模型、租约与停车门槛不变，正式验收未恢复。
详见[第二十八批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_POSE_HISTORY.md)。

## 2026-10-31 第二十七批：Python实际发送日志容量上限

生产接收证据保留最多16384条实际发送回调的完整日志前缀；下一次回调触发粘性故障，不复制或校验遗漏输入、不补建候选，不允许据此继续非零运动或确认完成。
容量耗尽不让发送后观察者关闭健康执行通道，零速/STOP及原停车清理仍可用；摘要明确披露前缀截断与未校验回调计数，不能把保留长度当完整发送总量。
原生4096帧、候选8192项与状态65536条的上限不变；不是姿态、传输、磁盘或整个进程内存有界，模型与安全门槛不变。
详见[第二十七批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_SUBMISSIONS.md)。

## 2026-10-31 第二十六批：Python发送候选映射容量上限

生产接收证据最多保留8192项已发送非终止命令候选的完整前缀；超限后锁存故障，不能再放行非零命令或支持任务完成。
不淘汰、不重建遗漏候选，旧反馈不能冒充最新发送绑定；发送后观察者不因容量耗尽抛错，零速/STOP及原停车清理仍可使用。
原生已接受帧上限仍为4096；本批不限制实际发送日志、姿态日志或整个进程内存，不改变模型与安全门槛。
详见[第二十六批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_CANDIDATES.md)。

## 2026-10-30 第二十五批：Python接收状态历史容量上限

生产接收证据最多保留65536条有效状态的完整前缀，不滚动淘汰；下一条输入触发粘性故障，不能再成为新鲜反馈或支持任务完成。
超限后的输入停止解析/留存并作饱和计数，原始故障不被覆盖；零速请求、发送记录及原停车清理继续可用。
只限制接收状态历史，不代表发送/姿态日志、传输队列或整个进程有界；原生库、模型与安全门槛不变。
详见[第二十五批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_HISTORY.md)。

## 2026-10-30 第二十四批：原生防重放帧历史容量上限

原生接收端每会话最多保留4096个已接受帧引用，达到上限后拒绝新授权并锁存零速，不淘汰旧证据、不自动重置或重新授权。
STOP不消耗帧名额；容量故障保留上一已接受帧、序号和原期限，并阻止任务完成。
已重建原生库；这只是原生帧集合的有界化，不代表Python日志、传输队列或整个进程内存有界，也不提供来源认证。
详见[第二十四批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_CAPACITY.md)。

## 2026-10-30 第二十三批：原生已接受帧元数据回传

接收状态升级为v2，回传最后实际接受命令的帧引用、采集时间和签发时间，与真实发送副本精确绑定。
生产路径拒绝缺字段或v1降级；STOP/到期/拒绝不覆盖上一授权的帧元数据，错误保持粘性且零请求不受阻。
本批有意升级原生源码及二进制；不等于图像内容校验、发布者认证或实际轮速证明，模型与安全门槛不变，正式验收不恢复。
详见[第二十三批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_FRAME.md)。

## 2026-10-30 第二十二批：接收状态与实际发送内容绑定

以实际socket发送后的独立副本核对原生状态的已知序号、速度和原绝对期限，拒绝内部自洽但与发送不符的反馈。
保留未确认发送与原生STOP保留上一授权的语义；零请求不受阻，错误反馈或旧终态不能支持任务完成。
不等于帧回传、来源认证或实际轮速证明；模型、原生协议/二进制及安全门槛不变，正式验收不恢复。
详见[第二十二批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_BINDING.md)。

## 2026-10-30 第二十一批：正常运行接收状态新鲜度门控

所有非零提交前检查原生状态发布时间与原绝对期限，拒绝陈旧、未来、到期或倒退时钟状态；失败保持粘性，零速停车不受阻。
GUI显式开始后仅允许从未授权的双端disarmed状态做一次启动观察，不用于恢复运动或续租。
真实丢反馈/旧反馈重放及正常受限样例另存；模型、租约、速度与停车门槛不变，不等于硬实时或实车保证。
详见[第二十一批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_FRESHNESS.md)。

## 2026-10-30 第二十批：通道失效后的独立停车观察

执行通道已不可恢复关闭时，先确认新鲜原生零授权及健康步进helper，再用原有限步进预算独立观察实际停车。
没有反馈或步进结果未知则拒绝继续观察；不恢复运动权限，不用独立观察覆盖原异常、清理失败或任务失败。
正常清理、模型、租约和停车门槛不变；不等于实车或硬实时制动保证。
详见[第二十批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_INDEPENDENT_STOP.md)。

## 2026-10-30 第十九批：执行客户端逐请求ACK绑定

启动握手与每条ACK增加严格字段、时刻及命令序号/速度/期限/终止语义核对；坏ACK立即进入不可恢复失败，不能被后续正常回复覆盖。
真实IPC/原生故障诊断与正常受限样例分别保存；故障后的补充物理观察不冒充生产清理成功。
不改变模型、租约、原生联锁或停车门槛，不等于回复认证、实车或硬实时保证。
详见[第十九批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_ACTUATOR_ACK.md)。

## 2026-10-30 第十八批：执行端审计链与独立终态核对

执行日志增加命令序号连续性、固定期限/目标、锁存不可恢复等检查，并与独立IPC最后确认状态核对。
真实原生日志删掉中间命令后，不再因末尾停车/退出记录尚在而误报完成；深度坏JSON也保留为审计故障。
不改变模型、执行逻辑或安全门槛，不等于日志认证、运行时篡改制动或物理安全保证。
详见[第十八批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_ACTUATOR_AUDIT.md)。

## 2026-10-30 第十七批：接收端状态一致性校验

拒绝“不可能的授权状态”、同序号期限/速度改写及锁存后序号变化；深度嵌套坏JSON也记入粘性审计故障。
坏反馈不再继续放行非零命令或支持任务完成；真实原生话题故障与正常受限样例另行验证。
不改变原模型、原生联锁、租约或停车门槛，不等于反馈身份认证或实车安全保证。
详见[第十七批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_STATE.md)。

## 2026-10-30 第十六批：原生接收命令有界解析

接收端先限制元数据数量/键值长度，再解析复制；解析移出物理/租约共享锁，避免超长输入在该锁内造成无界开销。
真实Gazebo超量字段、超长值和发布进程暂停诊断均检查停车与不可重启；正常受限L型与公共回放保持通过。
不等于传输洪泛防护或硬实时保证，原模型、0.8秒租约与停车门槛不变。
详见[第十六批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_BOUNDS.md)。

## 2026-10-30 第十五批：Gazebo接收端车轮租约联锁（P0）

新增原生接收端联锁：速度发布进程暂停/被杀死后，按原0.8秒绝对期限禁止继续驱动车轮；
畸形包和迟到新命令不能解锁，不依赖第二个零速发布者。真实物理故障诊断和正常L型样例已检查。
新增私有构建脚本，缺少/损坏联锁时拒绝运动；原模型、地图、相机和停车门槛不变。
仅限Gazebo软件保护，实车仍需驱动器/固件超时制动。见[第十五批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_RECEIVER_INTERLOCK.md)。

## 2026-10-30 第十四批：执行端日志阻塞隔离（P0补漏）

日志写入移至独立非发布进程，执行端使用有界非阻塞队列；日志阻塞/死亡不能再卡住原租约检查。
审计失败锁存零速，残缺日志不能误报任务完成。真实满管道与原生Gazebo暂停日志进程均已验证。
模型、0.8秒运动租约与停车判据不变；执行进程自身失效仍需接收端/底盘固件保护。
详见[第十四批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_WATCHDOG_AUDIT.md)。

## 2026-10-30 第十三批：Gazebo独立执行端租约（P0）

速度发布移至独立进程，按原0.8秒绝对期限失效锁存；不发续租心跳，迟到/重复命令不能恢复运动。
真实助手卡住、导航进程SIGSTOP、命令断流三类检查中，独立零速发布与外部物理停稳均已验证。
正常受限L型样例仍完成；这不是底盘硬件/硬实时保证，也不是原模型C02绕行验收。
实现、延迟、回归与剩余边界见[第十三批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_GAZEBO_WATCHDOG.md)。

## 2026-10-30 第十二批修复：Gazebo步进助手有界通信

修复助手握手/回复的阻塞读取绕过超时的问题；半行、断流、坏回复或时钟停滞后锁存失败，
不把迟到ACK当作清理步进结果；握手失败回收自有子进程。
助手已退出时的管道写入错误不会再因收尾重复flush而丢掉结果报告。
真实助手暂停故障已验证约30秒后进入异常零速链，但这**不是0.8秒独立看门狗或实车停车保障**。
范围、回归与限制见[第十二批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_GAZEBO_STEP_IPC.md)。

## 2026-10-29 第十一批修复：最终停车执行与新鲜位姿审计

最终零速提交/清理步进失败不再静默忽略；主异常与清理异常分别留存。
停车证明必须使用最后一次清理步进内新收到的两条有序位姿，旧静止数据不能冒充停车完成。
真实Gazebo不可达拒绝及声明故障已检查；不代表导航成功，也未改变模型、租约或停车速度门槛。
范围、证据与限制见[第十一批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_GAZEBO_POST_STOP.md)。

## 2026-10-29 第十批修复：Gazebo异常退出与失败帧留存

修复执行器拒绝/后端异常后控制器仍显示moving、失败帧及命令丢失的问题。
先请求零速并锁存中止，失败尝试独立保存，不改写已提交决策前缀；显示故障不再抹去已提交命令。
原17类模型C02新实跑仍未完成：停靠时得分约0.586低于原0.6门槛，保留拒绝，不改模型或租约。
范围、实跑与限制见[第十批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_GAZEBO_FAILURE_CAPTURE.md)。

## 2026-10-29 第九批修复：直走启动拒绝诊断

修复启动纹理拒绝未进入阻断项汇总的问题；三张启动图各自保留时间、像素哈希和逐格检查，
不再仅保留最后一张图的完整诊断。新增已有运行的只读公共RGB诊断入口，不重跑物理或模型。
默认6m/seed313仍因整流支持5格、低于原6格门槛而拒绝，普通直走尚未跑通。
范围、命令和证据见[第九批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STARTUP_DIAGNOSTICS.md)。

## 2026-10-29 第八批修复：异常停车链路与DDS诊断

修复超大数值绕过MotorGate失败锁存，以及无新帧时零速中止被错误归类的问题。
真实ROS2/DDS和PyBullet检查五类故障停车及一条正常路线；回放按三张指定探测帧身份核对，
挂起审计按待处理帧/旧速度租约的较早期限判定，原租约、超时与制动门槛不变。
这不是17类模型、Gazebo或实车验收；原始失败和同轨迹复评均保留。
范围、证据及限制见[第八批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STOP_CHAIN_FIXES.md)。

## 2026-10-29 第七批修复：时序事件失败隔离与完成屏障

时序会话遇到坏事件/时钟异常先锁存失败、清空确认票；终态不再解析后续坏包或读取时钟。
完成反馈先校验转移与时钟再提交，墙钟回退不能降低“完成后新帧”的采集屏障。
不改模型、规划与三帧/超时门槛；协调器失败锁存不等于真实执行器已经停车。
复现及验证边界见[第七批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_SESSION_EVENT_FIXES.md)。

## 2026-10-29 第六批修复：感知接口契约与时间戳

拒绝布尔/越界概率、畸形向量与阈值；有效标志只认可内建True，非法标志/帧龄返回UNKNOWN。
预测器拒绝非法采集/结束时间戳，不重打显式时间，不返回不一致预测包。
模型、预处理与0.6识别阈值不变；这不解决真实unknown/遮挡误识别，也不代表导航验收通过。
复现、回归及异常处理边界见[第六批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_PERCEPTION_CONTRACT_FIXES.md)。

## 2026-10-29 第五批修复：视觉路线输入与停车确认

模型/标记路线控制器增加坏包零速锁存、决策时钟单调和身份去重检查；终态不再读取或绑定后续坏包。
模型停靠证据丢失后清零确认票数，必须重新连续三帧稳定停车，再作独立节点确认。
模型、识别阈值、几何/标记解码和运动门槛不变；不代表普通直走或Gazebo C02绕行已经通过。
复现、测试及边界见[第五批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_ROUTE_SAFETY_FIXES.md)。

## 2026-10-29 第四批修复：测试范围与报告分层

新增[测试入口](/home/sunrise/toponav_project/tools/run_project_tests.py)：`--scope current`检查当前回归，
`--scope historical`检查历史报告，默认`all`仍递归检查全仓并保留历史失败。
每次生成新目录中的收集清单、源文件哈希、JUnit和分组结果；当前范围通过不代表全仓或导航验收通过。
历史范围独立导入不再依赖当前测试的收集副作用；原有同名模块避重忽略规则也会披露，不能冒充零排除的完整通过。
操作与严格退出码规则见[测试说明](/home/sunrise/toponav_project/docs/TESTING.md)，
本次验证见[第四批记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_TEST_SCOPE_FIXES.md)。

## 2026-10-29 第三批修复：直走感知数值校验

拒绝非法启动运动估计和畸形前表面距离区间，避免NaN/布尔值绕过比较检查；
非法静止证据不能完成停车确认或恢复运动。启动诊断逐帧记录失败检查及纹理支持数量。
**纹理门槛、近场覆盖守卫、模型及运动许可不放宽；普通直走仍未跑通。**
复现、测试与近场限制见[第三批修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STRAIGHT_FIXES.md)。

## 2026-10-29 第二批修复：结构与通行权限分层

物理出口不再因禁行标志而从结构签名中消失；定位匹配物理结构，
运动预测、公共规划和执行仍拒绝禁行通道。理想结构传感器同步修正，未改真实模型输出。
这修复了C02封路时的一类结构匹配错误，**不等于原模型Gazebo绕行已经跑通**。
测试、只读旧图诊断及限制见[第二批修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_STRUCTURE_FIXES.md)。

## 2026-10-29 项目检查后的第一批修复

修正ROS感知异常后的零速发布、转角/连续路线终态锁存和无效帧龄拒绝；
直走的Gazebo/浏览器入口新增独立受限探测授权，**默认仍禁止运动，确认净空不自动授权**。
运行参数见[浏览器说明](/home/sunrise/toponav_project/docs/VISUAL_SIMULATION.md)和
[Gazebo多场景说明](/home/sunrise/toponav_project/docs/GAZEBO_SCENARIOS.md)。
本次不训练模型、不降低视觉/制动门槛、不恢复历史正式验收；S2未通过、S6未完成结论不变。
完整修改范围、测试结果及剩余问题见[本次修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_FIXES.md)。

## Gazebo 交叉验证入口（2026-10-20）

**新增原17类模型闭环模式**：运行 `bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh --scenario vision`，就绪后点“开始导航”。
已在显式标定、无路标/无黄线场景跑通原模型 → 原结构定位/规划 → RGB几何控制 → 下一节点模型确认 → 终点停车。
默认路径 N01 → N02 → N03 → N04 → N06；这是受限工程联调，不是通用视觉或S2/S6验收。见[模型闭环说明](/home/sunrise/toponav_project/docs/GAZEBO_MODEL_LOOP.md)。

运行 `bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh` 打开**场景中心 + 原生 Gazebo 拓扑图**。
默认打开**六节点全流程（新增视觉标记/黄线辅助基线）**，窗口就绪后点“开始导航”，可看完整行驶、节点确认、转弯和目标停车。
支持 C02 封路绕行和 C06 不可达拒绝；原无标记六节点 `topology` 仍只预览。原 L/T 局部模式保留。
这不是原17类模型/自然无标记导航验收。操作及边界见[全流程说明](/home/sunrise/toponav_project/docs/GAZEBO_FULL_FLOW.md)。
新增T型左右转已跑通；直走/阻挡当前因地面证据不足拒绝起步，未冒充通过。
详见[多场景与拓扑图](/home/sunrise/toponav_project/docs/GAZEBO_SCENARIOS.md)。

已接入原生 Gazebo Harmonic / DART / Ogre2，受限 L 型走廊左右转与停车均实际跑通，
未确认初始净空时拒绝起步。运行 `bash tools/run_gazebo_cross_validation.sh --output reports/gazebo-manual-$(date +%Y%m%d-%H%M%S) --direction left --confirm-initial-clearance`。
默认离屏真实渲染，不弹 GUI；可导出俯视/四向相机 GIF。
需要原生 Gazebo 窗口时，在上述命令中加 `--gui`；运行后窗口会保留，关闭窗口结束记录。
这是连续 RGB 控制器的工程交叉检查，不代表完整导航、17类模型、ROS桥接或S2/S6验收。
详见[Gazebo运行说明](/home/sunrise/toponav_project/docs/GAZEBO_CROSS_VALIDATION.md)。

## 可视化模拟入口（2026-10-19）

运行 `.venv/bin/python tools/visualize_simulation.py --demo left`，在浏览器打开
[本地模拟面板](http://127.0.0.1:8765)。可查看实际轮驱轨迹、四向RGB和控制状态，
支持左右转、T路口、直走/阻挡场景，以及暂停、倍速、时间轴回放。
这是既有PyBullet运行器的只读观察面板，不改控制器或历史正式结果，不补签S2/S6验收。
详见[可视化运行说明](/home/sunrise/toponav_project/docs/VISUAL_SIMULATION.md)。

更新日期：2026-10-15（Asia/Shanghai）。项目：/home/sunrise/toponav_project。

当前主系统已统一为 v2 地图 → 结构候选定位 → 公共路径规划 → 相对动作 → 离散模拟执行。不是 ROS/真实摄像头/实车系统。

第四块已有 PyBullet 四向 RGB 与冻结 MobileNetV3-Small 图像识别基线；第五块新增独立时序会话入口，支持停车后持续观测、三帧确认、执行事件去重与有界被动恢复。旧离散入口保留作对照。接口、运行及限制见 [时序导航说明](/home/sunrise/toponav_project/docs/TEMPORAL_NAVIGATION.md)。

第六块新增 **6B-1 单走廊连续运动原型**：轮电机驱动物理车体，固定颜色/标定条件下用RGB几何纠偏与视觉停车；故障制动、部分运动反馈及单向碰撞审计可用。它与主拓扑链路独立，不等于连续拓扑导航、17类模型控制或实车能力。运行及限制见 [走廊连续运动说明](/home/sunrise/toponav_project/docs/CORRIDOR_MOTION.md)。

后续 **6B-1V 冻结边界验证**已完成：25个新条件中10例端墙停车、12例安全拒绝、3例视觉停车未通过位置审计；原控制器未改。地面涂色与宽走廊反例说明它尚不是通用直走控制器。方法、全部结果及复现见 [冻结边界验证](/home/sunrise/toponav_project/docs/CORRIDOR_VALIDATION.md)。

随后 **6B-1D 证据与反馈分层设计**完成：区分颜色边界、物理几何候选与节点到达，新增离线契约原型和44项协议测试，当时完整回归796通过。它不提取图像几何、不接轮速，设计和限制见 [证据与反馈设计](/home/sunrise/toponav_project/docs/specs/2026-10-15-corridor-evidence-design.md)。

随后 **6B-1P 只读RGB几何探针**完成：真实像素三帧追踪、单应/纯旋转竞争和分解多解诊断，新增41项测试，当时完整回归837通过。旧3486帧形成3441个窗口，全部因当前证据门槛不足保持unknown；这不是感知成功或驾驶改进。入口和限制见 [RGB探针说明](/home/sunrise/toponav_project/docs/CORRIDOR_RGB_PROBE.md)。

随后 **6B-1O 受控RGB可辨性实验**完成：冻结探针检查20条件、100图，57窗中27有诊断流候选、30未知，另1条件显式标定ID拒绝。单墙纹理平移有支持，但动态墙可与移动相机生成相同RGB，隐藏标定失配未被可靠拦截，物理几何仍未知。新增31项测试，完整回归868通过；未接轮速或修改模型，下一步为只读地面追踪与跨视图/多平面一致性原型。见 [受控实验说明](/home/sunrise/toponav_project/docs/RGB_OBSERVABILITY.md)。

随后 **6B-1G 地面追踪与共享运动解释原型**完成：逆透视/相位初始化改善追踪，结合各视图单应分解检验共享解释。冻结后新纹理地面前进6窗由旧0变为6个流候选，房间前进6窗有共享解释；重复纹理仍有3窗流候选，慢速共享支持不足。新增36项测试，完整回归904通过。物理几何仍未知，不授权运动、未改17类模型。见 [地面/跨视图原型说明](/home/sunrise/toponav_project/docs/RGB_GROUND_RIG.md)及 [最新进度增量](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_6B1G.md)。

随后 **6B-1H 歧义拒识与低视差残差佐证**完成：增加竞争patch匹配、自相似检查及跨视图拟合/留出残差验证。冻结后26个新种子条件中，重复纹理候选窗8→2（仍有残留），慢速房间获得3/6窗残差佐证；但横移/单墙正常覆盖损失明显，不能替换旧策略。新增37项测试，完整回归941通过，仍不接车辆控制。见 [只读增强说明](/home/sunrise/toponav_project/docs/RGB_MOTION_CHECKS.md)与 [最新进度增量](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_6B1H.md)。

最新 **6B-1I 原生像素外观与逆透视重复否决实验**已完成新条件与重放核验：32条件、93有效窗中，地面/房间横移各从3→5/6窗有流候选，重复纹理仍3/18窗且单墙仍0/6；慢速支持下降，跨视图支持并集12→9，**只作对照，不替换H、不接车**。新增38项测试，独立完整回归979项通过、重渲染/诊断重放一致。见 [实验说明](/home/sunrise/toponav_project/docs/RGB_RESAMPLING_CHECKS.md)与 [本轮进度](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_6B1I.md)。

最新实际进度见 [2026-10-15进度更新](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15.md)；原PROJECT_HANDOFF文档保留为实施前快照。

## 1. 快速运行

在 Ubuntu 或 SSH 远程终端执行：

```bash
cd /home/sunrise/toponav_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
export MPLBACKEND=Agg MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib

python -m simulation.topology
python -m simulation.dijkstra
python -m simulation.structural_navigation
python -m simulation.structural_navigation --disable C02
python -m simulation.structural_navigation --disable C06
python -m simulation.structural_navigation --output reports/system_unification/manual.json --plot reports/system_unification/manual.png
```

前四个导航/规划成功案例退出0；关闭C06是合法地图上的不可达，导航安全停止并退出1。非法输入退出2。完整图/场景校验命令 simulation.topology 退出0表示配置合法，task_reachable=false不代表拓扑非法。

## 2. 文件与版本

- configs/graph.json、scene.json、两份schema及graph_blocked_C02.json：当前默认v2，与configs/v2同名文件一致。
- configs/v2/examples：4/6/7节点、任意编号、无任务/几何/照片及封路样例。
- configs/invalid_cases、configs/v2/invalid_cases：故意非法的测试输入；必须与manifest指定场景配对。
- configs/legacy：保留原v1地图和错误样例。旧节点概率基线默认读取这里。
- simulation/topology.py：统一字段、方向、动作兼容及v2跨文件校验。
- simulation/dijkstra.py：v2最少跳数；v1保留按length_m加权最短路。
- simulation/path_planning.py：从可靠估计规划，按出口方向转换为下一条相对动作。
- simulation/structural_localization.py：候选集合与公开执行反馈；decide调用公共规划块。
- simulation/structural_navigation.py：当前主入口、独立结构模拟传感器和物理执行器。
- simulation/navigation.py：仅作节点概率旧基线，不作为结构识别能力证明。
- tests：旧回归测试与新增system_integration集成测试。
- reports/system_unification：本次当前验收、运行JSON、轨迹图与交付哈希。
- reports/HISTORY.md：旧验收记录的版本边界；历史结果不冒充当前验收。
- datasets、models已有正式模拟图像数据和封存结构模型；data_raw、data_processed、edge_deploy、ros2_ws/src仍未接入真实业务数据或ROS包。

## 3. 唯一的数据与方向约定

graph.task保存任务；node_type仅描述道路结构，N06为dead_end而非“视觉目标类别”。
graph.nodes[].exits显式绑定neighbor_id/channel_id/direction。方向使用地图轴：东0°、北90°、西180°、南270°。
scene.channels是独立物理通道与两端向外方向；可选scene.geometry只用于显示、标注和米制评价。
出口方向、传感器结构、候选推进及动作转换不读取显示坐标；弯曲道路按到达端向外方向+180°得到到达朝向。

统一运动命令：straight/left/right/uturn/stop。
观测方向键仍是front/left/right/back，二者不是同一种载荷。
旧front/back/u_turn仅在动作输入边界兼容；新主系统日志只输出统一运动命令。
当前任务允许原子掉头仿真，不等于真实小车具备掉头安全授权。

## 4. 路径代价与几何边界

v2路线输出cost_unit="hops"、cost、hop_count；length_m始终为null，不把一条边当成一米。
真实行驶米数由模拟器根据scene.geometry.channel_dimensions_m另行累计，存放在final_truth.distance_m。
没有通道长度时距离为null；没有节点坐标时position_m为null，仍可进行完整离散导航。
绘图必须有全部节点坐标，否则明确报输入错误，不虚构坐标。
v1显式输入保留旧length_m加权语义，便于回归和历史对照。

## 5. 任务与安全

默认起终点及朝向读取graph.task。缺失任务或初始朝向时必须显式提供，不默认朝东：

```bash
python -m simulation.structural_navigation --map configs/v2/examples/no_task_no_geometry_no_photos.graph.json --scene configs/v2/examples/no_task_no_geometry_no_photos.scene.json --start N01 --goal N06 --heading 0
```

定位歧义、结构不匹配、低置信度、执行失败或未确认均停止。发送命令不等于执行成功。
只有模拟传感器、物理执行和日志审计读取真值；定位、规划不能用真值修正估计。
上述旧入口仍遇stop即结束；新增时序入口实现有限多帧确认与被动恢复，图像入口使用实际模拟RGB推理。主拓扑链路仍未接入连续动力学，尚无全局重定位、真实摄像头或实车控制；模型遮挡拒识不足没有被多帧策略自动解决。

## 6. 测试与兼容入口

```bash
.venv/bin/python tools/run_project_tests.py --scope current
# 默认全仓检查，仍包括已知的两项历史源码快照冲突：
.venv/bin/python tools/run_project_tests.py
.venv/bin/python tools/run_project_tests.py --scope historical
python -m simulation.navigation
python -m simulation.dijkstra --map configs/legacy/graph.json
python -m simulation.structural_navigation --map configs/legacy/graph.json --scene configs/legacy/scene.json --heading 0
```

旧测试保留，fixture明确指向configs/legacy；结构版运动输出断言随统一名称更新，没有删除失败测试。
新增测试覆盖v2端到端、所有封路/起终点组合、无几何、任务缺失、布局独立、弯曲通道、真值隔离、故障安全和CLI。
直接运行`python -m pytest -q`的发现行为未修改；新入口不会给历史失败添加白名单、skip或xfail。
范围、报告解读及CI严格退出规则见[测试说明](/home/sunrise/toponav_project/docs/TESTING.md)。

## 7. 环境及历史

复用既有Python3.10、ROS2 Humble、CPU PyTorch和科学计算工具，无新增依赖。
requirements.txt、requirements-cpu.txt、requirements.lock.txt未改；当前虚拟机约4GB内存，不是Jetson/TensorRT端侧环境。
本次前版本完整备份在backups/system-unification-20261014-0d07b054。
旧总说明和被替代说明另存docs/history/2026-10-14-before-unification；旧验收报告保留原文和原始哈希。
当前接口见docs/NAVIGATION.md、docs/PATH_PLANNING.md、docs/DIJKSTRA.md和docs/topology-v2-usage.md。

## 第六块长期goal（执行中，非最终交付）

已按用户授权建立长期goal，依次完成视觉证据、直走、转角、T路口、多节点、安全验收和ROS2联调，全部通过后才最终交付。目前S1开发中，米制平面仍有可辨性/覆盖问题，未接控制，不能将新研究模块视为已完成第六块。顺序与退出门槛见 [执行计划](/home/sunrise/toponav_project/docs/plans/2026-10-15-block6-goal.md)，状态见 [进行中记录](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_BLOCK6_GOAL.md)。上述历史验收结果保持。

### 长期goal最新增量：S1受限条件通过，S2进行中

第六块尚未完成。S1新条件验收：正常运动50/50、双侧几何29/30、近前面9/10；1068回归通过，旧模型/正式图/源码/报告保真。隐藏标定失配并未被普遍识别，输出仍无运动/节点授权。下一阶段为独立直走闭环和自由空间/障碍制动；S3—S7未开始。详见`/home/sunrise/toponav_project/reports/block6-goal-20261015-3tth93x5/S1_ACCEPTANCE.md`。goal保持active，不是最终交付。

### 2026-10-16：单转角控制（受限仿真）

已加入真实轮电机/接触物理下的RGB反馈进弯、左右90°转向、出弯对准和停车；复用现有蓝灰地面/灰墙边界方法，不是识别网络升级或任意材质/实车保证。直接扩展已有源文件，原直走控制器/策略保留，未重新加入暂缓的重复纹理检查。

冻结后20条件：8/8正常完整转角、12/12故障/拒绝条件通过，0碰撞；1181全回归通过。首个失败候选完整保留。S2和整个第六块仍未验收完成。运行命令、范围和证据见[单转角验收与运行说明](/home/sunrise/toponav_project/reports/single-corner-20261016-v2/ACCEPTANCE.md)。


### 2026-10-16：T形路口控制（S4受限仿真已通过）

新增支路进入T形路口的RGB接近、双分支连续观察、按指定左右选向、阻挡停车等待/重新确认、转向与出路口确认。直接扩展原源文件，未更换识别网络或恢复重复纹理检查。冻结后26条件：正常8/8、临时阻挡解除后完成2/2、另一分支受阻仍按计划完成2/2、14/14安全条件，全程0接触；1228回归通过。S1与S3历史冻结保留，S3默认20序列3894决策回放完全一致。

仅蓝灰地面/灰墙、静态平地与声明标定的T形支路进入场景，无直行出口；不是任意材质/路口/实车认证或拓扑节点到达。S2未正式验收，S5—S7未开始，第六块未完成。下一步为多节点连续导航。证据与命令见[T形路口验收与运行说明](/home/sunrise/toponav_project/reports/t-junction-20261016/ACCEPTANCE.md)。

### 2026-10-16：多节点连续导航（S5受限仿真已通过）

已把RGB反馈转弯/路口接上原v2公共规划器，在**同一实际轮驱世界**连续完成多节点任务。停车后以公开带节点/朝向标签的自然参考照片和当前四向结构，连续3帧独立确认节点；转弯出口本身不算到达。反馈绑定/幂等/超时，部分执行失败不假装回到原节点。物理位姿/速度/接触仅事后审计，未更换或训练17类网络。

116项首次生成前冻结后，正常六条新布局/起终点路线6/6、重复反馈整路2/2、安全条件16/16，全部0接触；1280完整回归通过。只覆盖已声明标定/静态平地/蓝灰地面与中性墙/正交支干进入L/T/死端的有参考地图路线，不是任意材质、无参考重定位或实车保证。S2仍未验收，下一步S6安全边界统一验收，之后S7 ROS2联调；整个第六块未完成。

[验收与证据](/home/sunrise/toponav_project/reports/multi-node-20261016/ACCEPTANCE.md) · [独立复跑入口](/home/sunrise/toponav_project/reports/multi-node-20261016/RUNNING.md) · [最新进度](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_BLOCK6_GOAL.md)。旧入口/历史报告/冻结快照继续保留。

### 2026-10-18：S6安全工程开发完成，正式验收未完成

已统一直走/L转角/T路口/路线的相机故障注入、异常零速日志、终止后好帧不重启检查、反馈严格类型校验、公共RGB精确重放与120Hz物理制动/接触独立审计。直接扩展既有文件，没有更换识别网络、新增依赖或恢复重复纹理启动检查。

最终版本17组真实轮驱开发独立检查无错误、全部0接触，正常L/T/两边路线3/3完成；4560次任务决策回放一致、51张终止好帧保持零速/结果不变。完整回归1342通过；旧S3/S4/S5回放及原成果保真通过。115项实现/协议/测试等已冻结，但**正式67组新条件尚未生成/评估，不能称S6整体验收通过**。S2与重复纹理一般可辨性继续延期；相机silence仅为同步调度缺输入测试，不是硬件看门狗保证。S7未开始，第六块未完成。

[S6开发报告](/home/sunrise/toponav_project/reports/control-safety-20261018/DEVELOPMENT.md) · [运行说明](/home/sunrise/toponav_project/reports/control-safety-20261018/RUNNING.md) · [最新进度](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_BLOCK6_GOAL.md)。

### 2026-10-18最新：S7 ROS2受限仿真联调已通过

按用户后续明确授权，新增[toponav_bridge ROS包](/home/sunrise/toponav_project/ros2_ws/src/toponav_bridge)，真实`ros2 launch`启动相机/唯一轮控world、公共规划与RGB连续决策navigation、段反馈feedback三个进程，经Fast DDS完成多节点物理导航。不是mock/只有Topic消息，也不是更换识别模型；旧原核心与S6原115绑定均保持。

137项首次新图生成前冻结后，8组新条件：两套新尺寸/含镜像布局的两边完整路线 **2/2**，消息断流/导航进程SIGSTOP/反馈断流/命令重放/错误版本/未来时间戳 **6/6**，全部0接触。3189公共事件/决策、3179接收RGB/4反馈精确重放、24张实际终止好帧不重启；最长制动.175仿真秒/1.2262cm。导航被暂停时，轮控独立.8墙钟秒租约仍触发零速。冻结前后完整回归 **1411通过**，依赖/模型/2040图/历史成果保真。

仅为已声明静态/标定/灰墙蓝灰地面/正交参考地图的锁步加速仿真。**仿真秒不是实车墙钟停车承诺，world/OS/电机进程挂死及硬件安全不在保证内。S2仍延期，S6正式67条件/重复纹理边界仍未验收，整个第六块未完成。** 早期构建/接口/环境失败与原始日志保留，未从正式部分输出调参。

[S7验收报告](/home/sunrise/toponav_project/reports/ros2-integration-20261018/ACCEPTANCE.md) · [构建、启动、参数、Topic、时钟及看门狗](/home/sunrise/toponav_project/reports/ros2-integration-20261018/RUNNING.md) · [最新阶段状态](/home/sunrise/toponav_project/docs/PROJECT_PROGRESS_2026-10-15_BLOCK6_GOAL.md)。

## 2026-10-18：S2恢复开发，本轮受限增强已验证，整体仍未验收

直接修改已有直走控制/轮驱运行器，没有生产包装层、模型训练或恢复重复纹理启动guard。新增运动中“新ID/时间但RGB像素完全不变”累计.30仿真秒后零速锁存；基本居中/对齐且前墙当前有效时限速接近；终止后坏包也不改写结果。当前源码先绑定后串行5组seed313开发：居中/+.16m偏移/-8°偏航正常3/3，起步/行驶冻结2/2，全部0接触、独立检查错误0；193决策公共精确重放、15终止好帧逐图/时间/锁存核对，最长完整制动0.166667s/0.012307m。1434全回归通过、18既有警告。

色带候选出现竖直阻挡误判，已从生产源代码完整撤回；初版减速造成偏移功能失败，旧13组及全部失败保留，加入对齐条件后另做当前源码5组，不将两候选结果混算。**涂色通行、一般重复纹理启动安全和S2正式20组仍未解决/验收；S6正式67组仍未运行，整个第六块未完成。** S1原84绑定、L/T/路线源码片段、旧冻结快照/报告/模型/2040图/依赖保持；S2现已获授权改变共享源码，旧S6/S7当前源码SHA不再适用，新正式批次需新冻结。

[本轮开发报告](/home/sunrise/toponav_project/reports/s2-improvement-20261018/DEVELOPMENT.md)。

## 最新任务进度：2026-10-18（按用户要求停止）

**当前已停止第六块任务，不再自动开发、重跑或继续验收；整个第六块尚未完成。** 本节为最新状态，前文“执行中”“正式条件尚未生成”等内容保留为历史记录，不代表当前状态。后台验收流水线及正在运行的 S6 评估进程已终止，停止后核对无残留任务进程；没有将任务标记为完成。

### S1—S7 当前状态

| 阶段 | 已实现内容 | 当前验收状态 |
| --- | --- | --- |
| S1 视觉与运动证据 | 已声明相机标定/静态平地条件下的 RGB 相对运动、双侧与前方几何 | 受限工作域验收通过；隐藏标定失配等限制仍保留 |
| S2 直走闭环 | RGB 纠偏、限速接近、视觉停车、运动中完全冻结图像检测及终止锁存 | **本次正式 20 组已完成，但验收未通过** |
| S3 单转角 | RGB 闭环进入、左右 90° 转向、退出与停车 | 受限仿真验收通过 |
| S4 T 形路口 | 支路进入、左右选向、阻挡等待及出路口确认 | 受限仿真验收通过；不包括直行出口 |
| S5 多节点导航 | 公共拓扑规划、连续轮驱执行、参考 RGB 三帧节点确认及反馈去重/超时 | 受限仿真验收通过；段退出不等于节点到达 |
| S6 连续控制安全 | 四类动作的时序/相机故障、异常零速、物理审计与公共回放框架 | **本次正式 67 组已全部生成；整批独立评估中途按用户要求停止，未形成最终验收结论** |
| S7 ROS2 联调 | 三进程真实 ROS2/DDS、唯一轮控拥有者、消息故障与独立轮控租约 | 受限 ROS2/PyBullet 仿真验收通过；不是实车或硬实时认证 |

### 本次实际完成的正式验收工作

本次仅冻结和验证当前源码，**没有修改生产代码、恢复已撤回的重复纹理启动检查、训练识别网络或更换依赖**。新候选共绑定 **205 项**源码/测试/配置/协议等；使用新的首次生成凭据，旧冻结、失败记录和报告保留。

- **冻结前完整回归：1434 项通过，18 条既有警告。** 这是本次运行前的检查；停止时，运行后的完整回归与最终保真步骤尚未执行，不能把前测写成后测。
- **当前源码历史公共输入精确回放通过：** S3 为 20 组/3894 次决策，S4 为 26 组/5689 次决策，S5 为 24 组/16625 次决策，S7 为 8 组/3189 次事件与决策（3179 张接收 RGB）。这是历史行为兼容性复验，不是新增场景验收。
- **S2：20/20 正式条件和 20/20 原固定颜色控制器对照均已运行完毕。** 所有对照首帧 RGB 一致；后续闭环轨迹不同，不宣称持续图像相同。全部 1534 次 S2 公共输入决策及最终结果精确回放一致。
- **S6：67/67 正式条件已串行运行完毕。** 原始图像、决策、反馈及物理轨迹已保存；随后整批评估被停止，**尚无 `s6-assessment.json`，不能宣布 S6 验收通过或已完成最终独立审计**。

### S2 正式验收未通过的具体原因

1. **正常任务仅完成 5/10，低于至少 8/10 的门槛。** 成功：`center_a`、正负偏航、材质、光照。未完成：宽走廊、正负横偏、组合偏差、地面涂色。两个纯横偏案例均未完成，也不满足偏移纠正家族门槛；安全停车不能算正常任务成功。
2. **重复纹理条件仍不满足“禁止启动”。** 实际发出 12 次运动命令、移动约 **0.183m** 后才终止停车；最终没有碰撞不能抵销这一失败，也不能宣称重复纹理安全问题已解决。
3. S2 主验收记录显示 20 组均最终停稳且无墙/障碍接触，但**整阶段仍为 `passed=false`**。补充独立物理审计复用了 S6 更严格的逐次停车口径，也输出 `passed=false`：存在制动尚未完全停稳即被后续命令恢复的记录。S2 原协议允许符合条件的新鲜有效证据中断停车请求，两种口径的最终逐项核对尚未完成；原始结果保留，不将补充审计包装为全部通过。

### 停止位置、证据与剩余事项

停止位置是 **S6 整批独立评估过程中**；之后的运行后完整回归、最终保真及总交付审核未执行。当前没有后台实验继续运行。若将来用户明确要求恢复，首先处理 S2 功能覆盖不足与重复纹理启动缺口，再按原协议完成新的独立候选验收、S6 最终评估和总回归；不能调整门槛、覆盖旧结果或将本次条件再次当成未见新条件。

- [本次验收协议](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/PROTOCOL.md)
- [S2 正式未通过结果](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/s2-assessment.json) · [公共回放及补充物理审计](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/s2-independent-audit.json)
- [S6 完整生成凭据](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/s6/phase-complete.json) · [流水线实际日志](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/pipeline.log)
- [用户停止与进程终止记录](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/USER_STOPPED.json) · [新候选冻结清单](/home/sunrise/toponav_project/reports/block6-acceptance-20261018/freeze.json)

已有成果仍限定为声明工作域内的经典 RGB 几何/连续控制与仿真联调，不等于改进冻结的 17 类识别网络，不等于任意道路泛化或实车安全保证。

## 第七块：实验与成果（P/N实验与7E阶段交付完成，连续/ROS未执行）

已整理 7A—7E 五个小任务，逐项规定目标、交付文件与验收指标：现状与实验方案、测试集与运行工具、感知跨场景实验、导航对照与连续边界实验、统计分析与成果交付。此处的第七块不是第六块中的 S7 ROS2 联调。

详见[第七块任务拆分与交付指标](/home/sunrise/toponav_project/docs/plans/2026-10-18-block7-experiments.md)。

### 7A：现状复核与实验方案已完成（2026-10-18）

已交付范围/能力证据说明、49 项带 SHA256 的成果清单、实验协议及 methods/metrics/budget 三份配置。区分冻结 17 类感知、无参考照片的离散拓扑导航、有参考照片的连续/ROS2 三条轨道；明确四方法对照与两组单机制消融。7A 时点生产配置不允许单帧确认且无关闭恢复开关；7B 已在独立实验适配层实现并验证四方法，不修改生产默认行为。

55 项方案/证据静态检查通过，2040 张正式图哈希与 2289 项原文件保真通过；7A 阶段没有运行模型推理、导航或 ROS，没有修改生产代码/依赖，也没有运行新的完整回归。首批样本预算为感知验证/测试共 2520 包、四方法离散任务共 4752 条；7B 阶段只导出静态定义，7C验收时感知验证/最终测试已完成，导航与连续轨尚未执行；当前离散导航亦完成，连续轨仍需明确恢复授权。**当前7A—7C已完成，7D原子离散N及7E现有P/N阶段交付完成；连续/ROS仍待授权未执行，整项7D/第七块未完成，第六块仍停止，S2未通过、S6未完成的状态不变。**

[7A 交付与检查结果](/home/sunrise/toponav_project/reports/block7/7a-20261018/ACCEPTANCE.md) · [范围与现状](/home/sunrise/toponav_project/reports/block7/7a-20261018/SCOPE.md) · [实验协议](/home/sunrise/toponav_project/reports/block7/7a-20261018/PROTOCOL.md)。

### 7B：测试集与可复现运行工具已完成（2026-10-18）

已交付18个新布局族（验证6、最终测试12，覆盖3档节点规模）的静态地图/场景、任务、因素与种子；计划4752条导航运行、2520包感知样本，**不是正式实验结果**。在声明的历史来源范围内，精确图同构筛查的历史/跨 split 重叠均为0；25份静态文件重新生成后字节一致。

已实现四方法实验适配、串行记账、超时回收、公共重放、独立真值审计和最终测试冻结门槛。开发干跑仅使用历史已暴露布局的真实RGB：正常四方法4/4到达；首帧丢失时2例恢复到达、2例按设计拒绝；70个公共事件精确重放，M1的31个事件与原生产核心等价。另记录21包感知开发样本，不据此宣称泛化改善。

36项资格检查通过；完整回归由1434项增至1522项，新增88项，均通过。2277项原文件（含2040张正式图）未变，无生产代码/模型/依赖变更。候选冻结仅绑定P/N，不授权连续轨、不启动正式验证/最终测试。首次冷加载导致的开发超时及中间记录均保留，没有放宽门槛。

[7B 验收与限制](/home/sunrise/toponav_project/reports/block7/7b-20261018/ACCEPTANCE.md) · [运行与复现说明](/home/sunrise/toponav_project/reports/block7/7b-20261018/RUNNING.md) · [交付哈希清单](/home/sunrise/toponav_project/reports/block7/7b-20261018/manifest.json)。7B验收时下一项为7C；当前7C已完成，见下节。

### 7C：感知跨场景实验已完成（2026-10-18）

在原冻结17类模型、预处理和0.6阈值下完成验证504包、最终测试2016包，全部记账，无重训/调参。最终测试12个布局族：已知结构准确率99.35%（1526/1536），布局宏macro-F1为94.54%；unknown错误接受率**25.83%（124/480）**，其中预声明遮挡阶段**93.75%（90/96）**，不能称为可靠拒识或安全泛化通过。覆盖率81.60%，已接受准确率池化92.46%；逐族宏平均、95%成簇CI和全部分母分别披露。

96包缺视图由输入守卫拒绝，不冒充网络分类正确；另1包UNIX帧龄与monotonic计时不一致，原守卫拒绝和证据保留，不重标或重跑。36项验收检查通过，14份分析文件独立复算字节一致；运行后完整回归1537项通过（新增报告测试15项），2813项原文件及2040张正式图保真。已保留首次P最终测试生成凭据，条件此后已暴露，不得调参后再称未见。

[7C 验收与结果](/home/sunrise/toponav_project/reports/block7/7c-20261018/ACCEPTANCE.md) · [详细指标与限制](/home/sunrise/toponav_project/reports/block7/7c-20261018/perception/RESULTS.md) · [失败分析](/home/sunrise/toponav_project/reports/block7/7c-20261018/perception/FAILURES.md) · [只读复算](/home/sunrise/toponav_project/reports/block7/7c-20261018/RUNNING.md)。**7C验收时7D—7E未启动；当前7D原子导航进度见下节。连续控制、ROS和第六块仍未恢复。**

### 7D：导航对照与消融——原子离散N已完成，连续边界未完成（2026-10-19）

验证1584/1584、正式测试3168/3168，共4752次全审计；正式测试成功1818、拒绝900、超时450，运行错误/未运行0。本批误到达0、错位姿发令0，非连续安全保证。正常任务M1=180/180、M0=172/180，预声明主差值**+4.444个百分点（95%布局族配对CI[1.667,7.222]）**；M1与M2同为180/180，不能声称三帧确认在正常成功率上有独立额外收益。所有因素/消融与成本详见阶段报告，不只展示有利结果。

用户暂停/显式续跑、单例无RGB模型加载期中断的预备进程重启及报告编排补充完整保留，不称新独立测试；冻结候选/模型/阈值/任务/预算不变。原7D续跑后完整回归1583项通过；7E又从原始图像/审计/事件全量只读复算，最终离散复核见[7D_N_REVIEW](/home/sunrise/toponav_project/reports/block7/7e-20261019/7D_N_REVIEW.md)。原STATE保留自动运行结束时“待复核”历史，不覆盖旧报告。

连续24/ROS8仅有计划预算，未精确逐例冻结、未授权未执行；不产生物理碰撞/制动曲线，不补签S2/S6，**整项7D仍未完成**。[连续资格及缺口](/home/sunrise/toponav_project/reports/block7/7d-20261018/continuous/ELIGIBILITY.md)。

### 7E：现有P/N阶段统计与成果交付（2026-10-19）

交付统一报告、1088项带分母/池化值/逐族宏平均/CI的汇总、9792项逐族指标、2641项诊断目录（含2个未执行范围条目）、成本与恢复删失表、6张图和只读复现命令。原始P/N分析21份产物与新复算逐字节一致，独立统计230项交叉核对一致，最终所有表/JSON/图/生成报告在新目录再次字节复现。新增28项资格测试、最终完整后回归1611项及46项交付检查通过，129801项旧文件（含2040张正式图）保真。

[成果总入口](/home/sunrise/toponav_project/docs/BLOCK7_EXPERIMENTS.md) · [阶段报告](/home/sunrise/toponav_project/reports/block7/7e-20261019/EXPERIMENT_REPORT.md) · [验收与限制](/home/sunrise/toponav_project/reports/block7/7e-20261019/ACCEPTANCE.md) · [只读复现](/home/sunrise/toponav_project/reports/block7/7e-20261019/REPRODUCE.md) · [交付哈希清单](/home/sunrise/toponav_project/reports/block7/7e-20261019/DELIVERY_MANIFEST.json)。

**本次仅完成现有P/N证据的7E阶段交付，未删除连续/ROS轨道；第六块、整项7D和整个第七块仍未完成。** 不启动新实验、不训练或调参，不将本结果包装为任意道路/实车/硬实时或可靠拒识能力通过。

### 2026-11-03：直行参考搜索计算去重

同次81候选复用搜索准备，保持完整匹配、输出和安全门槛；18组离线输入完整输出一致，已暴露RGB参考选点中位耗时减少约7%–11%。扩大回归1013通过、33项缺历史文件失败（修改前实现亦同）；标准完整current收集也被缺失历史工件阻断，不能称全量通过。不是整段导航或实车验收，S2未通过/S6未完成。

[实现与边界](/home/sunrise/toponav_project/docs/STRAIGHT_REFERENCE_PREPARATION.md) · [测量与验证记录](/home/sunrise/toponav_project/reports/straight-reference-preparation-20261103/REPORT.md)。

### 2026-11-03 后续：持续证据/恢复组合验证，未取得收益

新增显式 `gradient_history` 开发实验，把既有训练轨迹历史选择接到梯度版本；默认不变。三组普通条件两版均0/3完成，组合版行进距离全部降低，因此未推广。10次开发运行/374条决策及30张终态帧回放一致、物理审计无错误；这不是普通直行验收。扩大回归1038通过、33项原缺失历史文件失败，完整current仍被缺失工件阻断。完整调用剖析仍明显超0.1s，主要工作在描述子匹配及配对处理。

[实现与限制](/home/sunrise/toponav_project/docs/STRAIGHT_GRADIENT_HISTORY.md) · [失败、比较和完整剖析](/home/sunrise/toponav_project/reports/straight-evidence-recovery-20261103/REPORT.md)。

### 2026-11-03 后续：成对描述子总体精确去重

仅在画布/角色/模板/端点及有效支持完全一致时复用搜索总体和归约，不合并 H/I 接受决定。三条相同完整已暴露序列 AB/BA 测量的控制器总耗时中位数减少 7.01% / 5.24% / 4.16%，1950 条计时/预热输出一致；局部退化样本仍保留。581 条旧决策及 39 张终态帧兼容回放、旧物理轨迹复算无错误，未新增物理验收。最终定向 252 通过，扩大 1065 通过、33 项原缺历史报告失败；完整 current 仍阻断。单调用仍明显超 0.1s，未改善直行完成验收，默认权限不变，S2 未通过/S6 未完成。

[实现与边界](/home/sunrise/toponav_project/docs/STRAIGHT_PAIRED_POPULATIONS.md) · [完整测量及失败记录](/home/sunrise/toponav_project/reports/straight-paired-populations-20261103/REPORT.md)。

### 2026-11-04：补丁/NCC 复用候选拒绝，默认已恢复

尝试轨迹内精确复用原始补丁与 NCC。首批工具漏映射显式过滤器，错误基线数据已标记无效并保留；仅修工具、原候选不变后完整比较：中心/默认中位改善4.19%/4.50%，正横偏退化4.88%，因此不接入。H/I/配对三份既有源码恢复原字节，上一轮总体去重保留；只留下未被默认入口导入的实验类。有效计时/预热1950条输出一致；额外回放仅六例207决策/18终态帧完成，其他未计完成。最终定向346通过，扩大1159通过、33项原缺历史报告失败，完整current仍被阻断。没有新物理验收或安全策略变化，10Hz未达到、S2未通过/S6未完成。

[实验、工具缺陷、失败与恢复记录](/home/sunrise/toponav_project/reports/straight-paired-patches-20261103/REPORT.md) · [独立实验机制](/home/sunrise/toponav_project/docs/STRAIGHT_PAIRED_PATCHES.md)。
