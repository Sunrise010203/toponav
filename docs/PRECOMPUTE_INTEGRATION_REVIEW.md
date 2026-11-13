# 2M：Gazebo计算前 pending 与原预算贯穿——接入评审稿

**状态：仅设计评审，未实现、未批准生产接入。**机器清单在 `/home/sunrise/toponav_project/docs/PRECOMPUTE_INTEGRATION_REVIEW.json`。14项变更要求、12项转移审查与25类未来测试不是新运行结果。本轮工具只查结构/声明覆盖与固定源码锚点，不证明设计完整、可调度、可信或安全。

## 1. 不再重做影子模型：当前事实与目标分开

- 2B已有假定曝光下界的owner/token影子模型；它不是可信来源实现。2K已有纯合成工作窗口；2L已运行原LeaseGuard及独立ReceiverLease头文件零命令测试。本轮只读这些封存证据，不重跑成新实验。
- 当前Gazebo主tick：next_frame→辅助/主PNG→controller.update→本地arm/accept→command。已有旧hold仍有效，但首次计算前无该帧的下游pending。
- ROS World.frame已有capture后、PNG/DDS前pending；不把Gazebo缺口泛化到ROS，不改ROS默认MotorGate接口或阈值。
- 2L原样短expires不按接收时刻续租；**issued/expires联合重签仍可能被原两端接受**。seq0注册前无pending，声明review或停止的owner不能自行执行poll。

目标是未来增加额外**可用后工作预算**，不是放宽原保护或补造曝光证据：

```
D_work = ORIGINAL_complete_frame_available + 0.8
D_effective_before_accept = min(D_work_pending, old_D_hold_if_any)
issued = trusted_owner_issue_time
expires = min(ORIGINAL_D_work, issued + 0.8)
```

所有入口先检查旧hold与pending，now>=任一期限锁存；及时新帧消费后可替换旧hold到该帧剩余D，不永久裁剪到第一条hold。旧0.8s/审计ACK0.4s、原模拟年龄检查、限速/制动/恢复、权限完全保留。

## 2. 来源/所有权：必须先承认缺什么

|数据/动作|未来唯一写入者/检查者|现状|
|---|---|---|
|原完整帧可用时刻、同主机钟/boot及帧代次|受信采集边界，紧邻完整帧可用；不是PNG/hash/接收/计算结束|没有独立受信提供者；当前next_frame在导航IO中|
|原D、完整必需视图配置、登记册|与计算端隔离的owner，登记后不可改写|只有2B影子假设，无此生产边界|
|像素绑定|owner持有稳定原像素/视图身份，reserve后hash，seal一次|当前buffer/拼图不构成不可替换认证|
|结果|计算端仅提交结果及绑定引用，不能设置原点/issued/expires|生产调用方目前形成issued/expires|
|pending/消费状态|Python执行端、native接收端各自查登记预算并本地一次消费|只有原短expires检查，没有可信预算登记|

原点定义为**该配置完整可用帧的边界**，不能每次重读旧buffer或迟到注册再取now。当前CameraBuffer.latest在返回前已有拼图/等待开销，D_work不能宣称覆盖这部分或真实采集/上游排队；若未来要更早起点，必须另冻结定义和观测，不混用2K口径。

即使本地dataclass不可变、UUID唯一、SHA256一致、会话相同或字段由“owner”命名，也不构成来源认证/权限隔离/跨运行防重放。必须先证明谁能写采集记录、谁能发reserve/seal、计算端无法伪造/替换；这些机制尚缺，**不能靠当前协议多塞几个字段宣布解决**。

## 3. 最小闭环不是只移动一行arm

仅把本地arm移到controller前，仍与导航同进程停顿；MotorGate.accept原逻辑还会按accept时刻起hold。仅缩短expires又挡不住联合重签。必须作为未来完整变更集合评审：

1. 完整帧可用时建立最小attempt与原点记录，保留固定视图集合/代次。当前PNG失败前pending诊断可能尚未建立，不能把失败伪装进成功prefix。
2. **先reserve两端pending，再做像素hash/复制/PNG/计算。**否则把昂贵hash移到arm前，又留下新缺口。reserved状态只保护时间和帧身份，绝不接受command。
3. 两端分别确认已安装同一原D/身份/代次。确认须在旧hold与原D之前到达；预算从原availability算，不从ACK算。注册前停顿仍可能未安装pending：首次无旧hold时只有原disarmed零，不能声称按D锁存；恢复注册必须携原D，过期拒绝，未注册command禁止。
4. owner在原预算内计算配置全部视图绑定，seal一次；seal只能补充完整像素摘要，不可改frame/配置/原点/D。两端确认sealed后才能PNG/计算。引用/像素稳定性、复制费用和owner隔离仍是前置设计工作，不以Python只读flag冒充不可替换。
5. 结果返回先poll原两期限，再查完整binding/登记代次/未消费状态；只有owner签发剩余预算。后端查自己已安装的原记录，不以计算端自报“原D”当真实性证明，拒绝联合重签。
6. 每端本地一次消费pending并替换hold；不承诺分布式原子提交。任意一端结果未知/ACK迟到，保留未确认提交并进入既有失败/零路径，不重发旧帧、不重签、不退回旧能力。

安装/封存确认不是现有audit-writer的flush ACK，原0.4s审计限制不挪作新的握手超时。未来barrier只能在min(原pending,旧hold)剩余预算内等待，现有IPC限制也不放宽；没有新宽限或续租心跳。任何日志同步写、锁/OS卡顿都不获得实时保证。

## 4. 具体变更面与必须联动的约束（均未落地）

|位置|未来最小改动/风险|
|---|---|
|GazeboIO.next_frame及主tick|原点记录在完整帧可用边界；reserve/ACK/hash/seal在PNG与update前，撤销后计算arm，失败attempt提前；不改变真实controller输出|
|MotorGate/LeaseGuard|显式绝对pending API或独立版本结构；保持原ROS默认行为。pending最多1个，seq0也到期；accept不重置D，先poll旧hold|
|ReceiverLease|增加reserved/sealed/consumed状态且seq0检查pending；已消费/到期sticky不解锁；STOP不依赖新帧|
|receiver_envelope / Python严格keys|现v1恰8字段，直接加字段会被拒绝。新语义须显式版本/消息种类/有界解析，双方能力不匹配fail closed，不静默回退|
|native状态及发布去重|现v2回显accepted而非pending。未来需要pending代次/阶段/原D/消费回显；现sim/seq/reason去重键须包含pending变化，否则不能据旧状态认定安装|
|ActuatorClient/ReceiverEvidence|保留AFTER send账本；sent≠accepted，pending安装≠seal≠command消费≠物理执行。未知ACK不清除原失败，旧字段白名单须显式联动升级|
|构建/回归|将来改native源必须重新匹配构建摘要/ABI及协议回归；此轮不运行构建脚本或native构造|

保留原Python帧16384/native4096和既有审计/状态/候选/提交上限；未来pending计入有界容量，不提额、不淘汰旧证据、STOP不占新帧。并发覆盖、回调顺序、锁顺序和解析/输出阻塞须独立测试；此稿没有证明这些上界。

## 5. 首帧、后续帧、终止和失败路径

|路径|要求|
|---|---|
|首次帧|两端reserved/ACK/sealed后才准许未来结果消费；注册前没有pending的事实保留，不能宣称此稿已处理owner SIGSTOP|
|后续帧|保留旧hold，先poll；旧hold先到则锁存；及时新帧只替换到自己的固定剩余预算|
|normal observing零|与普通结果同样一次帧预算；不是heartbeat、取消pending或重臂|
|abort/visual_stop/STOP|无需新帧、seal、曝光、容量或ACK才发零；不增加新authority、不重写最后accepted元数据，STOP非任务完成证明|
|PNG/hash/update异常|先请求零再取消/写诊断；保留failed attempt与原异常，不能重算/重签或改写成功prefix|
|KeyboardInterrupt/tick耗尽|统一先零再cancel；当前_failure_stop已有此顺序，但KeyboardInterrupt/耗尽路径不同，未来必须覆盖，不声称已修|
|发送后未知ACK|保留未确认发送，不把后来匹配状态回填为先前健康；通道失败后不修复/重臂；沿既有独立零观察而非物理停车宣称|
|cleanup/重启|不改变unbounded join-before-detach，不用新session自动恢复运动权限；停止的owner/native/OS无法poll，仍需独立底盘/固件及制动反馈|

## 6. 离线测试计划、边界和推进门槛

JSON中25类场景全部为`planned_not_executed`，记录复用2B/2K/2L/源码事实的范围；既有反例并非本轮新实验。未来先实现**不接生产、无IO的版本化协议草案及畸形包拒绝测试**，再另行评审任何纯guard变更/零速实链；无授权不得跳到原生/设备。

本轮审查工具的24种变异拒绝只验证清单防漏/不提升权限，不是25类运行时场景已经通过。源码锚点只证固定字节和词法位置，不是完整控制流、C++语义、并发调度或安全证明。

六项前置缺失：受信原点、下游pending、认证绑定、版本化wire/status、运行时poll活性、真实曝光提供者。仍未知真实曝光、D_available不等于D_exposure；不重复实现2B影子owner/token。

项目仍未完成：0.1s（旧60/144超预算）、旧1.068s晚ACK根因/publisher SIGSTOP flush_ack_timeout、物理停车、S2未过/S6未完、四个暴露条件0/4、868扫掠格0支持/9远点非区域覆盖/0.25m窄障碍。原0.8s/0.4s和普通直走权限不得因此放宽。
