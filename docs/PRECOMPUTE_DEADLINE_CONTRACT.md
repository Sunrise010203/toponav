# 2K：计算前期限缺口的离线复现与候选契约

**未修复生产门控、未启动原生仿真或设备，不是普通直走/物理停车验收。**
本轮不是首次发现缺口，也不重新声称实现2B已有的DeadlineProposal。
新工具只用纯数据整数纳秒比较三种时间窗口，并独立冻结/只读检查现有源码的词法顺序。

## 核对现有代码（不导入、执行被审计模块）

`tools/precompute_deadline_cases.py`只读取显式冻结的六个固定源码文件，必须提供完整pins及pin文件摘要。
源码采用现有bounded regular-only / NOFOLLOW双读与末尾重读，要求静止文件，不是原子/敌对文件系统快照。

- Gazebo run主tick：next_frame返回 → PNG保存 → controller.update → local gate.arm → gate.accept → io.command。
  **第一次新帧的计算/PNG阶段尚无该帧pending**。旧已接受命令的hold仍可能在独立执行/接收端到期，不能说已有保护完全消失。
- ROS World.frame：capture返回 → gate.arm（非terminal分支）→ PNG保存 → DDS发布。Navigation.drain含controller.update，是不同角色/方法。
  这是已有的pending位置，不应改写成“ROS也在导航计算后才arm”。本轮只读取源码，不新观测ROS进程或调度。
- MotorGate.arm先poll旧hold；poll检查pending与hold；及时accept后hold=wall+lease_s。
- 原LEASE_S=.8、AUDIT_TIMEOUT_S=.4、ROS默认.8不改。C++ receiver_lease.hh本轮只固定字节，不编译、不声称新验证原生行为。
- 旧2B shadow模型仍要求假定的bounded_exposure_mapping并以最早曝光下界+.8为锚；它没有真实认证提供者。

AST检查限定到命名方法/主tick并要求锚点唯一，附文件摘要及一基行号；只证明特定源码字节中的词法关系，
不证明所有分支控制流、异常路径、OS调度、线程独立性、实际执行时延或电机停车。

## 新增纯数据算术模型

`tools/precompute_deadline_contract.py::compare_schedule`无IO、当前时钟、sleep、线程、回调、nonce、owner、permit或运行时依赖。
输入仅合成同一时钟域`synthetic_monotonic_ns`、capture开始、完整帧可用、序列化、计算开始/返回、检查时刻、旧hold及完整输入/结果身份。
模拟戳只用于身份比对，绝不与host时间相减；view摘要只是合成字符串，不是独立读取的真实RGB。
返回None表示尚未返回，不能绑定一个已返回结果。输入/结果要求完整front/left/right/back，可附up/down，严格整数/上界/顺序，bool和float戳拒绝。

比较三种**假定成功路径的算术**（不计算真实协议/状态机接受，不代替MotorGate/ReceiverLease）：

|窗口|工作期间新pending|及时结果后的假定hold|
|---|---|---|
|Gazebo现有post-compute简化|无；旧hold仍约束|result_return+.8|
|ROS已有pending简化|frame_available+.8|result_return+.8|
|候选fixed availability work-budget|frame_available+.8|保留frame_available+.8，不按完成重签|

工作期间有效期限=min(旧hold（若有）, 新pending（若有）)。结果必须严格在有效期限之前且frame/模拟身份/全部分路摘要匹配。
及时新帧结果可替换旧hold；不是永远钉在第一条命令期限。exact boundary视为过期，零宽容续租。
`work_deadline_due_at_review`只比较旧工作期限与声明检查时间；不是当前持有命令状态，也不能把及时完成后已经替换的工作期限当成仍有效的锁存依据。
`reported_result_within_window`不是协议接受或运动许可；hypothetical_hold只对该抽象模型中及时且绑定的结果给出。

### 候选性质与硬限制

候选D_available=frame_available+.8只是**可用后工作预算**。它不是D_exposure，也不能将未知曝光/交付年龄变成新鲜度证明。
缺真实曝光/设备时钟映射、认证来源、完整owner/token链时，shadow admission/native live/motion授权仍恒false。
下一次未来接口若使用该预算，只可作额外拒绝条件，绝不能拿它替代原有曝光/门控/制动净空条件。

- 首帧慢计算或慢PNG可使旧post-compute简化模型仍有新的.8窗口，而prework模型拒绝迟到结果。
- 慢capture/同步等待发生在完整帧可用之前，三种模型的新帧预算都不覆盖这段；旧hold如存在仍有独立期限。
- 生产者/owner停顿时，算术期限可以到期，但没有在此实现独立poll/下游强制零；**没有物理停车或及时锁存证明**。
- 本模型没有跨调用状态，重复同一schedule得到相同结果，不是防重放或实际sticky latch。
- 即便结果在.8内，也不表示达到原.1s计算目标。固定.4s ACK与所有速度/制动/恢复预算不变。
- 旧ACK/SIGSTOP故障、S2/S6与近场窄障碍未解决。旧2B的假曝光区间/私有进程证据不是本轮新测量。

正式driver只审计冻结源码及预先列出的合成样例，不读公共PNG、不重放原导航、不重跑controller/performance。
所有输出实际曝光null，来源认证、shadow/live/motion、物理停车、生产修复、独立执行证据均false。
