# 2P：隔离 C++ pending 消息草案解析与 Python 固定语料对照

日期2026-11-03。新增便携C++17 header及一次性stdin/stdout包装器，**不是生产native接收端、Gazebo插件、owner或admission gate**。
私有版本仍toponav_pending_offline_draft_v1 / offline_schema_only。生产wire v1、native恰8字段、status v2及runtime模块均不修改、不链接。

## 边界与接口

- `/home/sunrise/toponav_project/tools/pending_protocol_cpp_draft.hh`：标准库、无IO/时钟/回调/线程；`Decode(string_view)`仅返回归一化字段或抛Refusal。
- `/home/sunrise/toponav_project/tools/pending_protocol_cpp_probe.cc`：显式opt-in单进程、单文档stdin/stdout，无会话/操作/transport参数。最多读取8193字节，>8192拒绝；不是上游队列/反序列化/OS调度上界。
- `/home/sunrise/toponav_project/tools/pending_cpp_cases.py`：先核对固定输入pin，保存编译命令/版本/编译器和二进制hash、全部包/Python结果/C++ stdout/stderr/退出码/差异。无自动重试，不降级Python冒充C++。

严格有界UTF-8 JSON及闭合消息schema：1..8192字节、容器深度8、object16字段/key64；重复key（含解码后的转义别名）、float/非有限数、bool整数、越界、缺/多字段、能力降级均拒绝。
自包含JSON解析代码只用于本隔离草案，不是经完整模糊测试或审计的通用JSON库，不能直接放进生产或宣称无内存安全缺陷/硬实时保证。
所有合法字段/keys/ID在此闭合schema内均ASCII。C++可提前拒绝非法schema中的非ASCII超长key或不成对surrogate/超大负整数；不要求两种解析器的错误文字或拒绝阶段相同。

归一化结构与Python `asdict(decode_message(raw))`比较：kind/session/request、record、完整有序摘要、seal引用、for_kind/accepted声明、snapshot。未使用的字段null/空数组；STOP reason在两者归一化dataclass表示中都不保留，但原包及schema检查保留它。
整数以signed64保存，合法时间0..2^63-1，request/generation1..2^31-1；加法先检查available<=MAX-.8，避免溢出。-0与Python一样归一化0。

## 对照范围：仅单包 schema，不是上下文或执行一致

固定语料读取2N可重复合成器的全部message/reserve/seal/reply包，再增加转义ASCII/Unicode、UTF-8、数值与字节/深度边界。
**2N某些原“invalid”场景仅因跨包引用/ACK时限不匹配而拒绝，它们的单包schema本身合法。**本轮期望只由固定Python `decode_message`得出，不套用2N的reference/reply预期；因此wrong_frame等单包通过是正确范围，不是新漏洞修复或能力回退。
不重新实现2N reference/reply检查或2O轨迹机器，也不运行生产Python guard/native ReceiverLease。
通过固定语料只能说明这些包的接受/拒绝和成功字段一致，不能证明全输入空间等价、并发、异常调度、解析性能、来源真实性或两端执行一致。

所有C++结果classification=offline_schema_not_admission、曝光null；来源认证/曝光年龄有界/shadow/live/motion权限、独立执行、物理停车、生产修复、runtime pending安装/结果消费恒false。
过期record仍可通过schema；重复包返回同样结果，不做一次seal/消费、防重放或pending容量管理。整体替换原上下文仍可schema一致，没有受信登记/认证隔离。
accepted=true/consumed/native标签是未认证自报值；STOP仅解析，不真实发出零或停车请求。模拟戳仅身份，不读取当前钟重签或与主机钟相减。

2M十四项实现要求/25类运行时场景仍not_implemented/planned_not_executed；portable C++进程次数单独记账，不冒充新增runtime/native Gazebo实测。未重做或冒充2B已有影子owner/token。
不改控制器/模型/采集/标定/传感器/生产协议/权限/门控/0.8s lease/0.4s审计ACK/速度/制动/恢复；不构造Gazebo transport/world/publisher/actuator/step，不访问设备或真实运动。
所有开发档案采用.snapshot不可执行后缀，避免上轮同名旧测试副本污染pytest收集；不加skip/xfail/白名单、不改发现配置，保留所有拒绝/差异/失败及完整普通全量结果。
实际计算前pending/首次发送停顿/可信下游绑定、2L原两端联合重签、真实曝光/设备时钟与认证提供者、跨运行owner/native token仍缺。
0.1s（旧60/144超出）、旧1.068s晚ACK、publisher SIGSTOP flush_ack_timeout、物理停车、S2未过/S6未完、暴露四条件0/4、近场868格0支持/9远点非区域覆盖/0.25m窄障碍、普通直走验收未解决；cleanup仍unbounded join-before-detach。
