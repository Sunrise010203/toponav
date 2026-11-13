# 2Q：隔离 C++ 调用者快照跨包引用与 ACK 声明草案

这是 **opt-in、纯离线、固定合成语料** 比较，不是 native/Gazebo 接收端实验、生产接入、owner、登记册、permit 或运动授权。旧 2P header/probe 和所有旧封存文件不改。

## 文件与接口

- `tools/pending_protocol_cpp_reference.hh`：只包含2P纯schema header与标准库optional；ReviewReference/ReviewReply接受原始字节、调用者上下文、冻结视图声明、合成整数时间。无IO、时钟、线程、回调、状态或runtime对象。
- `tools/pending_protocol_cpp_reference_probe.cc`：独立进程的一次stdin/stdout包装，不是transport。无CLI操作参数。
- `tools/pending_cpp_reference_cases.py`：固定语料、严格C++17编译、逐项输出、Python对照；只有显式运行CLI才正式审查。
- `tests/test_pending_cpp_reference.py`：便携独立进程单测；这些执行不是实际两端的安装/消费实验。

正式命令：`.venv/bin/python -m tools.pending_cpp_reference_cases --output <新目录> --input-pins <固定JSON> --input-pins-sha256 <hash>`。hash/schema/全部文件/2M和旧2L反例在创建输出、编译及运行语料前核验；不回退Python，不自动重试，不跳过编译器缺失。

## 有界包装，不重编码原始消息

stdin使用私有二进制TPQ1信封，非生产wire：magic四字节，随后五个交错的big-endian uint32长度+原始字节字段，顺序controls/message/reserve/seal/reply。0xffffffff表示缺省，仅允许后三字段；0与缺省不同。controls上限4096字节、单包传输上限8193字节（允许把8193超界样本送给8192上限schema拒绝），信封总上限36892字节。包装最多读36893字节，严格检查魔数、截断、长度、尾随及上限。不吞掉非UTF8、不把非法包先解析再序列化。忽略上下文的STOP/拒绝ACK也受**信封本身**上限约束；Python函数没有这个传输信封，不宣称域外等价或无限流实时性。

controls恰含operation、required_views、views_frozen、backend、review_ns、old_hold_deadline_ns。views_frozen布尔仅编码Python tuple声明，**不是可信冻结配置**；required_views仍验证有序完整视图。operation仅reference/reply/schema/repeat；schema用于保留2N过期单包与disarmed例，不冒充跨包通过；repeat在同一进程调用纯函数两次，不消费。controls使用2P有界整数JSON解析器，浮点或超范围控制数提前拒绝，拒绝阶段/文字不与Python强行对齐。正式对照只声明固定语料结果与成功字段的类型敏感canonical JSON一致，不宣称通用等价、fuzz覆盖或硬实时。

## 跨包与回包规则

沿用2N：原始reserve的整份record（含模拟采集戳，仅标识）、session、有序视图配置相等；请求号顺序、seal上下文与结果摘要、seal_request_id一致。result必须绑定seal及其原reserve；即使reserve/seal操作不需要seal，**提供的可选seal也必须先解码**。STOP先解码自身，随即不检查reserve/seal/视图配置，返回frame_free_stop_schema_only。

ACK/status需请求种类、后端声明、session/请求号、ACK for_kind、阶段、观察时间及所有record/摘要/seal声明匹配。正向非STOP需 available <= observed <= review < min(original_D, old_hold_D)，整数计算，不使用浮点、不延长D；原D始终available+800000000。old_hold不存在时只用原D。没有新增或改变400000000ns审计ACK规则：2N的review_reply并未另加0.4s上限，本草案也不臆造新限制。

拒绝ACK在身份/端/合成观察时间/for_kind校验后提前返回declared_refusal_not_execution，synthetic_reference_matches=false；它**不解码上下文、不证明原record匹配、不检查原预算**。正向STOP仍检查回包身份/端/观察时间，但无帧预算/上下文；不实际发零、停车或证明物理停车。status不是独立可信poll，也不是已安装/已消费事实。

## 必须保留的反例与缺口

- 整组reserve/seal/result及available/D一起替换仍可一致；不是来源认证或跨运行防重放。
- 重复调用相同输出，没有真正一次seal/消费、pending容量、runtime owner/生命周期、下游可信预算绑定或独立poll。
- 过期record仍可schema/自洽引用通过；正向ACK到D或old_hold边界拒绝，不等于真实超时执行。
- 旧2L两端联合重签原反例只读保留，accepted仍true，D>10.8，不重跑/重签旧证据，不声称生产漏洞修复。
- 所有成功/拒绝包装和成功review：真实曝光actual_exposure_wall_s=null；来源认证、曝光年龄有界、shadow/live/motion权限、独立执行、物理停车、生产修复、runtime pending安装/结果消费十个字段均false。
- 2M十四项要求仍not_implemented，25场景仍planned_not_executed；不冒充2B已有能力的新实现。

不修改生产native/Gazebo插件/控制器/模型/传感器/协议/门控/权限/0.8s/0.4s/速度/制动/恢复。不构造真实transport/world/publisher/actuator/step，不接设备或运动。真实曝光与设备时钟、认证源、计算前pending/首次发送停顿/下游绑定、跨运行owner/native token、0.1s、旧晚ACK/IPC、物理停车、S2/S6、暴露四条件、近场窄障碍、普通直走验收仍未解决。cleanup仍unbounded join-before-detach。
