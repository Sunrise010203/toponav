# 2T：隔离共同输入域 v1（仅 opt-in）

## 范围与非等价性

新增 profile `toponav_pending_common_domain_v1`，Python 纯入口
`tools.pending_common_domain.review_envelope(bytes)`，C++ 纯入口
`toponav_pending_common_v1::ReviewEnvelope(string_view)`。独立探针只读 stdin，
输出 JSON；不是 native 生产端、传输协议升级、生产消息兼容层或权限入口。
旧 2N/P/Q/R/S 文件及其语义不变。共同域只是调用者数据的便携表示约束，
不是认证来源、可信时钟、runtime owner、admission 或设备状态。

**原 Python API 不等价于旧 TPQ1 包装器的全输入域。** 2S 四项差异仍然成立：
STOP/拒绝声明可以忽略浮点 views，STOP 可忽略 surrogate views 和8194字节 reserve；
旧 TPQ1 会在控制 JSON/信封层先拒绝。新 TPT1 则明确要求所有调用先做域检查，
这有意收窄可接受输入，不是修复或重新解释旧函数的结果。

## 有序域检查

1. Python 必须为精确 bytes（拒绝 bytearray/memoryview/str/None）；C++ 对应字节视图。
   总长至多36892字节；魔数 TPT1。探针最多读取36893字节以检测超限。
2. 五个字段 controls/message/reserve/seal/reply，交错 big-endian uint32 长度和原字节。
   0xffffffff 仅可表示后三字段的 null；空字节与 null 不同。controls≤4096；
   其余每个≤8193。禁止尾随数据/截断。先检查全部信封字段，再解析 controls。
3. controls 必须是**规范 ASCII JSON**：字典按键排序、紧凑、无额外空白、无重复键、
   字符串无冗余转义；恰好六键 operation/required_views/views_frozen/backend/review_ns/
   old_hold_deadline_ns。非规范输入直接拒绝，绝不先规范化接受。
   operation 仅 schema/reference/repeat/reply；views_frozen 必须 true；views 必须是有序
   front,left,right,back，后面可无、up、down、up+down。布尔/浮点/非有限数/指数/
   surrogate/非ASCII视图、负数、超int64整数不在域内。
4. reply：backend 为 python/native，review_ns 为非布尔整数[0,2^63−1]，oldhold 为 null
   或同区间整数；reply 字段不能 null（可以为空，随后由旧 schema 拒绝）。
   非 reply 的 backend/review_ns/oldhold 必须均为 null，即使旧路径会忽略它们。
   views_frozen 只是调用者配置声明，不是可信冻结或 owner 授权。

所有控制参数无效统一返回 `domain_controls`；这是新域的粗粒度错误契约，
不是把旧 reviewer 的错误重写成一样。信封结构错误按上述顺序给出独立 domain_* 错误。

## 输出阶段与比较承诺

- `domain_refused`：domain_accepted=false、consistent=false；未调用旧 reviewer。
- `review_refused`：domain_accepted=true、consistent=false；旧 schema/reference 拒绝，
  result=null，**原错误文本完整保留**。
- `review_returned`：domain_accepted=true、consistent=true；旧纯评审返回声明结果。
  consistent 不表示执行成功、认证通过、尚未过期或准许运动。

域内原始 packet 不预解析、不解码后重编码、不丢弃，包括 ignored reserve/seal/reply。
8193字节在信封域内但可能被8192字节旧 schema 拒绝；ignored malformed/empty/null
上下文仍由旧 STOP/refusal 分支决定是否使用。新域并未把全部 packet 限制成 schema-valid。
因此**不承诺整个域内所有错误标签精确等价，也没有全域证明**：固定语料比较
阶段/结果/类型/权限字段，成功结果须完整精确相等；仅逐例登记的旧错误标签差异
可以被记录为差异，原错误保留。不得以删除错误、索引、账本或状态来制造等价。

## 必须保留的限制

所有曝光为 null，十二个 authority/runtime 标志始终 false；内层旧结果十标志 false。
时间均为调用者 synthetic 整数；sim stamp 仅作身份，不与当前主机时间相减或重签。
保留旧0.8s原截止/oldhold更早截止，未新增或宣称纯评审具有0.4s执行保证。
整组 reserve/seal/result 可一起替换仍一致；repeat/跨次调用可相同，不消费令牌。
refusal 的 matches=false 不是原记录匹配证明。旧 trace 未接入新 profile，
其 unknown/迟到不升级/全部发送与失败账本按2S封存证据保留，未重新实现为新能力。
2L 原两端晚重签反例仅按原字节读取，不重跑、不重签。

2M十四项需求仍 not_implemented、25场景仍 planned_not_executed；不新增可信poll、
runtime owner/permit/native token，不把2B已有能力算作新交付。没有生产、Gazebo、
设备、运动、门控、速度、制动或恢复修改；普通直走尚未完成真实验收。
