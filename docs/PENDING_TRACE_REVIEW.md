# 2O：pending 草案的角色、阶段与失败轨迹离线审查

日期2026-11-03。只对调用者提供的有界合成轨迹进行lint，不是生产协议接入、运行时owner、登记册、permit或执行端。输入中的capture/owner/controller/python/native、发送、确认、时间均是**未认证声明**。

入口 `/home/sunrise/toponav_project/tools/pending_trace_review.py` 只导入dataclasses/json和2N纯模块；无IO/时钟/sleep/线程/回调/设备/runtime导入。`pending_trace_cases.py`另负责显式opt-in固定输入及合成语料保存。没有生产路径导入它，没有速度命令。

## 格式与方向

顶层精确字段format=pending_trace_claims_v1、scope=offline_transcript_only、session_id、required_views、old_hold_deadline_ns（可null）、events。
原始输入仅1..262144字节UTF-8；深度<=12、events<=128、object<=16字段/key<=64；重复key（含转义别名）、float/NaN、bool时间、缺/多字段均拒绝。每个内嵌wire复用2N的8192字节有界schema。

每个事件精确有action/actor/target/at_ns，按种类补充wire字符串、原error、ack_unknown的request_id/backend或finish的outcome。所有时间是synthetic_monotonic_ns整数声明，模拟戳仅身份，不相减、不读取当前钟重签。

|事件|角色方向|含义（全部仅声明）|
|---|---|---|
|freeze|capture→owner|一个原record，at_ns等于available_ns，2N reserve wire|
|send|owner→python/native|已准备的reserve/seal/result_ref，或独立无帧STOP|
|reply / late_reply|python/native→owner|2N ACK/status；late_reply只留证，不升级未知结果|
|hash_begin / hash_done|owner→local|开始hash前须两端reserve确认；done携全部有序视图seal wire|
|save_begin / save_done|owner→local|两端seal确认后才能开始PNG保存声明|
|compute_begin / compute_done|owner→controller / controller→owner|seal双确认且save_done之后开始计算|
|result|controller→owner|计算done后的2N result_ref，不携issued/expires/速度|
|failure / ack_unknown|owner→local|原始错误/未知确认，随后不得继续正常工作或重试旧结果|
|cancel / diagnostic / finish|owner→local|两端STOP发送声明先于取消/诊断；失败诊断保留原错误|

## 阶段与时限

一份文档限定一个原record：freeze→reserve双发送/双确认→hash_begin/done→seal双发送/双确认→save_begin/done→compute_begin/done→result→result双发送/双确认→completed_claim。
不依赖消息交错顺序，但每端确认须对应唯一已发送请求；禁止同端重复发送、重复ACK、seal/result准备重复、请求代号回退/复用或换record。
正向工作/确认检查available<=at_ns<min(original_D,old_hold)，D仍=available+.8，边界>=立即拒绝。ACK还须send_ns<=observed_ns<=review_ns；原0.4s是审计ACK预算，不被挪用为本草案握手超时。两端result及时确认之后，completed_claim只保留新帧原D，不永久裁剪至已替换旧hold；这仍只是文档声明，不执行任何hold替换。
文档内at_ns不得回退；两端角色/会话/请求/阶段/record/摘要/seal引用须一致。跨调用/跨运行没有时钟回退检测。

STOP发送不依赖origin/seal/预算/容量，可以在old_hold已过期时声明发送。随后取消不需要STOP ACK；这只是“先请求零”的文本顺序，**既不发送真实零，也不证明后端收到、执行或物理停车**。
此处128事件限制只是文档解析资源界限，不是生产pending或审计队列容量，也不能据此声称满队列时真实STOP有保证。

## 失败、未确认与不裁剪

正常完成必须有两端result确认声明。显式failure/拒绝ACK/ack_unknown后，只允许STOP、取消、原异常诊断、晚到声明留证和failed_claim；不允许重发/重签、继续hash/保存/计算或用后续成功suffix修好前缀。
STOP双发送声明须先于cancel，失败诊断须在cancel之后并与已保留原错误一致；不修改原成功prefix。正常无帧终止可stopped_claim，不据此判断任务成功。

输出submissions保存**每个可解码send尝试**（含错误方向、重复或失败后发送），状态为unconfirmed/declared_confirmed/declared_refusal/unknown_outcome；这些都是声明状态，不是实际接收状态。
未知ACK保存原错误；晚到late_reply保存索引但永不升级unknown_outcome，局部Python成功/native未知也不能变成两端成功。正向reply不能代替late_reply修补未知结果。
语义错误返回全部violations及ledger，后缀不能消除它们；结构不可解析时抛DraftError，driver保留原始trace.bin和拒绝原文，**不伪造空账本为完整发送记录**。

## 能力边界与保留反例

- 该审查器在一次函数调用内暂存顺序变量，仅为lint。重复提交同文档返回相同结果，未实现跨调用一次seal/消费、防重放或真实pending容量。
- 若调用者整体替换所有record/available/D及合成时间，文档仍可能一致；角色名字、UUID、hash、dataclass或自报capture都不是可信来源。没有真实登记/认证隔离。
- accepted/consumed、send/STOP、begin/done、错误字符串均可由调用者编造；通过lint不是实际执行、工作耗时或独立poll活性证明。
- 只读保留2N重复快照/整组上下文替换反例及2L原两端联合重签接受反例，不重跑原后端、不把新拒绝当生产漏洞修复。
- 所有输出真实曝光null，来源认证/曝光年龄有界/shadow/live/motion权限、独立执行、物理停车、生产修复、runtime安装/消费/一次消费执行/跨运行防重放恒false。

2M十四项实现要求和25类运行时场景仍not_implemented/planned_not_executed；本轮只实际执行离线轨迹单测。未重做或冒充2B已有影子owner/token能力。
不改生产控制器/模型/标定/采集/传感器/协议/权限/门控/速度/制动/恢复、0.8s lease/0.4s审计ACK，不构造原生Gazebo/transport/world/publisher/actuator/step，不访问设备或真实运动。
实际计算前pending/首次发送停顿/可信下游预算绑定、真实曝光/设备时钟映射和认证提供者、跨运行owner/native token仍缺。0.1s（旧60/144超出）、旧1.068s晚ACK、publisher SIGSTOP flush_ack_timeout、物理停车、S2未过/S6未完、暴露四条件0/4、868格0支持/9远点非区域覆盖/0.25m窄障碍与普通直走验收仍未解决。cleanup仍unbounded join-before-detach。
