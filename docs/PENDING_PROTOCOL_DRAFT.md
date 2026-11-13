# 2N：隔离的 pending 协议草案（不接生产）

日期2026-11-03。中断前创建的报告目录 `/home/sunrise/toponav_project/reports/straight-followup-20261102-23` 保留原名；完成日期以本说明/报告为准。

入口 `/home/sunrise/toponav_project/tools/pending_protocol_draft.py` 是只导入dataclasses/json的纯函数模块，无文件/设备/socket/时钟/sleep/线程/回调/运行时模块；`pending_protocol_cases.py`另负责显式opt-in的合成语料保存和审查。

**不是生产协议、owner、登记册、permit或执行端。**不修改现有Python wire v1/native八字段及status v2，不编译/构造原生Gazebo。没有任何速度命令，STOP也是仅解析的草案消息，不会发出停车请求。

## 1. 命名空间和边界

所有消息精确使用 `protocol=toponav_pending_offline_draft_v1`、`scope=offline_schema_only`，不能冒充生产version1/2。
除STOP外，capabilities必须完整且有序为reserve/seal/result_ref/pending_status；缺少/重复/降级声明拒绝，不做实际能力协商或兼容回退。

公共字段：protocol、scope、kind、session_id、request_id。额外字段/别名/生产issued/expires/速度/权限字段均拒绝。

- 原始输入仅bytes，1..8192字节，严格UTF-8；JSON深度<=8、每object<=16字段、key<=64字符。
- 在JSON解析前检查字节及忽略字符串转义的容器深度；解析时拒绝重复key（含Unicode转义后的同名key）、float、NaN/Inf和多余JSON。
- ID为有界ASCII标识，request_id/generation为1..2^31-1；时间整数为0..2^63-1，不接受bool/float。它们是草案资源限制，不修改生产容量或原0.8s/0.4s。
- 只允许synthetic_monotonic_ns。host_boot_id、session、帧、代次均为调用者合成声明，未认证；模拟戳只作身份对应，不与主机时间相减。

## 2. 消息与归一化字段

|kind|除公共字段/capabilities外的字段|局部schema约束|
|---|---|---|
|reserve|record|原点/D/必需视图声明；没有摘要、seal引用或速度|
|seal|record、view_digests|完整有序视图摘要，不可缺路/重复路|
|result_ref|record、view_digests、seal_request_id|仅引用，不含结果速度、issued/expires或权限|
|ack|for_kind、accepted、status|accepted必须bool，成功声明的phase与请求种类对应；拒绝声明为tripped|
|status|status|声明快照，不是实际独立接收/安装/轮询观察|
|stop|reason|无需record、seal、views或capabilities；reason为cancel/terminal/failure/shutdown|

record精确字段：clock_domain、host_boot_id、generation、frame_id、captured_sim_ns、required_views、available_ns、deadline_ns。
要求deadline_ns==available_ns+800000000且不溢出，不能在seal/result中单独另带issued/expires。
视图集合来自外部冻结配置比较：前四路front/left/right/back，可选up/down；digest为每路一个小写64位十六进制字符串。SHA256形状符合不等于真实像素/来源认证。

status精确字段：backend（python/native）、phase（disarmed/reserved/sealed/consumed/tripped）、record、view_digests、seal_request_id、observed_ns。
disarmed无record；reserved有record但未seal；sealed/consumed有全部摘要及seal引用；tripped可保留旧record及其绑定，或无record。这里**没有执行转换、消费token或替代现有accepted命令元数据**。

## 3. 三个纯函数及“原预算”含义

`decode_message(raw)`仅验证单包schema，返回不可变的字段表示。dataclass本身不是可信凭据。

`review_reference(message, reserve, seal, required_views=...)`对**调用者提交的快照**比较：

- message与reserve的session及record逐字段相等；record的required_views必须匹配外部完整配置。
- seal的request_id晚于reserve；result_ref晚于seal且引用其request_id和完整摘要。
- 原record被改而reserve上下文未改，拒绝；单独增加issued/expires或期限不满足+.8，拒绝。
- STOP无须上下文；不会因缺frame/预算而阻止其schema解析，但也不会真的停车。

`review_reply(...)`再比较ACK/status声明的backend、session、request_id、for_kind、phase、record、摘要、seal引用。
正向非STOP声明要求 `available <= observed <= review < min(original_D, old_hold（若有）)`；恰好边界过期，迟到ACK不延长D。
review与observed都是调用者给的同域虚构整数，不是当前钟或真实调度/独立轮询证据；不存前次时刻，不证明跨调用回退检测。
拒绝ACK保留为`declared_refusal_not_execution`，不声称原record已核对（synthetic_reference_matches=false）；STOP确认可在原帧期限之后解析，仍不是执行/停车证明。

## 4. 必须保留的反例（新增测试不是新认证）

1. **调用者把reserve、seal、result的available/D一起替换，新的三份快照仍可能一致。**此草案无法认证原availability，也没有受信登记册；2L真实原两端联合重签接受反例完全未修复。
2. 同一seal/result快照重复审查会得到相同结果，没有跨调用状态、一次消费、pending覆盖/容量或跨运行防重放保护。seal一次/consume一次仍是2M未来实现要求，不能把引用检查当成已执行。
3. 单包schema可以解析一个早已过期的record；decode不是运动admission。只有显式reply纯算术比较才检查传入review，且不实施任何zero/poll。
4. accepted=true和phase=consumed都是调用者声明；不能推断两端已安装pending、已消费、已应用命令、按时停止或真实曝光足够新鲜。

所有review输出曝光null，来源认证/曝光年龄有界/shadow/live/motion权限、独立执行、物理停车、生产修复、runtime pending安装/结果消费恒false。

## 5. 覆盖与尚未推进的范围

合成语料包含六种kind、四/六视图、正常/迟到/精确边界/旧hold先到、错字段/重复字段/错代次视图、升级降级混用、ACK错误及三种上述一致性反例。
全部输入与每项拒绝原文保存；只是离线schema/reference验证，未重做2B曝光owner影子、未运行实际控制器、没把2M的25类运行时测试标为完成。

保留生产控制器/模型/采集/标定/协议/传感器/门控/权限与所有速度/制动/恢复、原0.8s/审计ACK0.4s。
受信来源、真正计算前pending/可信下游绑定、版本化生产实链/登记一次消费/独立poll活性及真实曝光提供者均未有。
0.1s（旧60/144超出）、旧1.068s晚ACK、publisher SIGSTOP flush_ack_timeout、物理停车、S2/S6、四暴露条件0/4、近场868格0支持/9远点非区域覆盖/0.25m窄障碍和普通直走验收仍未解决。cleanup仍unbounded join-before-detach，不是物理安全保证。
