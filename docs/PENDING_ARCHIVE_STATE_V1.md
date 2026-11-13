# 2AB：隔离调用方状态与重放语义最小模型

API `review_state_trace(trace_raw)` 仅折叠调用者提供的离线JSON事件，不是生产消费器或runtime latch。
精确bytes/2MiB，严格UTF8、重复key/非整数JSON数值/过深拒绝。精确三字段`{profile,scope,events}`，scope为64字符ASCII调用者标签；最多256事件/128槽位。每事件精确`{slot,decision_utf8}`，slot为128字符ASCII标签，decision_utf8为原2AA JSON文本（至多16384 UTF8字节）或null未知。标签不允许空/斜杠/点/点点。
整个事件信封先验证再折叠；非法后缀或超限直接拒绝且无部分ledger。声明scope不是owner、时钟或凭证。

## 本次调用内的状态语义
第一次按封闭2AA结果登记：已知拒绝→local_refused，null/未知阶段→local_unknown，仅结构相符的离线正结论→local_recorded。复用未改2AA常量和claim形状helper，不调用其review函数。
同槽精确相同原始文本/null仅增加duplicate_count；**不规范化JSON、不仅凭SHA判断相同**。正结论后异字节（即使仅空白不同）使槽位进入local_conflict，永不重开。
拒绝/未知终态对任何迟到输入保持原状态和原错误/null，不被晚positive升级；冲突终态保持。所有槽记录保留首次阶段/错误/来源声明，后续输入仅在事件账本中记录。事件索引是模型列表位置，不是真实发送/接收/执行时间。
已知positive还需精确2AA输出shape/布尔/空值/声明性常量、预期claim/四hash内部匹配；一致伪造仍能符合，不查底层文件或执行语义。

## 非保证
每次函数调用从空字典重建，不持久化、不消费、不安装锁/owner/token、不执行准入/运动。相同trace可重复给出相同结果；换scope、换slot可重新登记同一内容。纯模型中的duplicate/conflict不是生产防重放修复。
整组重pin、一致伪造/元数据伪造/兼容跨调用拼接仍可local_recorded；不声明认证输入、一次性消费、跨进程owner或runtime terminal latch。全部旧20权限/来源/执行标志及所有新增限制标志均false；曝光/旧执行位置null。
2W12266拒绝、2X首轮1064失败、2Z硬链接/瞬时恢复、不相关配对、2AA一致伪造限制、16旧位置未知/66差异/四包装器差异/unknown迟到不升级/原2L晚重签均保留。
2M14需求/25场景仍未实现/未执行；不改封存schema/错误，不接生产/native/Gazebo/设备/运动，不动门控/0.8s/0.4s/速度/制动/恢复，不冒充既有2B能力。普通直走验收未完成。
