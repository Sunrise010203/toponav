# 2S：隔离C++错误分类与有界差分复核

这是对**旧2P/2Q/2R已封存便携草案**的固定seed观察和显式差异登记，不是修复旧校验器、生产native接入、runtime owner/permit/可信poll、来源认证或普通直走验收。旧文件和旧报告不改，新增C++实现0，不重做/冒充2B已有能力。

## 新文件和显式预期

- `tools/pending_cpp_difference_review.py`：只有json依赖的纯比较器；没有校验/编译/IO/时钟/运行时行为。返回分类，不发放权限。
- `tools/pending_cpp_differential_cases.py`：opt-in固定变体/编译与原始输出保存driver。
- `docs/PENDING_CPP_DIFFERENCES.json`：固定seed=2026110328，每packet family三组有界变体；每族显式登记Python/C++错误对、四个包装器不等价案例。这个登记表不是生产准入白名单。
- `tests/test_pending_cpp_differential.py`：便携对照与防“掩盖差异”单测。

命令：`.venv/bin/python -m tools.pending_cpp_differential_cases --output <新目录> --input-pins <固定JSON> --input-pins-sha256 <hash>`。22项输入固定hash、前序边界、2M十四项未实现/25场景未执行、旧2L原反例先核验；输出/编译/语料执行前要求Python整数文本上限4300，不修改进程设置。编译器缺失或超时不跳过、不用Python代替C++，部分stdout/stderr/退出码原样保存，不自动重试。

## 范围和原字节

分别编译旧2P schema、2Q reference/reply、2R trace包装器各一次，严格C++17标准库，不导入生产native/Gazebo headers。沿用旧stdin格式/字节上限/深度限制：2P单包8192，2Q TPQ1 controls4096/单字段8193/总36892，2R trace262144/深度12/最多128事件。没有扩大原schema、放宽能力字段或兼容旧生产包。
247案例：schema75、reference79、trace93。packet25族各三组固定seed变体（其中raw非法UTF8不注入Unicode wire字符串，而只测单包/原字节信封）；另11外层trace schema、4包装器不等价、10保留反例。部分输入会重复，不把case ID唯一冒充字节唯一或覆盖率。固定seed不是随机性、fuzz覆盖或全输入等价证明。
每例input.bin（协议包/TPQ1信封/trace原字节）、reference原包/controls、case规则、Python结果、C++原stdout/stderr、退出码、标准化事实和差异路径全部保存。非法UTF8不做replacement decode，不重编码包内容。Python/C++事实区分returned_result（有无结果）和consistent；trace语义拒绝仍有完整账本，不混成schema拒绝。

## 四类分类，不制造等价

1. `exact`：接受/拒绝、阶段、错误文字、全部结果类型敏感canonical JSON完全相等。
2. `refusal_error_only`：双方均无结果且拒绝，只允许本案例显式错误对不同；原错误均保留。
3. `trace_violation_codes_only`：双方有结果，只有指定事件索引的已登记违规错误文字不同；两次review、所有索引/顺序/状态/disposition/时间/发送/失败/回包索引/权限字段均须一致。只替换审查用副本中的已登记标签，原始结果不改。
4. `reference_envelope_refusal`：四个显式案例Python返回STOP/拒绝声明审查，C++更早拒绝信封/controls。**这是接受结果/阶段不等价，不是仅错误文字不同，不是修好了。**

新/未知标签、错索引、丢条目、失败错误改写、未知结果升级、跨调用重复结果差异、bool/int/float混同、权限/exposure改变，均不能以登记错误对掩盖；unregistered_difference或执行失败必须使本轮检查不通过。`registered_expectations_match=true`只表示观察符合固定登记预期；`all_results_exactly_equivalent=false`必须保留，绝不改称“所有结果完全一致”。

## 已登记差异示例及账本含义

- escaped孤立surrogate身份：Python message_identity，C++ unicode_surrogate。
- 超大/负溢出数：Python record_ns/message_json，C++ integer_bound；5000位整数需要固定的Python4300位限制。
- Infinity/NaN：Python integer_json_only，C++ message_json。
- 视图元素类型：Python view_schema，C++ view_order_or_duplicate。
- 拒绝ACK阶段不对：Python refusal_phase，C++ ack_phase。
- Unicode未知key的字符/字节界或17字段优先级：message_fields/object_key_bound、object_field_bound/duplicate_field等。
- 外层trace字节/UTF8/JSON/深度错误使用trace_*，旧C++解析器仍使用message_*/json_depth_bound等。
这些非法内层wire双方都在解码前拒绝时，均不产生该send条目；其他可解码send（包括错误方向/重复/故障后尝试）和失败前缀须完整保留。不同错误标签可能影响消费错误字符串的调用者，因此不称为可互换实现；本轮没有任何这样的实际调用者接入。

四个包装器差异：STOP/拒绝ACK忽略的required_views含float时，Python早返回但C++controls整数JSON拒绝；STOP忽略的surrogate配置C++先拒绝；STOP忽略的reserve8194字节超出信封字段8193上限，Python纯函数不检查它。原2Q文档已披露信封域限制，本轮提供显式合成证据，不扩大C++信封或弱化schema来迎合Python。

## 保留的能力边界

整组上下文/available/D/轨迹替换仍可一致；重复调用无消费；未知结果不能被迟到声明升级。迟到声明的未来观察时间/错误原record仍可能作为未授信附注保存，不是独立poll。STOP只是解析/声明，不发零或证明物理停车。
旧2L原Python/receiver晚联合重签证据只读保留，仍两端accepted=true且D>10.8；不重跑/以当前钟重签、不宣称生产漏洞修复。
所有比较器报告及旧包装/成功/语义拒绝review真实曝光null、认证/曝光年龄有界/shadow/live/motion权限、独立执行、物理停车、生产修复、runtime安装/消费/一次消费/跨运行防重放均false。没有实际后端实验、设备、运动或真实transport/world/publisher/actuator/step。
生产控制器/模型/标定/采集/传感器/协议/门控/权限/0.8s/0.4s/速度/制动/恢复均未改。可信曝光/设备时钟/认证源、计算前pending/首次发送停顿/可信下游预算、跨运行owner/native token、0.1s、旧晚ACK/IPC、物理停车、S2/S6、暴露四条件、近场窄障碍与普通直走验收仍未完成；cleanup仍unbounded join-before-detach。
