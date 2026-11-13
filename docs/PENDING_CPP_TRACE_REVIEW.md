# 2R：隔离 C++ 有界轨迹快照审查对照

**纯离线、opt-in、固定合成轨迹声明 lint，不是运行时 owner/登记册/permit/可信 poll、生产 native 接收端或普通直走验收。** 本轮不改旧封存文件，也不重做/冒充2B已有影子owner/token能力。

## 新文件和接口

- `tools/pending_trace_cpp_json.hh`：从2P有界JSON解析器派生的独立外层trace解析器；复用2Q纯函数依赖，不修改旧header。
- `tools/pending_trace_cpp_review.hh`：ReviewTrace(raw)，只在一次调用内维护顺序/请求/发送/错误账本，返回值不是令牌。无IO、时钟、线程、回调、runtime对象或跨调用状态。
- `tools/pending_trace_cpp_probe.cc`：独立stdin/stdout包装；无CLI操作参数，一次进程至多读取262145字节（上限+1用于拒绝），一次schema合法输入在同一进程纯审查两次。
- `tools/pending_trace_cpp_cases.py`：固定语料、严格便携C++17编译及Python逐项对照，不调用真实后端。
- `tests/test_pending_trace_cpp.py`：便携独立进程单测；不冒充安装/消费/物理停车实验证明。

命令：`.venv/bin/python -m tools.pending_trace_cpp_cases --output <新目录> --input-pins <固定JSON> --input-pins-sha256 <hash>`。创建输出/编译/生成运行语料前验证全部固定文件、2M未实现/未执行状态、2Q能力边界和2L原始反例。编译器缺失、超时/失败不跳过、不回退Python；保存stdout/stderr/退出码，不自动重试。

## 原始字节与资源限制

只接受2O `pending_trace_claims_v1` / `offline_transcript_only` 私有JSON；trace上限262144字节、深度12、事件1..128、对象最多16字段/key64；重复key（含转义别名）、缺/多字段、浮点/bool整数/越界/非有限数等拒绝。128仅为离线资源限制，不是生产pending容量。
事件内wire仍保留调用者字符串，按Python字符数<=8192检查，再编码UTF8送2P单包schema（字节上限8192），不先解析/重编码wire，原trace.bin始终保存。不能把字符串格式不同但字段相同的send冒充已准备请求：准备与send的原wire必须逐字节相等。
外层JSON保留escaped孤立surrogate为内部CESU-8，wire实际编码失败记为event_wire_utf8；账本backend等孤立surrogate输出为\udxxx逃逸，不产生非法UTF8 stdout；合法surrogate pair组合为Unicode，raw UTF8 surrogate仍拒绝。
所有合成ns为0..2^63-1整数；请求号1..2^31-1。不用double减时间，不读取当前钟，不重新签署旧记录。模拟采集戳只是record身份字段。
这是封闭草案手写解析器/语义移植，不是通用JSON库、全输入等价、fuzz/内存安全/性能/硬实时审计。固定语料成功结果逐字段、类型敏感canonical JSON一致；顶层schema拒绝文字/阶段不要求相同。复用2P解码器的非法内层wire错误文字可能与Python不同；不能推广固定语料到所有非法JSON错误分类的全域等价。

## 顺序与失败账本

对齐2O Python：capture冻结唯一原reserve；两端reserve声明确认后hash；hash_done准备seal；两端seal声明确认后save/compute；save完成和compute完成后准备result_ref；两端result声明确认后可completed_claim。hash/save/compute/send/reply预算绑定原D及旧hold；完成时已有两端及时result声明，故completion只看新原D，不把已替换旧hold永久裁剪新帧完成。
每个可解码send在检查session、时间、方向、故障/终止状态前记账；坏方向/重复/故障后尝试不丢弃。未知ack记录unknown_outcome和原错误；常规reply不能事后升级，late_reply仅在故障后给未确认条目附加索引，**不升级结果**。它并不重新验证完整record/期限；未来观察戳或错误原record也可能被附注，正是“仅记录未经信任迟到声明”的能力边界，不是可信活性/确认。
拒绝ACK是declared_refusal，不是实际执行证明。失败前缀不能被成功后缀洗掉；原错误需先两端STOP声明、cancel，再diagnostic保留，最终failed_claim。STOP不要求帧或活预算；发送STOP并不真实发零或证明物理停车。stopped_claim不强制STOP ACK，仍允许未确认STOP账本。所有事件均为调用者声称发生的事件，不运行计算、保存、取消或停止。

## 保留反例和能力边界

整组轨迹/record/available/D/观察戳一起替换仍可一致；重复调用相同输出，不是真正一次消费、防重放、来源认证、独立执行或跨运行owner。0.8s原lease、0.4s审计ACK未改；2N/2O本来没有独立增加0.4s审查上限，这里不臆造新门槛。
旧2L原两端晚联合重签反例只读保留：Python/receiver仍accepted=true且D>10.8；不重跑、不给旧记录当前钟重签、不宣称生产漏洞修复。
所有包装、成功及语义拒绝review：actual_exposure_wall_s=null；来源认证、曝光年龄有界、shadow/live/motion权限、独立执行、物理停车、生产修复、runtime pending安装/消费、runtime一次消费和跨运行防重放12个字段恒false。一致或failed/stopped/completed_claim都不能升级这些字段。
2M十四项要求仍not_implemented，25运行时场景仍planned_not_executed；新增真实runtime/native/Gazebo后端实验0。
不改生产控制器/模型/标定/采集/传感器/协议/门控/权限/0.8s/0.4s/速度/制动/恢复；不构造真实transport/world/publisher/actuator/step，不接设备或真实运动。
可信曝光/设备时钟/认证源、计算前pending/首次发送停顿/下游预算、跨运行owner/native token、0.1s、旧晚ACK/IPC、物理停车、S2/S6、暴露四条件、近场窄障碍与普通直走验收仍缺；cleanup仍unbounded join-before-detach。
