# 2V：只读字节定位证据 v1（独立 opt-in 草案）

## 评审结论：需要区分三种不同的“位置”

2U的16条关系，旧结果在阶段/错误标签上相同。若需要区分输入发生了什么，
只保留错误字符串不够；但额外再跑一个独立检查器，也**不能证明旧校验器实际执行到了哪一行**。
本草案只补充只读原字节事实及显式预测，不伪造旧入口执行轨迹。

独立profile `toponav_pending_location_evidence_v1`，纯Python入口
`tools.pending_location_evidence.inspect_evidence(raw_bytes, observation_snapshot)`。
不改2T/2U封存源码、旧输出字段/错误/schema，不提供C++实现、生产兼容或运行时入口。

三层信息严格分离：
1. `byte_facts`：直接由所提供原字节计算的长度头、null、cap、负载完整性、字节offset、
   已实际存在的负载SHA256；只能说这些字节存在，不是设备状态、旧分支或曝光证据。
2. `predicted_first_structural_failure`：按照已声明TPT1外壳顺序、独立扫描所预测的结构拒绝位置；
   `basis=readonly_byte_scan_contract_prediction_not_execution`，不是instrumentation。
3. `observation_snapshot`：完整克隆的旧JSON声明结果，按2U原纯比较器的形状规则检查；
   它的stage/error/result原样保留，仍未认证、未与raw绑定。
   **`observed_validation_site`始终null**；`old_validation_site_observed`与
   `unique_field_localization_observed`始终false，即使预测reason与声明error相等。

## 有界扫描，不是新消息校验器

精确bytes，最多36893字节（旧信封cap36892+1，用于观测超限），超出或类型不符直接
`evidence_refused`，不哈希巨量输入。不合形状的旧声明也拒绝，不丢弃原审计输入文件。
旧声明按闭合shape限制字典层级/成员数，error最多4096个Python字符，超出直接拒绝、
不截断/重写错误标签；该限制只属于新证据profile，不改变旧validator。
所有证据返回均**不表示消息有效**。空包/坏magic可以有完整的拒绝预测证据。

先记总长/magic事实；总长超过旧cap或magic不符时不继续猜字段布局。
否则按五字段读取big-endian uint32头：记录null（不合并为空）、实际负载长度/摘要、cap
与截断事实，不分配声明的巨大uint32长度。遇不完整头/负载就停止。
遇到已完整编码的非法必需null或超cap字段，可以继续收集后续**字节事实**；
它们绝不是“旧校验器也继续执行了”的证据。
第一结构预测按总长、magic、每字段头/必需null/cap或剩余长度、尾随数据的次序产生。
不解析controls JSON、不执行required_views/backend/time语义、不解析/重编码任何内层packet。
所以无结构预测并不表示controls/schema/reference成功；这些层的定位明确未知。

`reported_reason_agrees_with_prediction`只比较所给声明的domain_refused/error与结构预测。
没有结构预测时为null；相同为true、冲突为false。即使true也不认证来源、绑定输入或定位旧PC；
false也不能据此指控旧校验器有bug，因为所给声明可能根本来自其他输入。
两个SHA256仅固定本次输入及声明克隆的字节身份，不是签名/可信provenance。

## 封闭输出语料与负向例

读取2U封存的532个原输入及Python/C++声明快照，**不重跑旧domain/schema/reference/trace reviewer，
不编译或执行C++探针**。新诊断全部由Python执行；输出python/cpp键仅表示旧快照的归档来源，
不是本次执行了两个语言实现或任一真实后端。每来源两次调用须完整一致。
保存全部原信封与声明、新证据与重复证据。旧66项内层错误标签差异原样留在声明快照中。

重点检查16条同标签关系：旧声明摘要相同，但原字节与独立结构预测字段可区分；
旧实际执行位置在两端仍全部未知。这是新只读诊断可见性，**不是把旧不可见性补写成已观测**。
另加人工声明错配（有效声明配坏字节/错error配坏字节/拒绝声明配结构正常字节）、
整组替换、ignored负载改变、证据界限、巨大声明头、继续扫描与非法权限/形状声明。
人工错配不改归档旧结果，不能算旧校验器新增错误或运行时实验。

## 不变的边界

所有曝光null，旧十二权限标志和新的绑定/执行定位标志全false；不消费请求、不防跨运行重放。
拒绝matches=false不证明原记录匹配；整组reserve/seal/result替换与重复调用仍可一致。
旧四包装器差异、66内层标签差异、unknown/迟到不升级、2L原两端晚重签全部保留；
2L只读原字节、不重跑/重签。2M14需求仍not_implemented、25运行时场景仍planned_not_executed。
不新增runtime owner/permit/token/可信poll，不冒充2B既有能力，不改变0.8s/0.4s/门控/速度/制动/恢复。
无生产native/Gazebo/设备/运动修改，无物理停车证明或实际普通直走验收。
