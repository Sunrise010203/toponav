# 2AA：调用方拒绝优先纯决策草案（隔离 opt-in）

API: `review_reader_decision(record_raw, expected_claim)`。输入2Z封闭read.json原字节和调用者预期2Y八字段声明；**不是生产consumer或认证验证器**。
精确bytes/131072字节上限先于哈希/解码；严格UTF8、重复key/非整数JSON数字/BOM/过深拒绝。新profile，不修改2Z/2Y/2W旧schema/错误。
先保留reader阶段/原错误/读取完成值，顶层非projection_reader_checked一律拒绝，不检查嵌套positive来升级它。unknown/null保持未知，不伪造旧执行位置或曝光。
仅顶层checked后核对精确shape、error/errno/role null、所有权限标志精确false、读取完成/顺序样本精确true、调用者claim等值、四角色顺序/引用/普通文件签名/字节界限/final_recheck精确true。
继而核对2Y和2W嵌套shape/成功阶段/精确布尔值/空错误、原分类及声明性pin/pairing常量、四文件hash/count和claim的内部等值。旧错误原标签作为reader_error保留，不重新解释协议语义。
最多仅返回offline_reference_consistent/offline_reference_review_ready；diagnostic_only=true，stateful_consumption=false。全部旧20权限/来源/执行标志及新增限制标志仍false；旧执行位置和曝光null。

## 不是可信消费或修复
纯函数不读文件/时钟/网络、无状态，不调用2Z reader、2Y/2W契约或旧validator；formal仅读取旧封闭结果，不重新进行文件系统实验。
这只是**调用者提供的JSON与预期claim的结构一致性**。JSON中的SHA、inode、元数据、来源标签和结果对象均可伪造；输入SHA仅标识字节，不是认证pin。
不一致的部分篡改/跨调用拼接可拒绝；一致伪造/整组重pin/兼容的跨调用facts拼接可以通过。重复输入会重复给出同一离线结论，不是一次性消费、防重放、freshness或运行时门控。
事件/initial_sha256只是旧临时fixture的有界元数据容器，不作来源/时间/语义证据，既不认证也不生成运行时权限。
2Z硬链接/祖先瞬时替换恢复、不相关封存配对/opaque无效观测原样保留；四文件顺序样本仍非原子快照。不能在生产中把offline_ready当permit/token或motion authority。
2W原12266项拒绝、2X首轮1064失败、旧16位置未知/66错误差异/四包装器差异、unknown迟到不升级和原2L晚重签均保留。
2M14需求/25场景仍未实现/未执行；不触及生产/native/Gazebo/设备/运动/门控/0.8s/0.4s/速度/制动/恢复；不冒充既有2B能力。普通直走验收未完成。
