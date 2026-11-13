# 2U：TPT1 组合边界与拒绝优先级（隔离审计）

本次不改 2T profile、C++ namespace、探针、原 Python/C++ reviewer、schema 或生产代码。
新增固定语料、纯观察比较器、机器审计规格与测试；不是新增协议或 endpoint。
seed=2026110330。基线完整继承2T的243输入，不能抽掉66项内层错误标签差异或旧四项包装器差异。

## 优先级不是“更详细的错误总是优先”

以下顺序对应2T既有契约，本审计并非改写校验器：
1. 精确 bytes（Python边界独有，跨语言只比较字节）→总长≤36892→TPT1魔数。
2. **按字段0..4依次扫描**：4字节长度头存在→必需字段不能null→非null长度不超cap/剩余字节。
   先发生的结构错误优先。错误在controls内容中，不等于先于后续字段结构错误：
   比如controls={}且seal超长，先报domain_field_bound，不报domain_controls。
3. 无尾随数据→controls规范ASCII JSON/闭合参数约束→reply操作必须提供非null reply。
4. 只有完成以上步骤才进入旧reviewer。域内接受不是schema成功、执行或admission。

多重故障包含：总长、魔数、必需null、reserve/seal/reply超长、尾随数据、无效controls、
reply缺失与message UTF8。互斥的同字段写入组合不生成（控制null/无效、message null/UTF8、reply超长/null）。
全部兼容单故障与双故障，加固定种子抽样24个兼容三故障，两个稳定字节variant。
每个双/三故障可移除首故障，在同variant的既有单/双故障中找到反事实输入，
检查后继故障的登记输出。first_fault只表示构造所预测的首故障，不是旧入口新增的诊断字段。
**若前后故障返回相同错误码/阶段，输出不能唯一定位实际命中字段**；此时只检查相同标签和
反事实字节构造，不宣称已观测到字段定位或区分了两个故障。报告分别统计标签改变和同标签关系。
不能把这说成所有故障组合或全输入域的形式化证明。

## 长度与不透明字节性质

- 五字段分别覆盖0、1、cap−1、cap、cap+1、0xfffffffe、null-marker的声明/原始负载。
  0xfffffffe不分配巨量内存：保留巨大声明及短原始字节，期待结构拒绝。
- STOP和拒绝声明的reserve/seal/reply分别覆盖null、空、非法UTF8、8192/8193/8194。
  拒绝ACK的reply本身不可忽略；reply null在域层拒绝，其他非法reply在旧schema拒绝。
- message补空白到8191/8192/8193/8194：8192仍可schema成功，8193域内但旧schema拒绝，8194域外。
- controls补空白到4095/4096/4097：前两项结构域可容纳，但不规范而拒绝controls；最后字段超长。
- 总长36891/36892/36893：前两项用非规范4096字节controls和各字段cap构造，
  期待domain_controls，**不是**“4096字节controls可规范有效”的声明；最后总长先拒绝。
- 在已知合法信封的头/负载边界截取前缀：根据预先声明的字段前沿预测
  domain_magic/domain_length/domain_field_bound；完整前缀才可进入旧reviewer。

所有原始信封均保存。null与空保持区别；不预解析/重编码packet，不丢掉ignored字节。
每输入两次Python及两次便携C++评审，完整重复输出须一致；这只证明该固定调用的可重复性，
**不是**运行时一次性消费、跨运行防重放或执行了声明请求。

## 比较与不可越界声明

纯比较器只检查固定case的登记stage/error pair、完整成功结果和权限字段；任何错误差异
只能来自原基线逐例登记的原标签对，绝不删错误、状态、索引或账本以制造等价。
只有expectation_matches表示本次固定观察符合预期，不表示整个共同域全结果相等。
所有曝光null，十二外层/十内层权限标志false。views_frozen/所有时间/后端名仍是调用者声明。
不增加纯评审0.4s执行保证，不改0.8s原预算或oldhold较早截止。

整组替换、重复调用、拒绝matches=false、旧trace unknown/迟到不升级、2L原两端晚重签都保留。
trace只读封存证据，不接入新profile、不重跑2L、不新签截止。2M14要求仍not_implemented、
25运行时场景仍planned_not_executed。不实现runtime owner/permit/token/可信poll，不冒充2B既有能力。
生产native/Gazebo、控制器、门控、权限、设备、运动、速度、制动、恢复均不变。
物理停车、实际曝光/设备时钟/认证源、普通直走验收等仍未解决。
