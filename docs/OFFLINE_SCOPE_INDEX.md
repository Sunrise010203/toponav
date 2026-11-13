# 多scope显式清单与会话连续性审查（2H）

`tools/offline_scope_index.py` 仅审查调用者给定的离线闭合scope清单。无自动发现、live来源、IO/控制器/探针构造、回调、线程、当前时钟、运行时owner注册表、token或运动桥接。生产入口不导入该工具；原控制器/相机/模型/标定/协议/权限/门控/预算均不改。

## 调用与冻结边界

```bash
.venv/bin/python -m tools.offline_scope_index \
  --root /absolute/evidence-root \
  --manifest /absolute/index.json \
  --manifest-sha256 independently-held-sha256
```

API：`review_index(root, manifest, manifest_sha256=...)`。外部独立保存的清单SHA256为必需参数；无效/不符时不读取任何scope。
索引先按独立摘要核对字节，再整体检查路径/七文件pins/schema，**全部schema通过后**才读取任何scope。
清单文件及祖先不跟随符号链接，两次读取一致并在审查结束再次核对清单字节；实际scope使用原2G NOFOLLOW普通文件只读入口。

格式固定`offline_scope_index_v1`、coverage固定`declared_window_only`：

- sessions：1—32项，每项显式给session/config/provider/主机clock/boot、source_kind、完整有序CameraSpec、initial_counts、terminal_counts及scopes。
- 每session 1—32个scope，总计≤64；清单≤256KiB。JSON边界沿用2G严格解析，禁止重复键/非有限数/超限结构。
- scope：scope_id、根目录内相对path、独立七文件SHA256（含status）、predecessor_scope_id及predecessor_snapshot_sha256。
- 相对路径每级非空，禁止绝对路径、`.`/`..`、反斜杠及NUL；不跟随工件中的路径，不进行glob或目录发现。
- 清单内session ID、scope ID及相对路径分别唯一；不允许相同scope换session再次入列。
- 首scope两项predecessor必须null，表示**声明窗口起点**，不意味着真实session起点或计数必须0。
- 后续predecessor指向前一声明scope及其规范化snapshot摘要，不是原生生产者签发的链或不可伪造日志。

工具只向stdout返回冻结结果，退出0仅表示本声明窗口诊断一致；原body_failed也可以保留为recorded_failure且父清单一致。退出0绝非场景成功、安全或导航通过。
无修复/拼接/回填/清除失败；未知额外目录和文件不属于声明窗口，不能据此声称完整档案已审查。

## 逐项与跨项核对

每个声明scope先执行2G七文件只读审查。跨项元数据再次读取且必须同时匹配清单pins和同次子审查全部七文件摘要，不把另一份不固定元数据混入边界检查。
逐项核对实际scope ID、session/config/provider/clock/boot/source/相机集合与清单上下文一致。

同session相邻scope：

1. 前closed_s≤后started_s；只在已核对相同主机时钟域/boot的上下文中比较，不做模拟戳与monotonic相减。
2. 前最终四类累计计数必须精确等于后counts_before；首基线等于显式initial_counts，末最终计数等于terminal_counts。
3. observer_errors不可倒退；前scope留下原IO error则不能声称同owner继续形成下一scope。
4. 原64环中两边**共同保留**的同ID历史记录必须逐字段一致。允许有界环丢弃旧记录，但不重建已丢部分，也不将丢弃当成完整历史。
5. 原失败/诊断拒绝保持原类；日志不完整/损坏scope不能充当后续连续性锚点。

对schema合法的清单，任一子包/跨项失败仍保留**全部声明项**的子审查结果。任何失败令父清单整体不一致，不输出“成功子集通过”。
如果整个清单schema非法，因尚不能安全读取任何子项，结果没有子审查，不称其余子项已验证。
verified_edges仅统计已比较一致的局部边；父清单可能因另一个scope/端点失败，不能单用边数或局部子结果宣称整体通过。
所有父/子结果曝光仍null，provenance/shadow/live/motion四授权恒false。

## 无法证明的内容

SHA256与本地session/scope/boot标签只能固定调用者持有的字节，不能认证真实传感器、证明线程过去实际退出或构成生产者防篡改链。
重复调用同一完整清单可以再次一致；本工具无跨运行持久状态，**不是防重放owner**。
允许明确声明后半窗口并给非零初始计数；这不证明前半窗口不存在或整个session没有遗漏。
零活动scope也不一定改变计数。保留旧predecessor链接时可检出清单内删项；若清单/链接/摘要也共同重新声明，不能恢复被共同删去的零活动scope。
共同替换session标签、索引及全部pins后仍可能局部一致；独立原索引摘要只能检出与其不符的改写，不是身份凭证。
相同声明窗口之外的遗漏、跨运行复制、敌对文件系统/OS、SIGKILL或存储崩溃不由此保证。
要求静止闭合包；多次读取不是原子多目录快照、文件锁或实时可用性保证。

## 验证范围

- 原任务15真实保存的重挂两scope：一条连续边，不重建回调。
- 原23个scope显式分为22个session：三个原日志故障仍不完整，父清单整体不一致，全部23项保留。
- 保存文件损坏/边界场景检验摘要、schema、重复、路径、上下文、端点计数、前驱、删项/倒序、子包损坏、历史换写与错误继续。
- 人工插入空scope桥、原body失败声明和其他改写只属于**合成保存数据fixture**，不是恢复/真实观测到的新scope。
- 保留重复清单、明确后半窗口及共同重标三个不保障反例，不计来源认证/防重放/全档案完整性通过。

不重新执行原导航controller.update，不做性能/0.1s预算改善或物理停车结论。真实曝光/设备映射、认证来源/跨运行防重放、原生token、Gazebo计算前期限与实际停稳仍未解决。
