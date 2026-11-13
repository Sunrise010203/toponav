# 离线闭合scope证据包只读审查（2G）

`tools/offline_scope_review.py` 是独立 opt-in 的保存文件审查器，不是运行时门控。
只使用标准库文件读取与原纯数据 receipt_snapshot_adapter；不导入或构造 GazeboIO/导航控制器/探针，不启动线程、回调、transport、发布者、执行器或world，不读取当前时钟。
生产入口不导入它；控制器、相机/模型/标定/协议/权限、超时、门控、限速/制动/恢复不变。

## 入口

```bash
.venv/bin/python -m tools.offline_scope_review --scope /absolute/closed/scope --pins /absolute/independent-pins.json
```

pins 文件是固定七个文件名到SHA256的小型JSON字典，**包括status.json**。调用者应从独立冻结的交付清单提取，不能仅复用包内自报摘要。
CLI只向stdout输出JSON，无修复/写入/覆盖/自动找回路径。诊断一致且工件完整返回0；不完整或损坏返回2。**0不是场景成功、认证、安全或导航通过。**
无pins也能做局部一致性审查，但结果明确为`trusted_bytes_match=null`、`unpinned_local_consistency_only`。
匹配pins只代表与调用者提供的字节相同；本地清单或UUID/boot标签不是经过认证的来源凭证。

Python API：`review_scope(path, trusted_files=None, external_inputs=None)`，返回冻结的ScopeReview。
external_inputs是调用者独立持有的完整输入列表，每项只有expected（frame_id/captured_sim_ns/完整有序view_digests）与composite_sha256。
审查器不接受输入文件路径，也不自行定位图像或跟随记录中的路径。独立PNG读取只在隔离审查driver中完成。
不接受live来源、设备时钟映射、曝光或新运动授权。

## 文件边界

固定读取begin.json、events.jsonl、snapshot.json、scope.json、inputs.json、receipts.json、status.json，绝不跟随包内artifact_files中的路径。
七项必须齐全；包内摘要集合恰为前六项。未知额外文件忽略且不读取，不代表整个目录的所有附件都已经审查。
Linux目录逐级O_DIRECTORY/O_NOFOLLOW，拒绝根及祖先符号链接、`..`组件；固定文件NOFOLLOW/NONBLOCK，fstat只允许普通文件，拒绝FIFO/设备/目录。
每项≤2MiB、总计≤8MiB，最多256事件、32输入/尝试/receipt、原历史每类64；JSON拒绝重复键、非有限数、超限整数/字符串/节点/深度/集合。
两次读取比较完整字节与dev/ino/size/mtime/ctime；正常关闭所有fd。

**只支持已关闭且静止的工件包。两次读取不是原子多文件快照、OS锁、敌对文件系统沙箱或断电耐久保证。**
有权限的攻击者仍可能共同伪造一致数据；文件系统观察与外部pins也不能认证真实传感器。没有实时耗时/可用性保证。

## 交叉核对

- 原始begin与scope/spec/status的scope/session/主机域/boot、相机集合、clock元数据、起止与累计计数基线一致；不拿当前主机boot或现在时间给旧记录重签。
- 七项外部pins与六项内部摘要；规范化snapshot pin；严格JSON类型（True不是1）。
- 原64环计数/后缀record ID/丢弃与基线；原曝光字段恒空，原授权恒false。
- 原输入唯一身份、有序完整分路/合成摘要、注册时间、尝试引用和预计ID；可选独立公共像素摘要逐项一致。
- 原事件顺序：挂载前记录→挂载→注册→交付/请求→返回→drain卸载；请求时间不晚于原selection开始；每个当前scope的有效callback能绑定到更早注册与交付。
- 重新调用**纯数据**adapt，逐项比较已保存receipt全部内容或原拒绝理由；不重新执行采集/选择/计算，也不输出新时间或控制输入。
- 已返回尝试与receipt一一对应，失败尝试保留；计数/错误列表决定原status优先级，不能只改status标签洗成成功。

原坏包缺少时间/摘要时只能保持带原IO错误的失败证据，不补造关联。
原日志故障允许核对已存在的事件前缀，但`classification=recorded_incomplete`、complete_evidence/evidence_consistent均false，保留原body_failed/cleanup_failed；不恢复遗漏事件。
缺少必需文件仍为invalid_bundle，不自动创建“故障sidecar”。

## 结果含义与不保障反例

- `recorded_failure`：文件/诊断关系一致，但原body/worker/cleanup/operation失败不改变。
- `recorded_rejection`：旧缓存或历史丢弃等原诊断拒绝保留；原环丢弃不是被修复的完整采集历史。
- `receipt_only` / `observation_only`：仅局部离线诊断，不能当作安全或完成验收。
- `recorded_incomplete`：明确原日志/清理故障、证据不完整。
- `invalid_bundle`：损坏/缺失/歧义/摘要不符等，不修复；recorded_status仅作为读到的原声明保留。

`complete_evidence`只指本格式所需工件与已声明诊断的完整一致，不证明无历史丢弃、真实曝光完整、进程已经实际停止或发生过真实物理动作。
资源关闭字段是过去工件的声明；本审查器不能通过保存文件重新观测过去线程。
所有结果actual_exposure_wall_s恒null，provenance/shadow/live/motion四标志恒false。

保留三个反例：重复同一包、共同改session与自报摘要、共同抹去只在status内记录的body错误，都可能通过**无外部pins**的局部一致性审查。
独立原七项pins会拒绝后两种改写，但若pins也被共同替换仍无来源认证或防重放。不能宣称“所有伪造都能识别”。

## 本轮验证

读取前轮23个已封存scope而非重建fixture，保留20个文件诊断完整及3个原日志故障不完整分类。
40类损坏/边界/反例fixture：37类拒绝、3个上述不保障反例保留；另对3个公共case各4帧独立PNG摘要核对。
单元测试另检查符号链接/祖先链接、FIFO不阻塞、路径越界不读取、两读变化、JSON边界、status外部pin、fd关闭、无时钟/线程与CLI退出码。
正式审查一次、无重试/裁剪；与普通未过滤全量测试串行。

真实曝光提供者、来源认证/防重放、原生接收token绑定、Gazebo计算前期限、0.1s预算及物理停稳均不在本轮解决。
