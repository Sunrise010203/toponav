# 2Y：大父清单与最小引用目录分层契约 v1

独立 opt-in 纯字节入口
`tools.pending_archive_projection.review_projection(parent_raw, projection_raw, claim, raw, observation_raw)`，
profile `toponav_pending_archive_projection_v1`。没有文件读取适配器、不调用2X、2V或旧协议校验器。
只在分层核对成功后调用**原封不动的2W纯引用契约**，传入的不是大父清单，而是两个成员的最小目录。

## 为什么分层，而不是修改旧上限

2W总交付清单有12266项，超过2W既定12000项；2X真实读取成功后仍应返回`manifest_shape`。
本轮保留该封存拒绝及2X首轮1064项开发失败，不修改2W/2X封存代码、错误或schema。
新增的**父清单profile**界限为8MiB、1–24000项，只属于2Y；并非让2W接受12266项。
父清单仍是闭合`{scope,files}`，scope最多4096字符，所有成员（含未选中的成员）均须满足
512字符ASCII相对路径和64字符小写SHA256形状。严格UTF-8，无BOM/重复成员/非有限常量。

最小目录原字节最多8192字节，也为闭合`{scope,files}`：scope必须为机器规格的固定非认证说明，
files**恰好只有claim声明的raw_ref和observation_ref**，不允许额外成员、丢失成员或改名。
它是派生文件，**不声称它本身已属于先前封存的父清单**。目录中的父子关系由本次claim明确声明，
不会在2W单独检查时自动变成来源证明。

## 契约与顺序

claim闭合8字段：`profile,parent_manifest_ref,parent_manifest_sha256,projection_manifest_ref,
projection_sha256,raw_ref,observation_ref,pairing_basis`。
pairing_basis只能是caller_declared_only；四个相对引用互异，词法规则不变；两个pin为小写SHA256。
parent/projection/raw/observation均须为精确bytes，界限分别8MiB/8192/36893/65536，先界限后哈希/解析。
引用名字只保存声明，不打开文件、不检查真实路径/文件存在，不认证root或设备。

依次检查：输入类型/界限、claim、父清单原字节pin/形状、派生目录原字节pin/形状、
两个选中引用在父清单中的存在及父子摘要相符，最后调用2W核对实际raw/observation字节。
未执行的比较为null，冲突为false；跨层摘要相符不代表最后的实际blob也相符。
若2W拒绝，顶层`projection_reference_refused`，嵌套`reference_result`保留旧错误原文，绝不截断/改写。
只有全部相符才为`projection_reference_checked`。

父/派生pin都固定**原编码**而非重编码；空白变化不重pin拒绝，重pin可相符。
observation仍不解析，空blob/非JSON可在调用者自建清单中相符。
返回claim副本，无状态/时钟/消费动作，重复结果完整一致。

## 不能推出的结论

父pin、派生pin、成员选择、raw/observation配对全由调用者提供，不是可信根或签名。
整组替换并重pin、不相关的封存文件配对、原拒绝声明、重复调用仍可相符。
所有旧20个权限/来源/执行标志与新的4个原子性/父来源/父子认证/文件读取标志仍false；
曝光与旧实际执行位置null。不新增运行时准入、owner/permit/token、一次性消费或防重放。
2X的硬链接/瞬时祖先替换恢复可通过仍保留；本轮纯字节检查没有消除顺序采样局限。

## 封闭验证与保留项

以真实12266项父清单和2U原532个raw/1064个Python/C++来源快照生成各自两成员目录，保存
父/派生目录原编码、原blob、claim、完整结果及重复结果。另验证较小的2X真实总清单、
整组替换/不相关配对、错误pin/缺成员/跨层冲突、未选中非法条目、严格JSON、尺寸与成员数边界。
origin=cpp只是原归档来源，不是本轮C++/真实运行时后端执行。
独立分析只核对封闭字节/摘要/选择映射，不执行新契约或旧运行时。

16旧位置未知、66错误标签差异、旧四包装器差异、拒绝/整组替换/重复调用/unknown迟到不升级，
2L原两端晚重签不变且不重跑/重签，不冒充2B能力。
2M14需求仍未实现、25运行时场景未执行；不接生产native/Gazebo/设备/真实运动，
不改0.8s/0.4s/门控/权限/速度/制动/恢复，无物理停车证明，普通直走验收仍未完成。
