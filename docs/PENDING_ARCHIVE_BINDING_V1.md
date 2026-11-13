# 2W：只读档案引用与输入绑定声明契约 v1

独立 opt-in profile `toponav_pending_archive_binding_v1`，入口
`tools.pending_archive_binding.review_archive_binding(manifest_raw, claim, raw, observation_raw)`。
这是**纯字节契约**，不是生产协议、文件读取器或2V诊断的替代实现，不改任何旧schema/错误。

## 三层声明，不混为一谈

1. `file_references_agree=true`：调用者给出的manifest原字节SHA256等于调用者给出的pin，
   两个相对引用均在该manifest中，两个所给blob摘要分别等于目录中的摘要。
2. `pairing_declared=true`：调用者声明这两个引用配对。目录成员关系不证明它们来自同一次执行。
3. 来源认证、设备采集、旧执行与输入绑定：**全部未建立**。即使前两项为true，
   `observation_input_binding_authenticated/archive_origin_authenticated/pair_execution_link_observed`
   仍false，`observed_validation_site`与曝光仍null。

manifest pin与manifest_ref本身也由调用者提供，不是可信根；`manifest_ref`只保存声明的名字，
没有外层目录来认证它。引用名检查只检查词法，不检查文件存在、真实路径、符号链接或磁盘内容。
检查器**不打开路径、不联网、不解析observation、不调用2V或旧协议校验器**。
所以`archive_reference_checked`既不是“已从文件安全读取”，也不是“快照合法/与raw语义匹配”。
后续如需文件适配器，必须另行评审有界只读、NOFOLLOW、特殊文件、路径替换和竞争条件；本轮未实现。

## 精确契约

claim闭合六字段：`profile/manifest_ref/manifest_sha256/raw_ref/observation_ref/pairing_basis`。
pairing_basis只能是`caller_declared_only`。三个引用互异，1–512个ASCII字母/数字或`._+/-`，
不得绝对路径、空分量、`.`、`..`、反斜杠或NUL。摘要只能是64个小写十六进制字符。
本轮目录形状为闭合`{scope,files}`；scope最多4096字符，files为1–12000个上述路径到摘要的映射。
只接收严格UTF-8 JSON，无BOM、重复成员、非有限常量；不修改任何旧目录解析规则。

三个输入必须是精确bytes；manifest最多4MiB，raw最多36893字节，observation最多65536字节。
字节界限先于哈希/JSON解析，claim形状先于哈希。JSON深度错误安全拒绝。
manifest按**原编码字节**哈希，不重编码后比较；空白变化不重pin会拒绝，重pin则可相符。
observation完全不解析，保留原编码身份；非JSON/空blob也能在调用者自建目录中引用相符，
这正是引用检查与语义校验不能混同的反例。

按字节类型/界限、claim、manifest摘要、manifest形状、raw引用/摘要、observation引用/摘要依序拒绝。
首错后未检查的摘要比较为null，不伪造为false或成功；失败时不颁发任何权限。
返回claim副本，调用者与输出后续修改互不影响；无缓存、消费状态或时钟，重复调用仍一致。

## 封闭语料与不变的缺口

对2U原532份raw及Python/C++原始快照文件分别建立引用，共1064个原归档配对。
新检查全部在Python中运行；origin=cpp仅指归档来源，不是本轮C++/后端执行。
补充错误pin/缺引用/单件篡改/原编码空白、目录与路径/类型/界限负例。
两个真实封存但不相关的文件可配对相符，整组目录/pin/两个blob替换仍可相符，
人工不透明空blob也可相符；这些不是旧validator故障、来源认证、单次消费或防重放。

16条同标签关系仍保留：引用可区分原raw，但旧位置未知；66个旧错误标签差异与旧四包装器差异
原样保留，未证明全域等价。拒绝声明、整组替换、重复调用、unknown/迟到不升级、
2L原两端晚重签仍保留且不重跑/重签；不将2B既有能力算成新增。
2M14需求仍未实现、25运行时场景仍未执行。
不新增runtime owner/permit/token/可信poll，不改生产native/Gazebo/设备/真实运动，
不改0.8s/0.4s/门控/速度/制动/恢复，无物理停车证明，普通直走验收未完成。
