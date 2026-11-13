# 任务2D：真实时间来源的候选数据契约（没有真实提供者）

入口：`/home/sunrise/toponav_project/tools/time_provider_contract.py`。
仅标准库、不可变数据与纯函数；没有时钟读取、文件/网络/设备I/O、线程、门控、token适配或执行器。
它审查**声明之间的结构、区间算术与绑定一致性**，不是生产接入或来源认证。
全部结果恒有`provenance_authenticated=false`、`shadow_admission_authorized=false`、
`native_live_observation_authorized=false`、`motion_authority=false`。
即使所有数值完整、互相一致，也不能形成任务2B的可信曝光前提或运动许可。

## 1. 独立持有的配置和输入身份

未来独立owner须持有ProviderSpec：config/provider/session、主机单调时钟域和boot身份，
冻结四路front/left/right/back及原配置要求的up/down（最多六路），每路唯一逻辑sensor身份。
device时钟身份、device boot及**整个ClockMap规范JSON的SHA256**成组固定；
不能由计算结果删掉旧视图、换映射数字、改session或只提交最新子集。

ExpectedBinding来自实际输入的独立快照：frame_id、可空模拟戳、完整每路原RGB摘要。
BundleClaim逐项匹配这些身份与摘要；图像同模拟戳不能替代字节绑定。
工具只比较传入摘要，不自己读取RGB，更没有证明摘要与设备物理曝光不可替换绑定。
规范JSON是本地dataclass的固定序列化，不是已部署的线协议/签名格式；不提供宽松JSON解析器。

dataclass、字符串、摘要、合成boot/nonce都不是凭证。
若攻击方同时改Spec/ExpectedBinding/Claim或伪造互相一致的曝光声明，本契约无法发现真实性问题。
测试明确保留“共同伪造摘要仍通过一致性”的反例，所有授权标志仍为false。

## 2. 不允许把交付、模拟时钟当曝光

`receipt_only`只允许callback_host_ns；曝光ticks/scope/mapping必须为null。
它可以通过结构检查，但所有该路曝光区间保持null；只要一路未知，bundle的oldest lower仍为null，
不能取已知子集的最小值冒充整组新鲜度。
历史replay或Python回调入口只能作为交付观察；未知排队/网络延迟不能用receive减零或减自定常数补下界。
captured_sim_ns可空，只作帧身份匹配，从不减主机monotonic。

`device_exposure_ticks`要求明确的`all_pixel_exposure_envelope`：
最早像素开始曝光至最晚像素结束曝光的包络，不是首行/末行中心戳、帧编码时间或发布时间。
rolling shutter、曝光时长、帧拼合/辅助视图应如何取得完整包络，真实驱动器必须另给可审计依据。
这个scope字符串本身不证明驱动器确实做到。

## 3. 有界映射：整数ns与正有理数率区间

ClockMap声明：设备时钟/boot、目标主机单调时钟域/boot、anchor_tick，
anchor时刻主机区间`[reference_lower_ns, reference_upper_ns]`，
整个有效设备窗口内成立的正率区间`[rate_lower_num/rate_lower_den, rate_upper_num/rate_upper_den]`（ns/tick），
设备tick窗口及主机有效期`[valid_host_from_ns, valid_host_until_ns)`。
另保留校准依据内容摘要；工具不读取/认证这份依据，更不证明误差界满足真实设备的漂移/重置/同步不对称。
全部整数有界，拒绝bool、float、NaN、负值、零分母、反转区间、过期/未来/跨boot/domain、超窗与输出越界。

对设备曝光起止ticks：

```
lower = floor(reference_lower + min((start-anchor)*rate_lower, (start-anchor)*rate_upper))
upper = ceil (reference_upper + max((end-anchor)*rate_lower, (end-anchor)*rate_upper))
```

正率保证最小/最大曝光发生在两端；anchor之前差值为负时必须反转率端点选择。
使用Fraction及向外floor/ceil，避免浮点四舍五入缩小不确定性。
映射输出、callback及审查now须仍在有效主机窗口，`now == valid_until`即过期，无时间宽限。
曝光upper不得晚于callback，callback不得晚于审查now；主机域一致仍只是声明，不是硬件证据。
最老可能曝光取**所有必需视图lower**的最小值，不能取upper、最新一路或计算完成时间。
这不计算新安全年龄，不调用原0.8s租约，不修改原0.1s/0.25s/0.4s及任何限速/制动/恢复预算。

tick回卷/重置不能由本契约猜测修复，真实owner必须更新device boot、重新审查校准/配置并处理序列。
此函数无会话状态、重放注册表或计数器回退历史；重复相同bundle可再次通过结构审查，**不是防重放通过**。

## 4. 隔离审查与反例

`/home/sunrise/toponav_project/tools/time_provider_contract_cases.py`只构造合成声明。
20个预定场景：未知/完整四路/完整六路/混合未知、缺路、同戳换图、换会话、主机/设备重启、
交付戳冒充曝光、映射到期、同名映射改数字、模拟时钟、首行戳、曝光晚于交付、重复bundle、
共同伪造摘要、anchor之前的率区间、分数向外取整、输出溢出。
其中一致性通过不是“安全通过”；重复/共同伪造是明确的非认证反例。
合成校准与ticks不是真实相机、历史RGB曝光或生产调度证据；本轮没有性能/停车测量。

## 5. 零运动live观测也不得自动接入

清单见本轮报告的LIVE_OBSERVATION_ADMISSION.json，当前全部未满足/未授权。
未来单独接入计划至少须审查：

1. 独立确认真实驱动/原生transport符号、相机集合/路由及原始曝光字段语义，不能创造不存在的字段。
2. 审查校准证据、整个窗口内误差界、设备/主机boot和跨进程时钟一致性，来源/像素/session不可替换绑定。
3. 证明读取式接入不会构造执行器/发布者/step helper，不会用零心跳续租、解锁或使能非零命令。
4. 明确独立停止、资源/日志/异常与并发卸载边界；保留坏包、掉路、重启、回退/过期、日志/进程暂停反例。
5. 预先冻结输入/代码、原预算和一次性零运动测试计划，明确授权后才能连接原生源。
6. 接入计算前owner、token与原生接收器是**另一轮审查**；结构检查通过不能触发此动作。

本轮没有真实provider、原生live连接、生产门控修复或运动权限。
Gazebo计算前期限缺口与ROS world已有pending须分别处理；旧IPC偶发晚ACK根因未解决。
真实曝光到物理停车、0.1s预算、S2/S6、普通直走完成及近场/窄障碍覆盖均未改善为验收通过。
