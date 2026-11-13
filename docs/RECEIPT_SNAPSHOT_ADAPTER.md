# 任务2E：离线receipt-only快照适配器（没有真实曝光提供者）

入口：`/home/sunrise/toponav_project/tools/receipt_snapshot_adapter.py`。
只依赖标准库和纯数据契约，读取调用方提供的静止快照；不读系统时钟/设备/文件/网络，
不安装回调、创建线程/发布者/执行器、签token或连接门控，生产入口不导入它。
离线fixture/报告驱动的文件读取和隔离真实方法调用在另一个显式工具中，不是适配器的隐式运行行为。

## 独立输入与作用域注记

调用方分别持有：

- ProviderSpec：冻结完整四视图及可选up/down、逻辑sensor、config/provider/session及主机域/boot注记。
  本适配器只接受**没有设备时钟/映射配置**的receipt-only配置。
- InputReference：独立frame_id、原模拟戳ns、完整每路原RGB摘要、控制器合成图摘要和指定selection/computation记录ID。
  不能从快照的摘要回填这些输入再称“独立核对”。模拟戳只作身份比较，缺失时拒绝，不推导墙钟曝光。
- ScopeAnnotation：作用域ID、上述上下文、source_kind、作用域起止、进入前四类记录计数及规范快照SHA256。

新fixture在安装前记录边界与计数，卸载后取关闭边界/快照。
旧任务2C快照**没有原始session/scope/boot认证字段**。导入旧数据只能显式标为`retrospective_offline`，
用公开输入和已冻结创建脚本核对；时间范围是事后包围区间，session/scope是本轮离线命名，
不是恢复出来的原始采集会话/安装边界。结果保留这个标签，不能晋升成独立来源证明。
`independently_recorded_fixture`也只是独立记录的隔离fixture边界，不是受信任驱动凭证。

快照SHA是JSON `sort_keys=True,separators=(',',':'),allow_nan=False`的摘要，
不是原文件字节摘要（原文件另由冻结清单绑定）。序列/节点/深度/字符串/数值/总字节均有限，拒绝任意对象序列化。
这里只接收已解析对象，不是宽松JSON/重复键解析器；安装/保存层仍应独立负责原始格式审查。
调用期间输入须静止，适配器不提供并发读写隔离或防恶意调用方的内存安全边界。

## 核对链

1. 只接受`replay_delivery`和`isolated_synthetic_delivery`，显式拒绝live来源；检查scope与独立配置一致及快照pin。
2. 必需相机集合完整；observer_errors、饱和计数、**任一历史丢弃**均拒绝。
   有限历史只在全部保留时做本次一致性核对，不能把保留的子集冒充完整审计。
3. 指定selection/computation记录必须在本scope进入计数之后；核对frame、模拟戳、合成图与全部辅助图摘要。
4. 核对各相机缓存/选择摘要，再用view、原模拟戳、RGB和五个阶段样本精确关联唯一callback。
   缺失/歧义、未观察缓存、同戳换图、缺视图、原异常或交付错误均拒绝，不重建缺失信息。
5. 验证回调入口→锁→decode→缓存→释放/完成、同步集合→选择→next_frame结束→update起止的局部偏序及scope边界。
   callback锁释放后可与selector并发，**不要求callback完成必早于selector**；这些不是独立硬实时上界。
6. 至少存在本scope内所有必需视图已观察、在最晚缓存后/选择前的同戳同步记录。

“拒绝”仅指拒绝生成离线诊断适配结果；不会影响原控制器结果、原缓存、原门控或任何权限。

## 秒到ns只用于交付观察，绝不补曝光

原Python monotonic样本为int/float秒。用`as_integer_ratio`对**该二进制样本**精确乘10^9后向下取整，
避免额外浮点乘法取整误差。不是原时钟ns精度恢复，更不是物理曝光上下界或系统时钟误差界。
契约的审查now使用**记录的scope关闭样本**，字段名为`review_at_recorded_close_ns`，
不采样当前时间、不给旧帧重签“现在”，不计算图像新鲜度或安全年龄。

输出每路都为`receipt_only`，exposure ticks/scope/mapping恒null；所有曝光包络及全组oldest lower仍null。
适配器及ContractReview的provenance/shadow/live/motion授权四标志恒false，无任务2B门控/token桥接。

## 仍不能证明的事

冻结快照和独立输入只证明这些**已持有数据**一致，不能证明传感器曝光、时钟来源或摘要到硬件的不可替换绑定。
若同时伪造Scope/Spec/InputReference/快照，或把旧内容换一个共同一致的新session标签，本模块无法认证。
无状态地重复同一快照仍可再次适配；测试保留共同重标上下文和重复快照两个反例，均不提供授权。
来源认证、防重放owner、device时钟提供者和原生接收绑定仍未实现。

## 隔离验证范围

`/home/sunrise/toponav_project/tools/receipt_snapshot_cases.py`：

- 新建最小`GazeboIO.__new__` fixture，调用原image/pixels/cache/next_frame与Echo.update；
  不执行原生构造器、建立世界/节点、执行step或运动。
- 24类固定正例/突变反例，其中重复适配调用两次；四路/六路使用独立原数组摘要和实际回调快照。
- 导入任务2C六份已保存instrumented快照，各12条，共72条；
  独立从原公共RGB重新计算合成/分路摘要，原decision只用于定位其公共input，不送入任何控制器。
  **没有新controller.update、历史曝光重建或性能复测**。

正式前冻结所有源、六份trace及全部读取的公共输入；串行一次审查与普通全量测试，不重跑删样本。
这不是新场景/完整序列/终态/导航或物理停车验收。
真实live连接仍需独立计划和明确授权，原0.8s/0.4s、限速、制动/恢复预算与ROS world已有pending均不改变。
旧IPC偶发超时、Gazebo计算前期限缺口、0.1s目标、S2/S6及近场/窄障碍/普通直走完成仍未解决。
