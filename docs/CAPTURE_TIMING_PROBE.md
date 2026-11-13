# 任务2C：只读采集链阶段观测（未接入生产）

`/home/sunrise/toponav_project/tools/capture_timing_probe.py`只提供显式、实例级、作用域内的观测包装。
生产入口不导入它，原源文件不变，没有新增门控/相机参数/权限/超时。诊断会增加耗时，不是零开销透明层。

## 实际观测边界

|字段|实际含义|不是|
|---|---|---|
|`callback_entry_s`|Python `GazeboIO.image`包装入口，在获取IO锁之前|传感器曝光、内核收包、原生transport开始分发|
|`lock_acquired_s`|原Condition已成功进入后采样|内核调度上界|
|`decode_started_s` / `decoded_s`|原`pixels`调用前/返回后，包括原校验、NumPy复制|传感器解码硬件计时|
|`cached_observed_s`|原`CameraBuffer.put`返回后采样|曝光时刻|
|`synchronized_set_observed_s`|put/摘要观测后发现所有必需视图已含同一原模拟戳|最早真实ready时刻，也不保证模拟时钟已允许选取|
|`selected_observed_s`|原`latest`返回并完成额外像素核对后|原`latest`内部零开销完成时刻|
|`next_frame_started_s` / `next_frame_finished_s`|原`next_frame`调用的作用域边界|曝光到取帧的延迟|
|`compute_started_s` / `compute_finished_s`|原`controller.update`调用前后|本地资料准备、PNG加载、网络、ACK或电机停车|

同一主机Python monotonic域可比较这些本轮阶段。原`captured_sim_ns/s`只作身份核对，绝不减去monotonic。
所有`actual_exposure_wall_s=null`、`exposure_clock_mapping=null`、`clock_mapping_verified=false`。
未知上游延迟时，回调收到图像的时间不能提供证明新鲜所需的曝光下界。

`Condition.wait`仍调用原方法，释放/重获同一把锁，参数与返回值不变。
保留等待次数、首个等待起点和最后一对等待/重获记录，不保存无限等待列表；不能把最后一对当完整等待分解。

## 像素与相机集合绑定

- 作用域进入时冻结原四视图加可选up/down配置，原相机集合不改，未知变化使诊断无效而不改变原控制结果。
- 在原put之后，对已复制的原数组计算SHA256；缓存元数据只覆盖原每视图8槽，最多48槽，不持有RGB数组。
- 原latest选择后重新核对各视图对象及当前字节摘要，记录合成图摘要；原数组/元组返回值原样返回。
- update之前核对实际输入合成图、模拟戳与所需up/down实际输入摘要；缺失或换图不能被同戳冒充匹配。
  frame_id只记录，不宣称这种摘要比较能认证会话/帧ID或替代原协议验证。
- 已在缓存但未经过本作用域观测的帧保持未绑定，不重建收到时间。重入作用域清空缓存元数据。
- 不往控制器frame/decision增加字段，不读取场景真值、位姿、接触或结果审计。

## 隔离入口与验证范围

`/home/sunrise/toponav_project/tools/capture_timing_isolation.py`通过`GazeboIO.__new__`构造最小fixture，
直接调用原image/pixels/cache/next_frame方法，**不执行原生构造器**，不建世界/节点/发布者/执行器/step helper。
将原保存的256×256公共RGB拆成原四路128×128 packed RGB消息，原CameraBuffer重新拼接，逐字节核对再交给原控制器。
这测的是**本轮离线交付**，不是历史采集或真实Gazebo transport。六视图合成例另核对up/down，不冒充真实路由采集。

六个隔离用例：四路、六路、缺路后补齐、IO锁竞争、selector等待后收到图像、原始坏包拒绝。
额外单测覆盖像素原地修改、未观测缓存、同戳换图、缺失辅助视图、异常/原实例覆盖恢复、
诊断时钟故障不改变原方法调用、固定历史丢弃计数和JSON空曝光字段。

正式公共前缀为三例各前12条，涵盖observing/bootstrapping/following及offset的waiting_evidence；
先ordinary→instrumented，再instrumented→ordinary，串行，不重跑、不裁剪慢样本。
OpenCV本轮固定1线程、结束恢复原值；BLAS/GC/控制参数不调。每次输出对原保存decision及配对输出精确核对。
这些是暴露过的策略/公共输入；gradient/wide策略不晋升，不宣称新场景、完整序列或终态/导航完成验收。

历史环各64条，元数据计数饱和时明确标记下界；快照脱离内部对象。丢弃不是完整审计证明。
异常路径仍调用原方法并恢复原对象/实例属性；诊断错误只计数，不替原控制器修复、拒绝或授予权限。
**退出前必须等所有自有回调结束**，不支持运行中并发卸载包装。截图/JSON只在测量调用之后写，不在回调内写文件。

## 仍未解决

哈希、额外锁和字典操作均会扰动原时间；不以本轮耗时作为生产调度上界、优化收益或原预算通过证明。
没有真实曝光/设备时钟映射提供者，没有接入任务2B影子token，没有补好Gazebo计算前期限或原生接收绑定。
ROS world已有独立pending守卫，不能用本工具泛化替换。真实曝光→电机/物理停车全链、底盘固件保护仍须另行验证。
本轮不解释旧IPC偶发超时；S2/S6、0.1s计算目标、普通直走完成、近场及窄障碍覆盖均不因此升级。
