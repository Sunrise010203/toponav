# 普通直走任务2：零速隔离时效/截止时间诊断

入口：`/home/sunrise/toponav_project/tools/straight_deadline_audit.py`，仅手动启用。
生产控制器、门控、租约、限速、制动和恢复参数均不改；生产入口不导入此工具。

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m tools.straight_deadline_audit --output /tmp/straight-deadline-new --repeats 3
```

## 实际覆盖什么

- 原 `ActuatorClient.request/_receive/_validate_ack`、Unix SOCK_SEQPACKET、独立进程中的原
  `serve/LeaseGuard/MotorGate`、原独立日志写进程和原0.8s期限。
- 只在构造隔离客户端期间，将其启动目标改为工具内的私有工作进程，随即恢复 `Popen`。
  工作进程中日志写进程仍由原生产入口启动，不继承启动重定向。
- `isolated_receiver_timing.cc`编译原生产 `ReceiverLease` 头文件，并在另一个进程中轮询。
  工作进程通过私有socket发送租约元数据；两端都拒绝非零速度。没有外部话题、原生Gazebo发布者、世界或硬件。
- 接收端使用**诊断专用编码和5ms测试轮询循环**，没有覆盖生产 `DecodeEnvelope`、Gazebo
  `PreUpdate`/最终车轮写入和传输集成，不能将此轮询时延推广成生产或实车保证。

## 四种预先声明的故障场景

1. `zero_quick`：注入20ms等待后签发零速，发送端之后不轮询/续租，观察两端按原期限锁存。
2. `zero_slow_first`：首次签发前注入1.1s等待，冻结模拟采集时间，再签发新期限零速。
   原执行链可接受这条零速，暴露“签发年龄不是墙钟采集年龄”的边界，**不是安全通过**。
3. `zero_blocked_producer`：零速被接受后发送端阻塞1.1s，不发送心跳；独立执行/接收端应到期锁存。
   阻塞结束另发新序号、新模拟帧、新期限零速，不能清除原锁存。
4. `zero_stopped_publisher`：接受后确认自有执行进程进入SIGSTOP，跨过原期限，再恢复回收。
   独立C++接收端应在暂停区间锁存。执行端自身暂停期间不能轮询，恢复后的零发布不能算及时。
   暂停也会冻结执行端对日志ACK的消费；原0.4s日志ACK守卫可能报`flush_ack_timeout`、拒绝后续ACK，
   工作进程日志可能不完整。这些失败、旁车和原始发布记录全部保留，不放宽期限或补齐日志。

每例三次，正序/反序/正序，串行、不裁剪、不重跑正式轮。0.15s仅用于评估所观察到的归零延迟，
不传给任何门控、不扩展租约，也不是硬实时承诺。`expected_observations_met`仅表示声明的诊断现象出现，
不表示全链健康、捕获新鲜、安全停车或导航验收通过。日志故障须单独看`audit-summary.json`。

## 时间戳与不可观测量

同一主机上的Python `time.monotonic`和C++ `clock_gettime(CLOCK_MONOTONIC)`属于同一时钟域。
可比较：注入等待起止、原issued/expires、客户端send/wait/ACK验证、执行端socket.recv返回、
私有发布调用起止、ACK中的发布后时间戳、C++应用层接收及锁存。

- `receives.jsonl`记录应用层recv**返回**，不是内核收包时刻，也不是解码/校验完成。
- `publications.jsonl`是私有发布回调，不是电机接收或轮速测量；ACK只绑定该发布回调完成。
  原ACK不等待独立接收端确认，不能假定“ACK先/后”能证明车轮执行顺序。
- `receiver.jsonl`只记录原C++权限状态；锁存日志时间在Poll之后，未测量物理减速。
- `captured_sim_s`为明确合成的模拟时刻，**不可减去monotonic**。
  `diagnostic_frame_available_s`只是本轮诊断素材就绪时间，不是相机曝光/真实采集时间。
- `compute_*`是声明的故障注入等待，**不是实际控制器耗时**。
  实际控制器耗时仍以任务1的冻结普通测量为准；公开RGB没有历史墙钟采集时刻，不能离线补造。

因此真实曝光→计算→电机接收→物理停稳全链延迟仍未测得，`capture_to_motor_latency_s=null`。
全部输出、子进程退出后才读取/封存；失败输出不覆盖。

## 生产代码审查发现的剩余缺口

`simulation/gazebo_cross_validation.py`当前在`controller.update`返回后才`gate.arm`，然后签发issued；
相机缓存保留模拟采集戳而没有曝光墙钟戳。原0.25s模拟图像年龄校验与0.8s签发租约仍有效，
但同步仿真在计算时暂停，模拟时钟不前进，不能据此证明墙钟采集新鲜。
独立执行端可约束**已经签发**的旧命令；这不等于为首次慢计算或无已持有命令的阶段建立了采集期限。

后续需要单独设计/冻结：真实采集或明确标注接收时刻的单调时钟链、计算前建立的独立期限、
缺失/未知采集年龄时拒绝运动、以及计算结果不得重新签发延长已耗尽的采集预算。
接收时间不能冒充曝光时间。必须先在零速/隔离路径做反例测试，再评估是否允许生产接入；本工具不修补这些权限。

本轮不解释旧8193零速IPC测试的偶发超时根因，不证明其已修复。
S2未通过、S6未完成、0.1s计算预算未达、普通直走完成能力和近场/窄障碍覆盖均不能因本诊断升级。
