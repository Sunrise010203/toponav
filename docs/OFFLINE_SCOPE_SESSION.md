# 离线作用域原始侧录与失败生命周期

`tools/offline_scope_session.py` 是显式 opt-in 的离线 fixture，不是原生采集提供者、watchdog 或运动门控。
仅通过原隔离工具的 `GazeboIO.__new__` 创建 Condition/CameraBuffer，调用原 image/pixels/cache/next_frame，配合原只读 CaptureTimingProbe、Echo 和纯 receipt-only 适配器。
不调用 Gazebo 原生构造器，不启动 world/transport/发布者/执行器/step helper，不导入生产入口；原生产逻辑与预算不变。

## 原始记录，不事后恢复

owner 创建时读取当前 Linux boot_id 与 `clock_gettime(CLOCK_MONOTONIC)` 元数据，生成本地 session UUID。
每次 scope 在挂载探针前独占创建目录，写 begin.json：scope UUID、完整逻辑相机、session/boot/主机时钟、进入时刻、原累计计数基线。
这些本地身份和文件可被修改，**不是传感器认证凭证或安全沙箱**。不支持替换私有句柄、外部直接回调、跨线程控制 scope 或恶意运行时修改。
仅接受 isolated_synthetic_delivery/replay_delivery；无 live 模式，也不接受外部原生 IO。

prepare 在交付前复制 RGB、设为只读，独立记录完整视图与合成摘要、frame/模拟戳。
只读 NumPy 标志防止意外写入，不是不可篡改内存保证。模拟戳仅核对身份，不与 monotonic 相减。
关闭侧录 scope 不是曝光区间；UUID/boot 和摘要只支持离线诊断一致性，不防重放或共同伪造。
输出曝光、设备时钟映射恒空；provenance/shadow/live/motion 授权恒 false。

## 异常与关闭

- 只允许顺序作用域；嵌套、外线程控制、跨 scope packet、关闭后操作与已有目标目录均拒绝，不覆盖旧证据。
- 每路实际交付仍经过原 image/pixels/cache/selector；Echo 使用原选择图像及缓存辅助路，不注入导航决策。
- compute 在原调用前记录预计 selection/computation ID 与输入；失败尝试也保留。
  fixture 预交付后使用 next_frame(timeout=0)，不是修改生产期限或超时。
- 后台线程是本 scope 拥有的非 daemon 线程，最多32；关闭先进入 draining，join **全部拥有回调后才卸载**探针并核对原实例属性身份恢复。
  join **无界**，回调卡死会令退出永久阻塞；不声称实时关闭或物理安全。
- body 异常原对象不被日志/清理错误覆盖；无 body 异常时先重抛原 worker 错误，其次清理错误。
  status 同时保留 body、worker、cleanup、event、原 IO 错误与失败尝试。
- event 流发生错误后拒绝新工作；清理仍执行。清理/日志失败 taint owner，后续 scope 拒绝；原 IO error 也绝不清除。
- detach 后记录关闭边界、关闭 event 流，然后保存 snapshot/scope/inputs/receipts/status；静止快照才适配。
  status 的 artifact_files 固定六项，不含其自身；artifact_files_complete 只表示这些文件完整，不代表场景成功或安全通过。
  报告最后的外部 delivery manifest 才覆盖 status 文件本身。
- 任一历史丢弃、未观察缓存、旧引用或适配歧义只导致离线诊断拒绝，不是生产拒绝门控。
  probe 重挂保留累计计数和历史，按独立基线识别新 scope，不清空失败证据。
- 最多32输入/计算/拥有线程及256事件；上限显式报错，不静默淘汰。原探针64环历史独立有界，丢弃计数照实保留。

scope 控制方法仅支持同一线程调用；不提供异步上下文或外线程直接退出语义。
日志为普通关闭文件，无 fsync/事务/崩溃耐久保证。SIGKILL、断电、OS/存储故障、join 中断或原卸载/取快照故障可能留下不完整工件；不得声称所有退出必有完整侧录。
外层保留 primary 并 taint 清理失败，但不能提供操作系统级资源回收保证。

## 验证边界

`tools/offline_scope_cases.py` 记录20种隔离生命周期场景，含可控 callback 未退时退出、body/compute/worker异常、缺路、捕获后继续退出、旧缓存、重挂、64环丢弃、日志write/close失败、嵌套、目录已存在、空scope、关闭后操作、坏包。
故障是原异常与原缓存/IO行为的受控注入，不模拟物理传感器或电机失效。
三个公共case各前4 RGB只重放交付并由 Echo 检查像素；原 decisions 文件仅提取 input，不把保存决策作为控制输入，不跑导航性能或完成验收。
闭合工件分析只读文件，不重新构造会话/回调。测试和正式审查串行，正式只跑一次，无重试或失败裁剪。

下一步宜完善独立只读证据审查与损坏/遗漏工件诊断；真实全像素曝光、来源认证、防重放、原生token绑定、计算前期限和物理停稳仍无本轮实现。
