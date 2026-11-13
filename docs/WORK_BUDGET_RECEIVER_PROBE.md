# 2L：候选工作预算传递到既有短 expires 的隔离验证

新增显式 opt-in `/home/sunrise/toponav_project/tools/work_budget_receiver_probe.py`；只调用原纯软件 `LeaseGuard` 和直接包含原 `receiver_lease.hh` 的独立 stdlib C++ 程序。不编译 Gazebo 插件，不创建 Gazebo IO、原生 transport/world/publisher/actuator/step，不接设备；全部 command 是 observing 的 `(0,0)`，不是运动验收。

## 范围

2K 的 `compare_schedule` 原样复用。候选 D=完整帧可用时刻+0.8s；这是可用后工作预算，不是曝光新鲜度。传入原两端的 expires=D、issued=虚构完成时刻。成功接收保留原绝对 expires，不按接收时刻续租。issued+.8 仍是既有上限；原0.8s/0.4s、生产协议、控制器、权限、速度/制动/恢复完全不变。

虚构时刻是同一 synthetic_monotonic_ns 域中0至1000秒的整数，送原接口时转为 double 秒。模拟戳只比身份；不与主机时刻相减。边界案例只证明这些给定浮点数的比较，不证明纳秒调度或真实钟分辨率。辅助程序闭合输入语法且不接受速度字段，v=w恒零。工具不读当前钟、不sleep、无socket/线程/独立poll；操作按给定列表同步执行。编译器/system headers并非密封工具链，仅记录环境和产物摘要。

## 20个固定案例，两端均保存完整轨迹

及时/延迟接收、首慢、精确边界、前后1ns、旧hold先到/边界、及时新帧替换、仅延expires拒绝、联合重签及迟到联合重签反例、到期后新帧不能重臂、owner首次发送前停顿/发送后停顿/无poll、同帧更新/同包重放、慢capture、及时结果但交付过期。

所有包直接送原纯guard，**刻意不由候选先过滤**：迟到拒绝及重签反例是后端观测，不是新生产issuer。抽象comparison与原guard轨迹分列，不能把within_window当成原生授权。两个原端拒绝reason可能不同，原文保存，不伪造一致。

- 短expires原样传递时，接收延迟不延长期限；now==expires拒绝/锁存；过期后不能新帧恢复。
- 旧hold先到则先锁存并保留旧accepted元数据；及时新帧可替换旧hold，不是永久取第一次期限的min。
- 只延expires超issued+.8会拒绝；**同时把issued和expires按完成/当前时刻重签可能被两端接受**，包括已经超原D的结果。原端不知道frame_available，缺可信预算绑定。此轮把它留作明确未修复反例，不声称“所有传递方式不可延长”。UUID/本地摘要/会话一致不构成认证或跨运行防重放。
- owner首次发送前停顿：接收端seq0、deadline空，不会因候选D自动锁存。发送后给定Poll(100s)可观察sticky零状态，但不是按时独立执行或物理停车；根本没poll时，只能保留上次状态，不能把声明review当作已执行检查。
- capture开始至完整帧可用的等待不由新预算覆盖。真实曝光恒null，exposure_age_bounded、来源认证、shadow/live/motion权限、独立执行/物理停车/生产修复恒false。

## 复现

先固定`SOURCE_FILES`列出的7个源文件SHA256，保存JSON及该JSON摘要，然后显式执行：

```sh
cd /home/sunrise/toponav_project
.venv/bin/python -m tools.work_budget_receiver_probe \
  --output reports/<全新目录> \
  --source-pins /绝对路径/SOURCE_PINS.json \
  --source-pins-sha256 <该JSON文件SHA256>
```

输出只新建、不覆盖。正式审查一次、无自动重试；不匹配也保留每条轨迹和失败。固定源/输入/二进制前后摘要只用于本地静止字节核对，不是来源认证或敌对OS/原子快照/崩溃耐久保证。

仍未接入实际Gazebo计算前pending或可信下游绑定；不重复宣称旧2B已有的曝光假设影子owner。0.1s、旧IPC晚ACK/publisher SIGSTOP、物理停车、S2/S6、近场窄障碍和普通直走验收仍未解决。
