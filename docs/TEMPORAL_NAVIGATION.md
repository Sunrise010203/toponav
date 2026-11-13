# 第五块：时序会话与有限恢复 v1

日期：2026-10-15。实现范围：已知任务起点/朝向、四向结构、原子转向与整通道执行。
不是连续控制、主动探索、全局重定位或实车安全验收。

## 2026-10-29 接口安全修订

活动会话遇到非法帧/反馈时先锁存`failed`并清空确认窗口，再抛`ValueError`；
时钟/定位后端异常同样先锁存失败，再向调用方传播，不允许恢复好包后沿用旧票继续发令。
`reached/failed`先于载荷解析和时钟读取返回stop，后续坏包不能改写终态。
帧处理和真实完成提交时要求墙钟不回退；回退/非法墙钟使会话失败，不能降低完成屏障。
完成反馈先验证转移及完成时钟，再提交completion、pending、候选、计数和新屏障；失败时不消费完成。
这些是协调器状态保证，不是执行器已停车的证明，调用方异常停车及finally仍不可省略。
复现、兼容性范围和保留失败见[第七批修复记录](/home/sunrise/toponav_project/docs/PROJECT_REVIEW_SESSION_EVENT_FIXES.md)。

## 1. 为什么需要独立会话层

旧离散入口遇到 stop 就结束。若仅删除 break，并把上一轮 succeeded 不断传给定位函数，同一完成反馈会被反复应用，造成虚构跨节点。

新增 `/home/sunrise/toponav_project/simulation/navigation_session.py` 维护事件消费与证据窗口，仍调用现有 `update_candidates`、`decide` 和公共路径规划。旧入口和核心函数保持不变。

## 2. 状态与安全门

| 状态 | 行为 |
|---|---|
| observing | 停车，等待新帧 |
| confirming | 在同一静止参考系确认结构，不执行缓存动作 |
| waiting_execution | 唯一运动命令在途；所有新观测都不能代替完成反馈 |
| recovering | 停车，重新观察，保留最后运动预测的历史候选 |
| reached / failed | 任务终止，不再发运动 |

每次收到新输入返回快照。**仅该次返回 `command_id != null` 才表示一条新运动命令**；重复调用、反馈或 tick 不重发动作。stop 是安全请求，不是“完成了上一条运动”。

协调器不接收模拟器、场景、真实位置、真实朝向或真实到达。执行器内部可读取物理真值，公开反馈不携带这些字段；事后审计不会反向修正决策。

一个会话由单一调度器串行处理事件，不是多线程共享对象；未来接 ROS 回调时须先排队再串行消费。当前驱动是同步最小实现，tick 没有后台线程；摄像头接口本身无限阻塞时无法抢占它。真实设备接入需要有界读取或独立 I/O 看门狗，不能把本版的输入事件超时当作已实现进程级硬实时急停。

## 3. 事件契约

```python
session.observe({
    "frame_id": "camera-unique-id",
    "captured_at_ns": actual_capture_time_ns,
    "observation": {
        "relative_open": {"front": True, "left": False, "right": False, "back": False},
        "confidence": 0.9,
    },
})
session.feedback({
    "event_id": "executor-unique-event",
    "command_id": issued_command_id,
    "feedback": {"status": "succeeded"},
})
session.tick()  # 即使没有相机帧，也须由调度器定期调用，以检查超时。
```

- 三个外层字段均严格校验；原 observation/feedback 字典不加字段。
- 每个会话默认生成 UUID 命名空间，命令 ID 不与前一个会话的 cmd-0001 混淆。仅确定性测试需要手动 session_id；生产者不得重用它。
- frame_id 全会话只计一次；采集时间严格递增。旧图不得重标为当前时刻。
- 同一 event_id 载荷变化、未知命令、矛盾完成结果会失败停止。
- 已完成命令的同状态重放（包括更换事件 ID）不会再次推进；晚到的旧 sent/unknown 也不会重开命令。
- 在途命令的 sent/unknown/缺反馈保留原位及可能完成候选，始终锁住运动；可信终态到来后从**命令发出时的候选**重算一次，而不是从 stay+move 再次推进。
- failed 只适用于“未移动且未转向”。半转弯、部分位移不属于本执行模型。
- 活动会话的非法接口输入先锁存failed并清空窗口，再抛ValueError；终态直接忽略载荷。
  外部接入者仍必须捕获异常后停车。所附驱动在finally中请求stop，并关闭传感器。

## 4. 多帧与时间策略

默认连续 3 个新鲜、完整、置信度至少 0.6、结构完全一致的帧构成全票窗口。结构变化重启窗口，不混合不同朝向或转移阶段。窗口分数取最小值，只用于现有可靠性门槛，不代表统计独立或校准置信度。

每次发运动命令、收到终态反馈后清空窗口。完成反馈后只接受**严格晚于完成处理屏障**采集的帧；处于在途期间的图像不会缓存成到达证据。

帧龄最多 1 秒；过期、未来、乱序、重复、旧参考系帧停止且不加票。Unix纳秒用于同机器帧龄，单调时钟用于持续时间；tick仍仅凭单调时间检查超时，不依赖墙钟。帧处理或新完成提交读到墙钟回退时失败并抛异常，不能将回拨后的较小时间设为完成屏障。单调时钟本身回退则失败停止；非法单调时间或时钟提供者异常先锁存失败再传播。时钟值/配置的有限性检查拒绝会导致数值转换溢出的巨大整数，不重打采集时间。

默认参数由 SessionConfig 固定：每阶段 5 秒、全任务 120 秒、最多 32 条运动命令、1000 个输入事件、2 次已确认结构冲突、1 次已确认原地失败后的重试。坏帧、重复反馈及 sent/unknown 不刷新阶段超时。调用 tick 可以保持部分有效窗口，但不能增加票数；完成屏障始终重置它。

## 5. 能恢复什么、不能恢复什么

可以：暂时低分/缺视图/旧帧/丢帧后的新观测确认；单帧错误；保留运动预测后重新观察纠正暂时结构冲突；确认原地失败后的有界重试；晚到可信完成反馈后的再确认。

不能：靠观测猜测未确认执行已经成功；对称候选自动选一个；清空候选后读真值或全图跳转；部分位移；主动回退或不确定位置下试探运动。

连续一致的相关错误仍可能被接受，错误先验在对称结构上仍可能导致误到达；单向审计会暴露，而非修复它。模型 v1 的不可观测错误接受 8/24、遮挡组 5/5 是已有失败，**本轮没有重训或证明消除这些失败**。

## 6. 运行

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1 OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
export MPLBACKEND=Agg MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib

# 理想结构，会等待确认；与旧入口独立
.venv/bin/python -m simulation.temporal_navigation
.venv/bin/python -m simulation.temporal_navigation --scenario mismatch_once
.venv/bin/python -m simulation.temporal_navigation --scenario late_success
.venv/bin/python -m simulation.temporal_navigation --disable C06

# 实际 PyBullet RGB -> 已封存 Predictor -> 同一会话，无理想传感器回退
.venv/bin/python -m roadvision.temporal_navigation \
  --run /home/sunrise/toponav_project/models/road_structure_frozen_v1_20261014 \
  --visual-fault missing_view \
  --output /home/sunrise/toponav_project/reports/temporal-missing-view-NEW.json

# 与旧版相同的暴露测试集 60 项回归；输出文件必须不存在
.venv/bin/python -m roadvision.temporal_navigation \
  --run /home/sunrise/toponav_project/models/road_structure_frozen_v1_20261014 \
  --suite --output /home/sunrise/toponav_project/reports/temporal-suite-NEW.json

# 19 项预定义单因素图像接入故障，目录必须不存在
.venv/bin/python /home/sunrise/toponav_project/tools/verify_temporal_acceptance.py \
  --output /home/sunrise/toponav_project/reports/temporal-faults-NEW

.venv/bin/python -m pytest /home/sunrise/toponav_project/tests -q -p no:cacheprovider
```

导航 CLI：退出 0 仅表示真实到达审计通过，1 表示安全终止/审计不通过，2 表示输入错误；视觉 suite 必须全部到达才退出 0。模拟传感器 CLI 与图像 CLI 均拒绝覆盖旧结果。

图像 CLI 默认 R000→R007 适用于 test_locked；换布局时显式传其节点，或使用 --suite。真实采集的 missing_view/stale/future 故障与推理后注入的置信度/结构错误有明确区别，日志分别保留原 prediction、控制器 frame 与 injection，不将注入说成模型改善。

## 7. 日志与指标

controller.history 包含帧 ID/时间、观测、候选、状态原因、确认帧 ID、命令 ID、反馈与是否消费；controller.metrics 保存恢复开始/成功/失败、恢复时间、停止请求、重复反馈、转移次数等。

steps.prediction 保留模型旁路包；steps.truth 与 audit 仅用于事后审计。mislocalized_commands 统计从错误位姿估计发出的命令，不代表已经测量真实碰撞安全。waiting_time_s 为调度器输出 stop 的等待区间累计，不包含原子执行的真实行驶耗时。驱动 audit.stop_requests 包含最终停车，协调器 stop_requests 只统计非终止等待快照，二者口径不同。

报告必须保留所有失败。原 test_locked 已暴露，只用于回归；19 项故障是工程验证，不是新独立场景的泛化成功率。完整对照、消融和新的封存布局属于第七块。
