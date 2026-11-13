# 第三分块：最小导航闭环（结构观测版）

2026-10-14，Asia/Shanghai。远程项目：`/home/sunrise/toponav_project`。

本分块现在使用**相对道路结构 + 执行反馈 + 候选位置集合**，不再把真实节点编号包装成识别概率交给定位。旧节点概率版本完整保留为旧基线；本次未修改其源码、测试、地图或历史报告。

| 版本 | 入口 | 定位依据 | 用途 |
| --- | --- | --- | --- |
| 当前结构版 | `python -m simulation.structural_navigation` | 任务起点/朝向先验、上一候选、执行反馈、新相对方向观测 | 第三分块当前实现 |
| 旧节点概率基线 | `python -m simulation.navigation` | 直接的各节点识别概率 | 保留对照，不作为结构定位验收 |

旧版使用方法见 `docs/NAVIGATION_NODE_PROBABILITY_BASELINE.md`。旧版代码仍是 `simulation/navigation.py`，不是把新算法套在旧节点概率输出后面。

## 1. 启动当前版本

在 Ubuntu 终端或 SSH 的远程 shell 中执行：

```bash
cd /home/sunrise/toponav_project
source /opt/ros/humble/setup.bash
source .venv/bin/activate
export MPLBACKEND=Agg
export MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib

python -m simulation.structural_navigation \
  --start N01 --heading 0 --goal N06 \
  --output reports/my_structural_run.json --plot reports/my_structural_run.png
```

`--start` 和 `--heading` 是任务明确给定的起始位置/朝向先验，默认地图起点 N01、朝东 0°；不是传感器读出的真值。当前不做未知起点主动探索。`--goal` 是任务地图目标节点。

```bash
# 已知 C02 关闭：经 N05 绕行，地图文件不变
python -m simulation.structural_navigation --disable C02

# 反向路线和任意一个正交起始朝向
python -m simulation.structural_navigation --start N06 --heading 180 --goal N01

# 确认执行失败：原地保留候选，重新观测后停止
python -m simulation.structural_navigation --scenario execution_failed

# 实际执行但反馈丢失；不能按成功处理，重新观测后停止
python -m simulation.structural_navigation --scenario execution_unknown

# 仅发送未执行；不能据命令直接推进位置
python -m simulation.structural_navigation --scenario execution_sent

# 方向识别全未知/低置信度；保留预测候选并停止
python -m simulation.structural_navigation --scenario ambiguous

# 结构与预测不匹配；报告不一致，不跳到另一个同类路口
python -m simulation.structural_navigation --scenario mismatch

# 每个非理想 scenario 默认在索引 1 注入，可改成首帧/首次命令
python -m simulation.structural_navigation --scenario execution_failed --fault-step 0

# 至多发送两条移动指令，不以真值中的移动次数控制算法
python -m simulation.structural_navigation --max-steps 2
```

`fault-step` 是从 0 开始的索引：观测故障对应观测索引，执行故障对应移动命令索引。这些场景是确定性故障注入，不是随机噪声统计。若故障索引超出实际路线，它不会被触发。

退出码：0＝控制器报告到达且独立真值审计通过；1＝安全停止或到达审计失败；2＝输入/文件错误。非理想场景正常退出 1，不表示程序崩溃。

## 2. 观测描述结构，不描述地点身份

```json
{
  "relative_open": {"front": true, "left": true, "right": false, "back": true},
  "confidence": 1.0
}
```

- 方向相对于当前车头，不是绝对东南西北。true/false 表示可通行/不可通行，null 表示无法判断。
- 只允许这两个顶层字段；不包含真实节点、坐标、朝向、边编号或节点概率。
- 观测根据物理场景和真实朝向生成；定位只用地图假设预测应该看见的结构，两条路径分别计算。
- 不使用 node_type 或 label 认领地点。N02 朝东和 N04 朝西的相对结构完全相同，都有前/左/后出口；观测相同不代表地点相同。

## 3. 候选集合更新

核心纯算法：`simulation/structural_localization.py`，不导入环境或场景模块。

```text
上一候选集合 {(地图节点, 朝向), ...}
      ↓ 上一相对动作 + 可信执行反馈
预测候选集合
      ↓ 新结构观测的一致性检查
匹配候选集合 + 明确状态
      ↓ 只有可信完整观测、唯一候选、执行状态已确认才继续
Dijkstra 从估计节点重新规划 → 发出一个相对动作
```

`update_candidates(graph, previous_candidates, observation, previous_action=None, feedback=None, disabled_channels=())` 不接收环境、场景或真实位置。

| 执行反馈 | 预测规则 | 后续处理 |
| --- | --- | --- |
| succeeded | 仅沿地图上该相对出口推进一条边并更新朝向 | 新结构一致且唯一才继续 |
| failed | 原位、原朝向，不推进 | 重新观测并报告 execution_failed，停止 |
| unknown / sent / 缺反馈 | 原位候选与可能成功的候选并集 | 新观测可排除不一致候选，但仍报告 execution_unconfirmed，停止 |

执行反馈只包含 `status`；accepted/已接收不等于 succeeded。本模拟约定 succeeded＝整条边完成，failed＝既未移动也未转向。若实车可能已经转向/部分移动，应归为未知，并补充更细执行模型，不能套用 failed 原地约定。

观测置信度低于 0.6 时不据其排除候选；部分方向未知时只核对已知项。为了最小闭环的保守性，只要观测不完整或置信度低，就停止请求可靠观测，即使历史仍只有一个候选。

| 定位/决策状态 | 含义 |
| --- | --- |
| localized | 完整可信观测与唯一候选一致，可规划 |
| ambiguous | 多个地点/朝向候选均匹配，不能任选一个 |
| observation_uncertain | 低置信度或部分方向未知，保留一致候选并停止 |
| observation_mismatch | 预测集合非空，但所有候选与新结构矛盾；不全图重定位 |
| transition_mismatch | 成功反馈对应的动作在所有先验候选上都不可能 |
| execution_failed / execution_unconfirmed | 已失败 / 尚未确认，停止等待处理 |
| unreachable / step_limit / reached | 无路线 / 指令预算耗尽 / 估计已到任务目标 |

`decide(graph, localization, goal, disabled_channels=())` 只读地图、定位结果和任务目标。相对动作 front/left/right/back 由估计朝向计算；执行器按真实朝向执行。算法不采用真值修正朝向。

## 4. 真值边界和逐步日志

模拟器/运行器：`simulation/structural_navigation.py`。复用旧二维通道执行器和绘图函数，不使用旧概率识别器。

每条 `steps` 记录明确分开：
- `truth.before/after/execution`：真实位置、坐标、朝向、路程和物理执行结果，仅用于审计。
- `prior_candidates`：上轮定位留下的地图候选，不从 truth 填充。
- `previous_action`、`feedback_used`：本轮定位使用的上一命令和公开执行反馈。
- `observation`：本轮相对结构及识别不确定性。
- `localization`：预测候选、匹配候选、估计、状态、原因和观测是否可靠。
- `decision`：动作、路线和决策状态。
- `execution_feedback`：执行本轮动作后返回的公开状态，下轮才使用。

每次发出动作后，无论成功、失败、未知还是仅发送，都再次观测，定位不能在命令发送时直接推进。终止时包含最终新位置的观测和 stop。

顶层 `status` 是控制器结论，`final_estimate` 是估计；`final_truth` 与 `audit` 是独立的单向验收结果。故意配置错误但结构相同的起点先验时，控制器可能误判到达，审计应失败；程序不得读真值把先验偷偷修正。这是明确暴露错误先验的限制，不是全局定位已解决。

## 5. 测试与验收

```bash
# 本分块+保留基线：316 项（旧 204 + 本次新增 112）
python -m pytest tests/test_dijkstra.py tests/test_navigation.py tests/test_structural_navigation.py -q

# 整个远程项目；可能还包含其他分块的并行开发测试
python -m pytest -q
```

实测结构版完成 N01→N02→N03→N04→N06，4 次移动、5 次结构观测、8 米；关闭 C02 经 N05 绕行。全部 64 个通道关闭组合 × 36 个起终点组合与 Dijkstra 对照，另测四种初始朝向的全部起终点组合。

同结构歧义、失败/未知/仅发送、模糊/不匹配、无效载荷和真值隔离均有测试。额外篡改整个运行过程暴露给日志的真值，控制器的候选、观测、决策和路线保持不变；真值只能影响最终审计。

验收见 `reports/STRUCTURAL_NAVIGATION_ACCEPTANCE.md`；八种运行日志为 `reports/structural_*.json`。旧版报告 `reports/NAVIGATION_ACCEPTANCE.md` 仍为历史节点概率基线，不能代替本次验收。

## 6. 范围

仍是离散节点级二维仿真：相对转向并沿一条通道移动是原子动作。没有连续动力学、部分移动、真实视觉、ROS 消息、实时避障或实车控制；不引入概率滤波。对歧义/失败/不匹配采用明确停止，不擅自恢复、全图跳转或主动探路。任务先验错误且结构对称时，仅靠本次最小实现无法保证识别错误先验。
