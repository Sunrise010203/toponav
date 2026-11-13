# 普通直走增量：联合参考选点与前表面标量接口

日期：2026-11-01（Asia/Shanghai）。用户要求继续修改普通直走。本轮不训练/更换17类模型，不降低数值、纹理、制动、恢复或租约门槛，不增加传感器或改变安装/标定，不启用新的主动恢复动作。**S2未通过、S6未完成，不能把本轮工程增量称为正式导航验收。**

工件入口：[本轮报告目录](/home/sunrise/toponav_project/reports/straight-followup-20261101-01)。

## 1. 启动选点：显式reference实验

原实现先在每个网格的九个固定候选中选原生图像最唯一的补丁，再对选中的一个做整流纹理检查。如果原生最优点恰好不满足整流检查，不会使用同一各轴±1cm范围内同时满足两种检查的候选。

新增`roadvision/reference_straight.py`：仅使用**同一张静止参考RGB**，在同样的九个位置里联合评估原生与整流支持。原生有效性优先，其次整流有效性，再用原来的原生唯一性分数稳定排序。不是从后续帧、几何拟合或物理结果中搜索通过者，也不对目标帧失败重试。

仍要求三张真实静止启动图分别满足：九个原生单元全部有效、至少六个整流单元有效、三个前向和三个横向分箱都有支持。显式`bounded_probe`授权、操作者净空确认、0.14m探测预算、停车余量、停车确认、三次重新确认及恢复次数等继承原控制器。

**相同数值门槛不代表相同接受集合：联合选择确实会放行原先拒绝的部分参考图，因此不推广为默认策略。** 默认仍调用原`assess_startup_texture`，该文件与修改前逐字节一致。原通行检查仍按原生参考选点；提取出的候选生成器在12组图像/遮罩条件、108个单元上与旧选择结果完全一致。

### 第一候选结果

冻结源码及条件后，执行13条件×2版本，共26次轮驱闭环及公共RGB逐决策回放。

| 冻结后生成的普通条件 | 基线跟随指令 | reference跟随指令 | 基线/reference几何完成 |
| --- | ---: | ---: | --- |
| 中心启动 | 0 | 16 | 否/否 |
| 初始偏航+2° | 0 | 8 | 否/否 |
| 横偏+0.06m | 0 | 15 | 否/否 |
| 横偏-0.06m | 9 | 9 | 否/否 |
| 合计 | 9 | 48 | **0/4、0/4** |

已知默认6m/seed313：三张启动图原生支持均为9，整流支持均从5变为8；reference实际跟随11次、位移约0.2576m，最终仍因证据恢复超时中止。它是开发复现，不算新的独立正例。

重复纹理、均匀地面、涂色、未确认净空、未授权均零运动拒绝。重复/均匀案例在更早的地面运动检查被拒绝，不能据此单独证明新联合选点已普遍解决重复纹理。缺帧和冻结均在已有运动后实际注入并停车。26次事后物理/停车检查均无错误；进展/故障覆盖检查基线8/13、reference12/13满足，不等于导航完成率。

详见[第一候选报告](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/ROUND1_REPORT.md)、[冻结条件与源码](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/suite.json)、[完整结果](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/COMPARISON.json)。

## 2. 真实近障碍案例发现的接口缺陷

第一候选近障碍案例产生有限、有序的区间`[0.9982514996105356, 1.1158100356034688]`，但两个端点是`numpy.float64`对象。控制器的严格对象列表校验因此输出`invalid_front_surface_interval`。JSON落盘后类型会变为普通浮点，**仅查看JSON数值会漏掉这个内存接口问题**。

在S2生产适配层`roadvision/straight_evidence.py`将两元素列表中的NumPy float16/32/64规范化成等值Python float。没有放宽消费端检查，没有改写数值、顺序或置信区间，没有把bool、string、complex或long-double强转成有效数据；NaN/Inf转换后仍被拒绝。S1研究生产者及17类模型保持不变。

这项接口修复对所有S2直走版本生效；它不同于只在显式reference版本启用的联合启动选点。修复后的开发回归使用新的源码冻结和独占目录，第一候选的失败证据保持原样。

详见[原始类型诊断](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/FRONT_SCALAR_DIAGNOSTIC.json)、[最终候选冻结](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/round2/suite.json)。最终闭环与回归结论见本轮`REPORT.md`。

最终候选在已暴露近障碍条件上，从`invalid_front_surface_interval`改为`visual_stop`，制动距离0.01218m、仿真时间延迟0.1667s，视觉停车时实际已静止，0墙/障碍接触，14条公共RGB决策与三张终态帧精确回放。它不是目标节点到达或普通整段导航完成。另一个已知4m/seed31013普通案例修复前后66条决策及完整物理审计相同，均位移约0.6366m后证据超时退出，没有伪造功能收益。

## 3. 近场盲区没有被修复

对第一候选首次跟随的真实公共RGB做只读完整扫掠审查：868个单元全部未知，没有以远处九个点充当近场区域覆盖，也没有把该检查伪装成已接入的新运动授权。新增选点不提供完整近场几何、普遍障碍检测或实车安全认证。

[近场审查原始结果](/home/sunrise/toponav_project/reports/straight-followup-20261101-01/NEAR_FIELD_AUDIT.json)。

## 4. 运行

默认仍是原启动选择；需要显式选择实验版本，输出目录必须不存在：

```bash
cd /home/sunrise/toponav_project
/home/sunrise/toponav_project/.venv/bin/python -m simulation.continuous_straight_run \
  --output /home/sunrise/toponav_project/reports/my-reference-straight-NEW \
  --length-m 6 --seed 313 --straight-variant reference \
  --confirm-initial-clearance --motion-permission bounded_probe
```

这个已知默认案例目前仍应报告未完成；非零退出码不是工具坏了，也不能靠调低门槛刷成成功。省略授权或净空确认应拒绝运动。实验版本禁止混入转角/路线模式，公共任务和回放严格绑定版本。

## 5. 验证边界

- 新增21项联合选择/模式/回放测试、19项标量接口测试；没有修改旧测试或增加skip/xfail。
- 第一候选当前目录3869项通过；其全仓检查在发现需修复标量接口后主动中断，原退出码2、未完成项与中断说明全部保留，不能称全仓通过。
- 最终候选全仓实际选中4062项：当前3888/3888通过，历史172通过/2项原有源码快照断言失败；总计4060通过/2失败、18条既有warnings，无skip/xfail。原历史同名分析文件忽略规则仍披露，因此全仓未通过、`verification_complete=false`。没有为最终候选另跑一个current scope；3888来自这次all scope的实际当前目录分组。
- 隔离旧生产者的首轮公共回放因缺少验证schema而失败，原错误保留；补齐相同配置的私有副本后，仅做只读回放，66条决策及三张终态帧一致，没有重新运行物理或覆盖原失败记录。
- 四个普通新条件已暴露；以后不能调参后仍把它们称为未见独立测试。当前近场覆盖、持续几何/通行证据及普通直走完成率仍是后续工作重点。
