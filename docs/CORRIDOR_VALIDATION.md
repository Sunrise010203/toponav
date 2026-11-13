# 6B-1V：冻结直走控制器的边界验证

本轮回答：在改变直走廊尺寸、表面颜色和光照后，原6B-1能继续工作、会拒绝，还是可能误停车？不开发转角，不调整控制器，不训练模型。

## 冻结与测试集合

计划：`/home/sunrise/toponav_project/docs/plans/2026-10-15-corridor-validation.md`。

原始预声明清单：`/home/sunrise/toponav_project/configs/evaluation/corridor_validation_v1.json`。

清单共26例：旧条件锚点1例；新条件25例（尺寸4、材质6、光照6、组合4、涂色2、障碍3）。这是新尺寸的**直走廊**，不是转角或新拓扑。所有数值在第一次新条件运行前冻结，测试后不调参、不替换失败场景。

尺寸变化包括4.8–8.4m长度、1.5–2.6m宽度及未用于前轮开发的偏移/偏航组合。材质包含新蓝色、灰/棕/绿地面及蓝墙；光照包含亮暗、方向、色温和阴影；歧义案例包含无物理碰撞的地面涂色、新位置障碍和与地面同色的障碍。

这不是盲测或随机抽样：测试设计参考已知颜色/几何假设，由同一项目在冻结后预声明。不能把这25例的比例解释成任意真实环境成功概率。没有使用原图像模型的test_locked选择参数。

## 如何做到不改原控制器

评测入口：`/home/sunrise/toponav_project/simulation/corridor_validation.py`。

- 原 `/home/sunrise/toponav_project/roadvision/corridor_control.py`、`/home/sunrise/toponav_project/simulation/corridor_motion.py`、原测试均只读保留。
- `ValidationWorld`继承原物理世界，仅改变静态物体RGB、场景指定障碍位置及TinyRenderer光照；地面涂色是无碰撞视觉薄层，不是隐藏墙体。
- 评测中使用作用域内的世界工厂替换，复用**原run_corridor循环**。这是单进程串行评测适配，不是生产入口变更；结束或异常都会恢复工厂，不能在同一进程并发运行。
- 渲染适配器保持原四向相机安装、分辨率、视场与拼图顺序。基础设施测试只用旧条件锚点，在几次直行/转向指令后逐像素、逐物理状态比较一致，再运行新条件。
- 控制器仍只接`frame_id / captured_sim_s / rgb`。场景配置、光照、物体位置、真值或接触不进入控制器。默认VisionConfig、超时、帧龄、增益和停车条件不变。

## 复现

已冻结的锁文件：`/home/sunrise/toponav_project/reports/corridor-validation-20261015-fR5B4u/freeze.json`。它绑定控制器、原运行器、评测器、测试、计划及场景清单哈希和默认策略；源码/清单变更会拒绝继续按该锁运行。

```bash
cd /home/sunrise/toponav_project
export PYTHONDONTWRITEBYTECODE=1
export OMP_NUM_THREADS=2 OPENBLAS_NUM_THREADS=2 MKL_NUM_THREADS=2
export MPLBACKEND=Agg MPLCONFIGDIR=/home/sunrise/toponav_project/.cache/matplotlib
.venv/bin/python -m simulation.corridor_validation \
  --suite configs/evaluation/corridor_validation_v1.json \
  --lock reports/corridor-validation-20261015-fR5B4u/freeze.json \
  --output /home/sunrise/toponav_project/reports/a-new-corridor-validation-run
```

输出目录必须不存在。每个案例含原格式`run.json`、实际输入`frames/*.png`以及完整条件`case.json`。原run.json中的版本/范围文字保留6B-1原格式；新增光照、材质等实际条件以case.json为准，不要只看原scene_config。

`summary.json`同时保留全部26例、新条件25例和各分组分母。每个案例离线重新读取PNG、核对RGB哈希，重放到全新冻结控制器并比较**所有逐帧输出**，检查轮速映射、制动零命令和端墙审计。这个离线审计不能反过来修改已经执行的动作。

评测脚本退出0只表示**所有案例执行并通过证据一致性检查**，不是所有案例导航成功。运行/证据错误返回1，输入/冻结/输出目录错误返回2；错误案例始终留在分母中。

## 结果怎样理解

| 结果 | 定义 |
|---|---|
| `end_wall_verified` | 保持6B-1端墙位置/方向/速度门限，且全程无接触 |
| `safe_rejection` | 无接触且最终静止，但控制器中止；未发令和部分移动分别保留，**不是到达** |
| `visual_stop_unverified` | 图像支持停车，但端墙位置/横向/朝向等审计未通过，不能升级成节点到达 |
| `unsafe` | 任一物理步接触或最终速度不达标，即使之后停车也不能抹掉 |
| `execution_error` | 执行异常或证据不一致，保留在分母，不计为安全拒绝 |

碰撞检查频率为120Hz，控制与采图10Hz，全部使用原模拟时钟；真实计算耗时另记，不能据此声称硬实时或实车安全。

最终实测数字、全部失败及图表见 `/home/sunrise/toponav_project/reports/corridor-validation-20261015-fR5B4u/RESULTS.md`。后续若基于这些结果改控制器，本集合就成为开发回归集；新版本必须另设未参与调参的验证条件，不能再声称它仍是独立测试。
