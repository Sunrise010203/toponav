# 直行参考候选搜索准备复用：开发记录

日期：2026-11-03（Asia/Shanghai）。本轮是等价计算去重，不是整段直行优化成功或新物理验收。S2未通过，S6未完成。

## 改动

仅生产 `roadvision/free_ground_path.py`、`roadvision/reference_patch_evidence.py` 两文件改变。
一次九格×九候选列表内懒创建一个 PreparedAlternative，复用参考图 float32 转换、坐标和支持腐蚀。
不缓存匹配结果、不跨帧、不改变候选集合/顺序/选点、分数/原因/逻辑计数或阈值。
有效候选仍做全图匹配；81个有效候选的匹配次数仍81，腐蚀由81减为1。平坦/全无支持仍0次准备和匹配。
目标图双向与自重复、整流重复否决、S1默认入口、控制器、速度、制动、恢复、0.8s/0.4s门控均未改。
默认仍no_motion，各显式实验策略没有获得推广。

`before/`保存本轮修改前的两份生产源码及既有测试快照（后者以.py.snapshot保留，避免额外pytest发现）。
既有测试唯一修改是oracle接受并**忽略**私有_search关键字，仍独立执行原始四次搜索；没有削弱断言。
新增72项准备路径测试；原default三参数reference接口仍执行未准备路径。

## 等价性与性能

源码/fixture/工具pin见MEASUREMENT_PINS.json；SHA256：
`86ad89a7a4e565ef85ea35e4815731031722f64943a1df301204f555bd4d2e09`。
测量前/后及最终交付核对全部pin相符。
比较原始源码快照而不是拿四次搜索的更早版本当基线；启动函数仅复制不变代码并替换候选函数绑定，绝不改写生产全局。
18组输入，每实现各预热1次，8轮×3样本（每组每实现24次），AB/BA交替。OpenCV单线程，结束还原；未调BLAS。
计时仅包围内存中函数调用，完整输出JSON字节比较在计时外；所有预热与样本输出一致。
18组×2实现×24=864个正式计时样本，全量样本、CPU/墙钟统计和输出摘要均保存。
不删除慢样本、不以p95等小样本分位数承诺部署SLA。

| 输入/调用范围 | 修改前中位ms | 修改后中位ms | 耗时减少 |
|---|---:|---:|---:|
| candidates--synthetic_random | 25.112 | 22.545 | 10.22% |
| candidates--synthetic_blur | 25.633 | 22.901 | 10.66% |
| candidates--synthetic_flat | 3.082 | 2.917 | 5.35% |
| candidates--synthetic_repeated | 25.943 | 22.626 | 12.79% |
| candidates--synthetic_padding | 3.467 | 3.583 | -3.35% |
| candidates--center_a | 25.088 | 22.327 | 11.01% |
| candidates--repeated | 19.091 | 17.721 | 7.18% |
| candidates--floor_paint | 25.656 | 22.747 | 11.34% |
| candidates--default_start | 25.839 | 23.440 | 9.28% |
| startup--center_a | 27.371 | 24.177 | 11.67% |
| joint_startup--center_a | 49.513 | 46.700 | 5.68% |
| startup--repeated | 20.438 | 17.776 | 13.02% |
| joint_startup--repeated | 30.617 | 26.861 | 12.27% |
| startup--floor_paint | 29.416 | 24.280 | 17.46% |
| joint_startup--floor_paint | 46.179 | 42.559 | 7.84% |
| startup--default_start | 27.876 | 25.955 | 6.89% |
| joint_startup--default_start | 47.258 | 42.623 | 9.81% |
| free_ground--exposed_gradient | 35.129 | 30.809 | 12.30% |

合成遮罩案例退化3.35%（约0.116ms），如实保留。范围包含完整startup与一个free-ground证据包，**不包含整个controller.update、采集、IPC、执行或制动**。
已暴露四张RGB的参考选点减少7.18%–11.34%；不能把局部节省宣称整车提速、10Hz已达标或直行完成率提高。

## 验证范围与失败保留

1. 修改前相关159项：首次普通pytest被ROS launch自动插件导入整个tests目录后遇缺历史文件，6个收集错误，未执行。原日志baseline-tests.log保留。随后显式关闭第三方自动插件，只跑原选定6文件，159通过（baseline-isolated-tests.log）。没有更改任何pytest配置。
2. 修改后第一次尝试只禁launch_testing，残留launch_ros报未知hook，0项执行，targeted-tests.log保留。随后仅禁用这两个ROS launch插件，相关7文件231通过，含新增72；其余插件正常加载（targeted-no-ros-launch-tests.log）。
3. 使用仓库标准run_project_tests.py --scope current原样执行：退出2、selected=0、verification_complete=false，收集缺reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json。完整审计另存reports/straight-reference-preparation-full-current-20261103。**不是完整current或all scope通过**。
4. 显式扩大到33个直行/通行/历史/描述子测试文件，仅禁两ROS launch插件，1046项：1013通过、33失败、1条warning，无skip/xfail（expanded-tests.log、expanded-junit.xml、expanded-command.json）。231项是其中子集，不能重复相加。
5. 33失败全部FileNotFoundError，缺reports/multi-node-20261016下layout/suite及reports/control-safety-20261018/suite.json。失败节点/消息见expanded-failures.json。在独立子进程把本轮变化的候选入口换成修改前源码，精确重跑这33节点：33失败，节点及错误消息与修改后相同（baseline-failures-*）。没有生成伪历史数据、skip/xfail或把失败改记为通过。
6. 本轮未恢复旧报告、未做旧逐帧精确回放；软件测试自带的短轮驱场景不算新冻结物理验收。没有native/Gazebo/设备生产接入或运动权限变更。

## 复现

输出目录必须不存在；原测量目录禁止覆盖。当前源码/fixture不变时可运行：

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m tools.straight_reference_preparation_benchmark \
  --before reports/straight-reference-preparation-20261103/before \
  --pins reports/straight-reference-preparation-20261103/MEASUREMENT_PINS.json \
  --pins-sha256 86ad89a7a4e565ef85ea35e4815731031722f64943a1df301204f555bd4d2e09 \
  --output /absolute/new/measurement-directory
```

这会重新测量，不保证得到原墙钟数值。源码或输入改变必须另行冻结，不修改旧pins冒充原候选。

## 未解决

普通四条件历史0/4、近场868格未知/九远点非覆盖、0.25m窄障碍、工作预算pending/来源认证和IPC旧故障均未修复。
本轮不推进2AB调用方状态模型到生产，不更改其schema或封存证据。
下一步应在可恢复的公共连续RGB序列上做完整控制器剖析，优先定位持续通行/几何证据中断；这需要独立记录数据，不能用本轮局部计时替代。
