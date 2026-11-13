# Gazebo 工程交叉验证

日期：2026-10-20。已在本机 Ubuntu 22.04 amd64 / Python 3.10 下实际运行。

现已增加**多场景选择和原生拓扑图**，运行
`bash /home/sunrise/toponav_project/tools/open_gazebo_scenarios.sh`。
详见[多场景说明](/home/sunrise/toponav_project/docs/GAZEBO_SCENARIOS.md)。
六节点地图目前是场景/规划预览，不等于全图自主导航。

## 已接入什么

- Gazebo Harmonic（gz-sim 8.15.0）+ DART 动力学 + Ogre2 原生相机。
- 差速轮关节实际驱动车体；不是给定轨迹动画，不用真值位置转向或瞬移。
- 未修改的 `CornerController` 接收四向 RGB、各相机一致的原始采集时间、左右转任务和明确的初始安全距离先验。
- 俯视相机只用于回放；位姿、接触只存审计区，不输入控制器。
- 通过原生 Gazebo Transport 接入，**不是 ROS2 Humble 与 Harmonic 的桥接验证**。

本机采用私有目录安装，没有 sudo、没有改系统 Gazebo/ROS、没有更换虚拟环境依赖。
默认由脚本启动独占 Xvfb 显示器，真实渲染后离屏记录，**不会弹出 Gazebo GUI**。

## 1. 自己跑一次

### 弹出原生 Gazebo 窗口

在本机已登录的桌面会话下，加上 `--gui`：

```bash
bash /home/sunrise/toponav_project/tools/run_gazebo_cross_validation.sh \
  --gui --output "/home/sunrise/toponav_project/reports/gazebo-window-$(date +%s)" \
  --direction left --confirm-initial-clearance
```

窗口中会显示原生 3D 场景，运动约按实时速度执行。
结束后保持窗口，关闭窗口才清理服务并写最终 `result.json`。
执行中提前关闭窗口会停止运行，不算任务完成。
可用鼠标调整视角；为避免干扰采集时钟，这个只读窗口没有播放/步进按钮，步进仍由 RGB 运行器管理。
`gui.log` 记录原生 GUI 加载信息。无桌面的 SSH 会话请使用下面的离屏模式。

### 离屏运行

当前工作区已安装，无需再下载。每次选择一个不存在的结果目录：

```bash
cd /home/sunrise/toponav_project
RUN="reports/gazebo-manual-left-$(date +%Y%m%d-%H%M%S)"
bash tools/run_gazebo_cross_validation.sh \
  --output "$RUN" --direction left --confirm-initial-clearance
```

右转把 `--direction left` 改成 `--direction right`。本机一次约 11 秒墙钟时间，模拟约 30 秒；不要求桌面环境或手动配置 DISPLAY。
脚本退出时会清理自己启动的 Gazebo 和 Xvfb，不关闭其他显示器。
只在提供的、已知初始净空的静态测试场景使用确认参数；它不是视觉感知出来的安全保证。

测试“未确认初始安全距离时拒绝起步”：

```bash
bash tools/run_gazebo_cross_validation.sh \
  --output "reports/gazebo-manual-reject-$(date +%Y%m%d-%H%M%S)" --direction left
```

预期退出码 **1**，`controller_result.result_type=rejected_no_motion`、
`reason=initial_clearance_prior_not_confirmed`、`motion_commands=0`。
这里退出 1 表示没有完成导航任务，不意味着上述拒绝行为不正常。

## 2. 看真实相机回放

接上面成功运行命令：

```bash
.venv/bin/python tools/render_gazebo_preview.py "$RUN" "$RUN/preview.gif"
xdg-open "$RUN/preview.gif"
```

GIF 左侧为原生俯视相机，右侧为送入控制器的原生四向 RGB；默认约 4 倍速。
四向图像位置：左上前、右上左、左下右、右下后。画面由保存的原始 PNG 合成，没有补画车辆轨迹。
没有图形桌面时，可直接在 Codex 中打开 GIF 文件，或复制结果到有桌面的机器查看。

现成回放：

- [左转 GIF](/home/sunrise/toponav_project/reports/gazebo-cross-20261020/gazebo-left-preview.gif)
- [右转 GIF](/home/sunrise/toponav_project/reports/gazebo-cross-20261020/gazebo-right-preview.gif)
- [本轮报告](/home/sunrise/toponav_project/reports/gazebo-cross-20261020/REPORT.md)

## 3. 判断是否正常

查看新结果目录中的 `result.json`：

| 字段 | 成功的受限 L 转弯应看到什么 |
|---|---|
| `exception` | `null` |
| `dart_plugin_loaded` | `true`，由真实加载日志核对 |
| `controller_result.state` | `visual_stop` |
| `controller_result.result_type` | `completed_visual_corner` |
| `audit.corner_geometric_exit_pass` | `true`，真值只作事后检查 |
| `audit.stationary` | `true` |
| `audit.observed_wall_contacts` | `0`，仅表示未观察到墙接触 |
| `completed` | `true`，仅代表本工程样例条件满足 |

几何判据是停车时距出口中心线小于 0.20m、沿出口前进超过 1.10m、朝向误差小于 6°；
最终两次原生位姿的线速度小于 0.01m/s、角速度小于 0.02rad/s。
结束前额外执行 1 秒零速指令以观察停车。
基础接触传感器未发送空消息，**不能据此证明完整碰撞覆盖或安全认证**；结果明确保留 `collision_free_certified=false`。

输出目录还包括：

- `world.sdf`：实际执行场景；`freeze.json`：控制器、适配器及场景 SHA256。
- `gazebo.log`：物理/渲染引擎加载及故障日志。
- `decisions.jsonl`：逐帧原始时间、图像哈希、决策与门控输出。
- `public/frames/`、`public/overview/`：真实四向/俯视图像。
- `audit/poses.json`、`audit/contacts.json`：只用于事后审计。

纯公开输入离线重放，不加载 Gazebo，不读取位姿/接触审计：

```bash
.venv/bin/python -m simulation.gazebo_replay "$RUN" \
  --direction left --confirm-initial-clearance
```

左右转及先验参数必须与采集运行一致；预期 `all_decisions_match=true`。
重放逐帧验证 RGB 哈希以及完整决策内容，不等同于重新验证物理引擎。

## 4. 安装与排障

本机已安装的私有前缀：
`/home/sunrise/toponav_project/.simulators/gazebo-harmonic`。
环境脚本只修改当前 shell 的搜索路径。适配器为每次运行生成独立 `GZ_PARTITION`。

相同 Ubuntu 22.04 amd64 环境重新准备时，可以运行：

```bash
bash tools/install_gazebo_local.sh
```

该脚本从签名的 OSRF/Ubuntu 软件源下载依赖并解包，不执行系统安装；
需要网络、可用的系统 apt 索引及现有 Python 虚拟环境。
本次实际安装采用相同的 download-only/解包步骤；**未另做全新机器上的安装脚本端到端复测**。
下载的包哈希见本轮报告的 `setup/DOWNLOADED_DEBS.sha256`。

重要排障经验：

- 本机 `--headless-rendering`（EGL）会给出纯天空图；控制器正确拒绝。
  不要把它算作成功，也不要修改控制器绕过。启动脚本默认使用已验证的 Xvfb + display 渲染。
- 缺少 Ogre 私有动态库/资源路径会导致启动失败；通过 `gazebo_env.sh` 设置，不修改系统库。
- Gazebo Python Transport 的同步请求会占住 GIL；步进服务独立进程，避免阻塞相机回调。
- 步进服务返回只是排队成功，适配器还等待原生模拟时钟实际前进，不能拿创建时的空图驾驶。
- 结果目录不得复用；失败目录与日志也应保留。

## 5. 结论边界

已验证的是**熟悉的静态 L 型走廊中，连续 RGB 转弯控制器跨到 Gazebo 后能运动并停车**。
走廊宽 1.8m、拐角中心 x=2.5m、出口长度 3.5m；四向相机各 128²、90°视场、水平安装高约 0.45m。
与 PyBullet 对齐名义几何/相机规格和蓝灰地面、灰墙条件，但没有移植原程序纹理；
渲染器、材料实现、轮接触与动力学不同，不是像素等价复刻。

这不是新的未见场景冻结验收，也不是完整拓扑导航、17 类网络识别或实车验证。
没有修改/训练原控制器和 17 类模型，没有重跑旧正式流程或追认 S2/S6。
原有 `arrival_status=unverified`、`atomic_edge_compatible=false` 保留。
同步暂停式模拟中的命令门控也不构成硬件看门狗、断链停车或 ROS 桥接的认证。
