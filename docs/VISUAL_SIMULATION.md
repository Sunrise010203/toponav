# PyBullet 导航可视化

这是既有真实轮驱运行器的本地浏览器观察面板，不是新的导航算法或预设路径动画。
不修改生产控制器、物理世界、模型、冻结套件和正式报告；每次使用新的结果目录。

## 启动

```bash
cd /home/sunrise/toponav_project
.venv/bin/python tools/visualize_simulation.py --demo left
```

在本机浏览器打开 <http://127.0.0.1:8765>。无需图形桌面、ROS2、额外依赖或单独启动 PyBullet。
`--demo left` 会运行一次新的 L 形左转仿真，并接受该声明夹具的初始净空前提；不是实车授权。
不自动运行可省略 `--demo`，在面板上选择场景、确认净空后点击运行。
其他预设：right、t-left、t-right、straight、blocked。

### 2026-10-29：直走运动许可独立确认

`straight` / `blocked` 仍默认 `no_motion`。面板新增单独的“授权直走受限探测”勾选项；
它与“确认初始净空”是两个前提，不勾选探测授权时运行器保持零速。
切换场景会清除探测勾选，不将其他场景的确认隐式转换为直走许可。
允许探测仍须通过原有纹理、静止、空间先验与距离预算检查，不保证能起步或完成任务。

命令行显式授权示例（仅声明夹具的仿真，不是实车授权）：

```bash
.venv/bin/python tools/visualize_simulation.py --demo straight --motion-permission bounded_probe
```

`--demo left/right/t-left/t-right` 不接受直走专用的 `bounded_probe` 参数。

左边是按实际审计位姿重建的俯视道路与车辆；右边是实际保存的四向 RGB。
轨迹以10Hz审计位姿插值显示，不是额外采集的高频定位；相机不插值。
物理模拟可快于观看进度，界面支持暂停、时间轴回放、速度选择和跳到最新位置。
启动时的未完成写入不会被当作完整图像/日志；最终结论来自 result.json。

底部区分任务位置审计、最终停稳、接触步数；无结果或运行中断不会显示通过。
障碍场景可能安全停车但不通过无障碍任务位置审计，不能把两者混淆。
观察器读到的物理位置只用于展示，不向控制器发送真值或运动命令。

## 结果、关闭与限制

原始记录位于 `/home/sunrise/toponav_project/reports/visual-simulation/<唯一ID>/simulation`；
同级 worker.log 保留原运行器输出。默认只监听127.0.0.1。
关闭浏览器不会停止当前任务；任务有既有有限tick预算。
在服务终端 Ctrl+C 会尝试中断子进程并执行原运行器停车清理；强杀进程不属于完整制动验收。

SSH远程访问可在客户端转发 `ssh -L 8765:127.0.0.1:8765 用户@虚拟机地址`。
端口冲突时使用 `--port 8766`。不要将未加认证的本地开发服务暴露到公网。

仅为声明灰墙/蓝灰地面场景的工程演示。S2正式验收未通过，S6最终评估未完成；
这不恢复历史正式流水线，不补签安全验收，不证明17类模型控制或实车能力。

## 测试

```bash
cd /home/sunrise/toponav_project
.venv/bin/python -m pytest -q -p no:cacheprovider tests/test_visualize_simulation.py
```
