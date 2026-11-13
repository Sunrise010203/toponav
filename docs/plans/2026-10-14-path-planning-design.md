# 第二分块：路径规划迁移设计与执行计划

## 范围

运行环境由用户确认仍为虚拟机 `/home/sunrise/toponav_project`。新增独立模块 `simulation/path_planning.py`，复用 `simulation/dijkstra.py`；原 Dijkstra、原测试、旧离散仿真及所有地图文件保持不变。旧 `navigation.py` 的绝对方向动作只属于其历史离散仿真，不是新规划块的控制接口。本次不修改该仿真执行器，也不增加 ROS、运动控制、定位算法或复杂规划算法。

## 接口

- `plan_from_estimate(graph, estimate, goal, disabled_channels=())` 返回节点和通道路线，定位明确时直接调用原 Dijkstra；定位不明确则返回 `localization_uncertain`，不使用地图默认起点或最大概率候选。
- `next_action(graph, estimate, route, disabled_channels=())` 只转换路线的第一个出口，输出 `straight/left/right/uturn/stop`。停止时明确是否请求重新观测。不会输出 east/north/west/south 控制动作。
- 沿用已有定位结果格式 `status=localized` 与单个 `node_id`。这表示上游已接受该节点估计；本模块不重新选取概率分布中的候选。
- 兼容同一虚拟机新出现的第一分块结构定位完整返回值：顶层 localized、观测可靠、单个节点/朝向候选且 estimate 与其一致时才接受。未确认执行或多假设等顶层状态不因存在 estimate 就被绕过；不改动第一分块源码。
- 可扩充 `heading_deg` + `heading_status=certain`，或 `incoming_channel` + `incoming_status=certain`。未显式标记可信时不使用该信息。显式 uncertain、缺失、方向证据非法或相互矛盾时停止并请求重新观测。
- 使用节点 `position_m` 与通道端点求出口及到达方向；不将地图 `actions` 中的绝对方向直接当作控制动作。坐标约定：x 东、y 北；角度东为 0、逆时针为正。
- 进入通道在当前节点处确定到达朝向：另一端 → 当前端。单向边不能反向推定到达；已通过后关闭的通道仍可作为确认过的到达证据，但不会被规划为可通行出口。
- 只处理 0/90/180/270 度相对转角（1e-6 度容差仅消除浮点误差，不是置信度或操控容差）。其他转角不擅自舍入，停止请求观测；不模拟连续转向。
- 起点等于目标、目标不可达均停止；位置改变或下一通道关闭导致旧路线失效时停止要求重新规划，不继续执行缓存动作。

## 三步执行与验收

1. 新增 `tests/test_path_planning.py`，在虚拟机运行确认未实现时缺少模块；测试涵盖四向出口/四种相对转角、同出口不同进入通道、不确定/冲突信息、到达、不可达、断路重算及地图不变。
2. 实现两个独立纯函数及最少辅助函数；运行新测试，随后运行全部原测试。基线为 204 项通过，其中原 Dijkstra 测试 104 项；保留已有全通道组合验收。
3. 保存 `docs/PATH_PLANNING.md`、本次测试日志与验收报告；核对原 Dijkstra/测试、旧仿真/测试及地图 SHA256 未变，并同步新增交付文件到 Windows。

实际测试命令：`.venv/bin/python -m pytest -q`，在远程项目根目录运行。新模块入口通过 Python 导入调用，不新增命令行参数框架。没有 Git 仓库，不创建仓库、不提交、不修改环境。
