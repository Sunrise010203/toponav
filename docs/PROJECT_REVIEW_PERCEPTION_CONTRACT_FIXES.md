# 项目检查：第六批修复——感知接口契约与预测时间戳

日期：2026-10-29（Asia/Shanghai）。本批仅修复损坏API输入的边界处理，不是unknown识别能力改善，也不是Gazebo/ROS/S2/S6验收。

## 问题与修改

涉及`roadvision/contract.py`与`roadvision/learn.py`：

1. **布尔值冒充概率**：原来`[False]*8+[True]+[False]*8`会生成置信度1的出口。现在概率必须是有限、非布尔实数，逐项位于`[0,1]`；略大于1（例如1.000001）也直接拒绝，不再借归一化容差输出大于1的置信度。不隐式转换字符串、不clip、不重新归一化。
2. **有效标志按真值放行**：原来`valid="false"`或`valid=1`也能授权。现在只有内建`True`能声明有效，包括NumPy布尔标量在内的其他值都返回完整UNKNOWN：四方向为`None`、置信度0。无效帧龄也返回UNKNOWN；巨大整数帧龄不再引发有限性检查溢出。
3. **输入形状/阈值异常不统一**：接受可按整数索引的一维17项实数向量，包括list、tuple及NumPy一维实数数组；拒绝映射、字符串/字节、生成器、集合和二维数组。形状、概率或阈值损坏统一抛`ValueError`。阈值仍为`[0.6,1.0]`内的有限非布尔实数；17类顺序与原`math.isclose(..., abs_tol=1e-5)`归一化容差不变。
4. **时间戳产生不一致或非法日志**：`Predictor.predict`在推理前校验显式采集时间，在推理结束后校验时钟值。仅接受非负内建整数纳秒，拒绝布尔、浮点、字符串、NumPy整数及超出有限数值诊断范围的巨大整数，统一抛`ValueError`，不构造虚假的预测包。显式整数原样保留，不转浮点、不重打时间；`None`仍使用原`time.time_ns()`默认值。没有新增任意64位日期上限。

三种处理必须区分：**损坏概率/阈值抛异常；非法valid/age返回UNKNOWN；非法timestamp抛异常且不返回预测包**。这不是实际执行器已收到停车命令的证据。调用方仍需自己的异常处理/停车守卫，本批未改变传输层。

合法时间行为不变：年龄恰好1秒仍可接受，超过1秒拒绝；采集时间为0按旧逻辑过期；合法未来整数时间仍给出`invalid_timestamp`。missing_view、stale_frame、unobservable、low_confidence的原拒绝优先级不变。巨大但仍处于有限数值范围的整数（例如`10**300`）可参与计算并被拒绝为未来帧；`10**400`在边界提前拒绝。

## 复现与测试

- 修改前快照：`/home/sunrise/toponav_project/.backups/review-perception-contract-20261029-203047`，保存生产文件、README及11份历史测试/conftest的哈希。
- 本轮报告：`/home/sunrise/toponav_project/reports/project-review-perception-contract-20261029-a9su4moi`。
- 新增`tests/test_review_perception_contract.py`共109项。修改生产代码前运行：**48失败、61通过**；修复后连同原相关测试：**191项全部通过**。原始日志、JUnit及退出码均保留。
- 三组畸形有效标志通过真实`NavigationSession`验证：连续三帧也不能累积导航命令。预测器测试执行真实`predict/prepare`，用合成Torch logits控制各类故障；不把合成概率当成冻结模型准确率。
- 独立`check_frozen_model.py`实际加载相同冻结权重，旧预测器绑定快照中的旧解码器，与新预测器比较已有启动/短走廊RGB及一视图黑屏/全黑构造图。四组结果除`inference_ms`外完全一致，含概率、类别、接受状态、观测及时间字段。采用相同构造整数时钟隔离耗时影响，**不是实时采集、新独立数据或物理行驶验证**。
- 源码片段对照确认原预处理、模型架构、Predictor初始化、训练、特征提取、统计及评估未改；冻结模型SHA256仍为`1188f35dd20d0773b780b73f2000daa25fede9b2db098b02dc0746ad0de6b03b`，识别阈值仍为0.6。

## 最终回归

在项目`.venv`下运行，Python 3.10.12、pytest 8.4.2：

| 范围 | 结果 | 入口退出码 |
| --- | --- | --- |
| 新增契约回归 | 109项通过 | 0（包含于下行） |
| 新增＋原相关测试 | 191项通过 | 0 |
| 独立current | **2466项通过** | 0 |
| 默认all | **2638项通过、2项失败** | 1 |

全仓的两项失败仍是历史`7d-navonly`与`7d-recovery`对live源码的冻结对照冲突，历史范围172通过、2失败，未增加新失败。没有测试skip/xfail/deselect、收集错误或收集阶段跳过；原有重复测试忽略路径`reports/block7/7d-20261018/test_analysis.py`仍披露，不能宣称全仓完整通过。

全量工件：

- `/home/sunrise/toponav_project/reports/perception-contract-current-20261029-a9su4moi`
- `/home/sunrise/toponav_project/reports/perception-contract-all-20261029-a9su4moi`

本轮`VERIFICATION.json`记录各文件/模型/结果哈希、前五批工件保真、历史文件未变及冻结模型只读对照。新输入校验是有意收紧的API边界；未更新旧冻结哈希来强行通过重放，也没有向测试入口添加失败白名单。

## 尚未解决

不训练/替换模型，不更改预处理、识别阈值、相机、地图、控制器运动门槛或训练评估逻辑。真实RGB下unknown/遮挡误接纳、普通直走完成率与近场可观测性、原模型Gazebo C02连续绕行仍未解决；S2/S6原始证据缺失，本批不补造，不声明S2通过或S6完成。
