# 当前回归、历史报告与全仓检查

日期：2026-10-29。测试范围必须显式标注，不能把“当前代码测试通过”写成“历史冻结与正式导航验收全部通过”。

## 常用入口

```bash
cd /home/sunrise/toponav_project
# 日常当前代码回归（tests/，包含其子目录）：
.venv/bin/python tools/run_project_tests.py --scope current

# 默认严格全仓检查，显式递归项目根目录，包含历史测试及其他目录中的测试：
.venv/bin/python tools/run_project_tests.py

# 单独检查报告目录内的历史测试：
.venv/bin/python tools/run_project_tests.py --scope historical
```

默认自动创建`/home/sunrise/toponav_project/reports/test-scope-时间-随机标识`。
也可用`--output /absolute/new/directory`指定不存在的新目录；项目内输出必须是`reports/`的直接新子目录，禁止写入历史报告内部或源码目录，已有目录一律拒绝覆盖。

新入口不创建`pytest.ini`、不修改默认收集规则、不加skip/xfail、不改历史断言。普通`.venv/bin/python -m pytest -q`仍按原规则检查全仓；新入口的默认`all`显式传入`.`，防止未来配置中的`testpaths=tests`把历史目录悄悄排除。

三个范围都显式将项目根目录及现有的`ros2_ws/src/toponav_bridge`包根加入子进程`PYTHONPATH`，并在命令工件中记录。历史测试不再依赖先收集当前ROS测试时偶然产生的`sys.path`副作用；这只支持本地纯Python测试导入，不表示已安装或验收真实ROS运行环境。

## 范围的含义

| scope | pytest目标 | 结论边界 |
| --- | --- | --- |
| current | tests/ | 只说明当前回归，不评价未运行的历史报告 |
| historical | reports/ | 历史报告内测试在**当前源码/环境**上的结果 |
| all（默认） | . | 根目录递归实际收集的所有测试，不限于以上两处 |

审计按实际源文件路径分成`current`、`historical`、`other`，不再凭JUnit classname前缀猜测来源。参数ID中的点号/`::`不影响分类；其他目录中的测试不会从`all`中消失。源文件解析到项目外的符号链接归`other`，不能冒充当前目录回归。

`historical`**不是重建旧候选环境，也不是精确历史回放**。旧候选针对当前live源码的文本对照可能因合理演进而失败。本入口不豁免或重新解释失败；若需验证旧源码，应在独立目录恢复其原冻结源码、输入及环境，不能改旧哈希来迁就新实现。

## 输出工件

- `command.json`：实际Python/pytest命令、工作目录、范围及有效`PYTHONPATH`和本地包根。
- `collection.json`：收集/选中/排除的完整node ID、实际源文件路径与SHA256，发现配置、被忽略的测试路径及收集阶段跳过。
- `pytest.log`：pytest完整原始输出；失败堆栈和依赖警告保留。
- `junit.xml`：原始JUnit结果。
- `pytest-audit.json`：每项测试的setup/call/teardown结果、分组计数、收集/内部错误及中断情况。
- `summary.json`：范围状态、退出码、失败清单、完整性标志及上述工件哈希。

工件用独占创建方式写入，不覆盖旧记录。审计插件仅由新入口显式加载，不自动加载到普通pytest，也不改变测试的选择或结果。

## 严格退出规则

1. pytest本身非零时保留其正退出码；进程被信号终止等负返回码归为入口失败1，原返回码仍记录。
2. 即使pytest返回0，存在skip、xfail、非严格XPASS、排除测试、被忽略的候选测试路径、收集记账不全、未完成测试或缺失审计工件时，入口仍返回1，不报告完整通过。模块级`skip/importorskip`即使没有生成测试项，也单列`collection_skips`并拒绝完整通过。
3. setup/teardown错误单列`error`，不能因为call阶段通过就计作成功。没有测试时保留pytest退出码5。
4. 为避免隐式筛选，非空`PYTEST_ADDOPTS`和`PYTEST_PLUGINS`环境变量会在启动前拒绝；请显式清理后再运行。CLI不转发`-k`、`-m`等筛选选项。
5. 配置中的关键词/标签筛选、ignore/deselect、last-failed/stepwise模式和非默认Python发现模式会记录为发现限制，不能给出“完整通过”。标准的隐藏目录、venv、构建/缓存目录排除仍保持pytest原行为。

`all_scopes_passed`仅在实际`all`范围完整通过时才可能为true。`current`范围通过仍为false，其`unselected_groups`明确指出其他范围未检查。
所有范围都固定`formal_navigation_acceptance=false`；单元回归、历史静态断言与S2/S6/ROS/实车正式验收是不同证据。

## 当前已知的历史冲突

- `7d-recovery`：旧候选要求live `StraightPolicy`片段与旧版一致，已因先前策略演进冲突。
- `7d-navonly`：旧候选要求转角/路线片段不变，已因第一批终态锁存修复冲突。

这两项仍是真实失败，不改写、跳过、xfail或设置“预期失败名单”。默认全仓入口继续返回非零。日常可明确运行`current`来区分新回归，但交付时必须同时公开历史/全仓结果。

### 原有历史发现限制

`reports/block7/7d-20261018/conftest.py`原有`collect_ignore = ["test_analysis.py"]`，用于避开与7C报告同名模块的冲突。同目录的`test_analysis_7d.py`与被忽略文件字节完全相同，且其17项测试实际已收集。但这仍是一条真实忽略规则，不是“没有任何排除”。新入口保留原行为，把该路径写入`ignored_test_paths`，**不按文件名或相同哈希豁免完整性检查**；历史/全仓的`verification_complete`仍为false，不能宣称零排除的完整通过。本轮未新建或修改这份历史`conftest.py`。
