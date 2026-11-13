# 项目检查：第四批修复——测试范围与报告分层

日期：2026-10-29（Asia/Shanghai）。只修改测试基础设施和说明，不修改导航/感知/规划生产代码、模型、阈值或历史验收证据。

## 修改

- 新增`tools/run_project_tests.py`：提供`current`、`historical`及默认`all`。`all`显式递归根目录，不改写pytest默认发现配置，也不默认排除历史报告。
- 新增显式加载的`tools/project_test_audit.py`：根据实际源文件路径分类测试，记录完整收集清单、源文件SHA256与逐阶段结果；历史失败不再只能混在一个总数中解释。
- 每次使用新输出目录保存命令、日志、JUnit、逐项审计和摘要。缺失证据、跳过/xfail/排除、未完成测试、收集或fixture错误不能被报为完整通过。没有按测试名称豁免任何失败。
- 更新README测试入口，避免继续把新的JUnit写进原`reports/system_unification/pytest.xml`，不覆盖历史记录。
- 新增`tests/test_project_test_runner.py`。小型独立pytest项目覆盖三种范围、根目录其他测试、参数ID、旧文件保真、收集错误、fixture错误、跳过/xfail/XPASS、隐藏筛选、防覆盖、环境/命令行限制及审计缺失。
- 为各范围显式设置并记录本地ROS桥接包的`PYTHONPATH`，消除历史测试对当前测试收集顺序的导入依赖。没有修改旧测试或安装/改动ROS生产代码。

使用方法、输出字段与严格退出规则见[测试说明](/home/sunrise/toponav_project/docs/TESTING.md)。

## 证据与边界

修改前README及10个历史Python测试文件的快照、哈希位于：
`/home/sunrise/toponav_project/.backups/review-test-scopes-20261029-194623`。
本轮开发验证记录位于：
`/home/sunrise/toponav_project/reports/project-review-test-scopes-20261029-tlu96ouu`。

这是“入口、报告和结论边界分层”，**不是把历史失败改成通过，也不是恢复旧候选环境**。默认全仓及独立历史范围仍会执行并展示两个已知源码对照冲突。普通pytest命令的收集配置不改，没有用收窄默认范围来获得绿色总结果。

### 开发中暴露的问题（原始失败保留）

- 首次独立历史检查：157通过、17失败。除两个已有源码对照冲突外，另15项因`toponav_bridge`未进入导入路径失败；以前全仓收集时，当前ROS测试的`sys.path`修改掩盖了这一依赖。原记录保留在`reports/test-scopes-review-historical-20261029-tlu96ouu`，没有覆盖。新增三个范围的隔离导入回归，在修复前3失败，修复后通过。
- 新审计记录了原有`reports/block7/7d-20261018/conftest.py`的忽略规则：`test_analysis.py`未被收集，同目录字节相同的`test_analysis_7d.py`已收集17项。两者SHA256均为`4399ead4b1c32672013b0739067a6a89bb519f585a40686d8ef62a10fc5a40dd`。**保留且披露限制，不增加哈希/文件名白名单，不修改历史配置。** 即使以后两个断言冲突解决，只要此限制存在，严格入口也不能报告完整通过。
- 补上“模块级跳过没有生成测试项”的审计缺口：`pytest.skip(..., allow_module_level=True)`与`importorskip`原本会在其他测试通过时被漏计。两项隔离回归修复前失败，现单列`collection_skips`并拒绝完整通过。环境中的ROS收集插件可能把模块跳过上报在其目录层级，因此记录实际collector node ID，不虚构未收集测试的数量。
- 入口回归测试现有44项；其中小型项目里的故意skip/xfail等仅用来测试审计拒绝机制，不是对项目失败添加skip/xfail。

## 最终验证

环境：项目`.venv`，Python 3.10.12、pytest 8.4.2。

| 实际运行范围 | 结果 | 入口退出码 | 完整性结论 |
| --- | --- | --- | --- |
| 新入口定向回归 | 44通过 | 0 | 工具回归通过 |
| current（独立运行） | 2254通过 | 0 | 仅当前范围完整通过 |
| historical（独立运行） | 172通过、2失败 | 1 | 失败，且披露原有忽略路径 |
| all（显式根目录） | 2426通过、2失败 | 1 | **全仓未通过**，不豁免历史失败/发现限制 |

最终三个范围的工件分别位于：

- `/home/sunrise/toponav_project/reports/test-scopes-verified-current-20261029-tlu96ouu`
- `/home/sunrise/toponav_project/reports/test-scopes-verified-historical-20261029-tlu96ouu`
- `/home/sunrise/toponav_project/reports/test-scopes-verified-all-20261029-tlu96ouu`

两项失败仍为`7d-navonly`与`7d-recovery`的live源码对照冲突。最终运行没有新增项目测试skip/xfail/deselect、收集错误或模块收集跳过；历史/全仓仍如实记录那条原有重复文件忽略规则。所有范围均为`formal_navigation_acceptance=false`。

开发期间名称含`review`、`final`的目录为中间运行，均保留；最终44项工具回归及模块跳过审计以`verified`目录为准。原始日志、哈希及旧文件保真核对见本轮开发报告中的`VERIFICATION.json`。

## 尚未解决

普通直走完成率与近场可观测性、unknown/遮挡误接纳及真实Gazebo连续C02绕行仍未解决；缺失的S2/S6原始证据不补造。测试报告的分层不能代替这些功能改进或正式验收。
