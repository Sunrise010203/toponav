# 项目整理与发布状态：2026-11-13

## 整理范围

原工作区没有 `.git`，本次初始化 `main` 分支。保留既有目录与业务代码，不重构算法、不修改冻结实验、不升级依赖、不改变测试发现规则，也不删除本地大文件。

新增项目总览与本记录，为 README 增加当前入口；旧开发记录继续保留。

### 纳入 Git

- 源码、脚本、配置、测试及测试夹具。
- 文档、当前实际存在的 `reports/` 证据与历史候选源码快照（包括失败记录）。
- 模拟数据集与 `models/road_structure_frozen_v1_20261014/` 的现有模型/评估材料。旧 `.gitignore` 忽略所有模型，本次仅开放这一已冻结版本。
- 本次验证产生的命令、日志和摘要。

### 仅保留本地

`.venv/`、`.cache/`、`.simulators/`、Python/pytest 缓存、ROS `build*`/`install*`/`log`、编辑器配置、私密文件及未来训练输出。具体规则见根目录 `.gitignore`。

本地原目录约 2.4GB；其中虚拟环境约 1.7GB、仿真器约 327MB。已有报告约 323MiB，应作为证据保留，不为了缩小仓库删掉或隐藏失败。
`.gitattributes` 禁止自动换行转换，以免克隆时破坏冻结材料的字节级哈希。当前没有为项目擅自指定开源许可证。

本次暂存 4622 个文件，约 342MiB。常见凭据模式扫描无命中，没有超过 50MiB 的候选文件、ELF 二进制或符号链接；这不是全面安全审计，详见[扫描记录](../reports/repository-preparation-20261113/publish-scan.json)。新增文档链接及上传排除边界检查通过。全体初始文件的 `git diff --cached --check` 返回 2，包含旧文档 CRLF/尾随空白等历史格式问题；未为了消除警告批量改写冻结材料。新增总览、状态文档及 Git 配置文件的单独空白检查通过。

## 本次实测

以下标准命令未添加 ignore、skip、xfail 或测试配置：

| 范围 | 命令 | 结果 |
| --- | --- | --- |
| current | `.venv/bin/python tools/run_project_tests.py --scope current --output reports/project整理-20261113-current` | exit 2；selected 0；收集失败 |
| all | `.venv/bin/python tools/run_project_tests.py --output reports/project整理-20261113-all` | exit 2；selected 96 但未运行；收集失败 |

对应完整工件：[current 摘要](../reports/project整理-20261113-current/summary.json)、[current 日志](../reports/project整理-20261113-current/pytest.log)、[all 摘要](../reports/project整理-20261113-all/summary.json)、[all 日志](../reports/project整理-20261113-all/pytest.log)。

主要阻塞：

1. 当前测试收集引用缺失的 `reports/straight-followup-20261103-31/DELIVERY_MANIFEST.json`；该问题在 2026-11-08 报告中已披露，不是本次整理新增。
2. 全仓递归同时发现 `reports/` 内的历史候选测试；其模块名与当前测试/其他历史候选冲突，出现 `ImportPathMismatchError`。
3. 历史候选还存在相对推导后的输入路径缺失，例如 `reports/straight-project-allocation-20261104/candidate/reports/straight-project-allocation-20261104/before/roadvision/corridor_rgb_probe.py`。

本次未补造上述文件、未修改历史断言、未添加排除规则。两种标准范围的 `verification_complete`、`all_scopes_passed`、`formal_navigation_acceptance` 均为 false。

独立最小检查记录在 [smoke-results.json](../reports/repository-preparation-20261113/smoke-results.json)：

- 三个 CLI（topology、dijkstra、structural_navigation）均 exit 0。
- 显式选取五个拓扑/规划/导航测试文件的第一次尝试，被系统 ROS `launch_testing` 插件触发的目录级导入阻塞，exit 4；[原失败日志](../reports/repository-preparation-20261113/smoke-4.log)保留。
- 仅对补充 smoke 设置 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`，同样五个测试文件 **529 passed、18 warnings、exit 0**；[补充日志](../reports/repository-preparation-20261113/smoke-unit-no-autoload.log)保留。该设置只作用于这一个子进程，未改变标准 current/all 命令、生产代码或测试配置。

小范围检查不替代标准 current/all 回归，也不替代物理或正式导航验收。

## 后续优先级

1. 从真实归档恢复缺失历史输入和清单，并逐项校验原哈希；无法恢复的条目继续明确标记缺失。
2. 单独研究历史候选的隔离执行环境和模块命名冲突，不通过隐藏报告来换取“全仓绿灯”。
3. 按原门槛继续 S2/S6 与匹配结构复用研究，不将整理上传当成能力或安全验收。

## GitHub 首次提交前核验

以下记录首次提交前的检查；实际发布结果以 GitHub `main` 分支及远程提交核对为准。

- 目标仓库：`https://github.com/Sunrise010203/toponav.git`，已配置为 `origin`。
- 提交身份由用户指定为 `sunrise <3052975498@qq.com>`，仅在当前仓库配置。
- 初始匿名 API 查询返回 404；完成 GitHub CLI 浏览器授权后，已确认这是私有仓库，登录账号 `Sunrise010203` 具有 push 权限。
- 认证后的 `git ls-remote --symref origin` 成功返回空引用，仓库 API 的 size 为 0；首次提交前没有既有远程分支需要合并。
- 本机 Clash Verge 混合端口为 7897，经用户在宿主机建立的 SSH 反向转发映射至虚拟机 `127.0.0.1:17890`。代理连接已验证；匿名 API 曾因共享出口额度耗尽返回 403，认证后仓库查询成功。
- Git 代理与凭据助手只写入本仓库的本地 `.git/config`；GitHub CLI 请求通过进程环境使用代理。不把凭据、用户授权码或本机代理配置纳入版本控制。后续 Git 网络操作仍需保持代理隧道可用。

本次使用普通首次推送，不强推覆盖历史；推送后核对本地 HEAD 与远程 `refs/heads/main` 一致，再确认发布完成。
