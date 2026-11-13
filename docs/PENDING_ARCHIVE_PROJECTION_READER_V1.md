# 2Z：分层目录四文件只读加载（独立 opt-in）

`tools.pending_archive_projection_reader.read_projection_archive(root, claim)`
接收绝对调用者根和2Y八字段claim；依次读取父清单、最小目录、raw、observation，再调用未修改2Y纯契约。
没有CLI生产入口、运行时控制器、native、Gazebo、设备或运动接入。

复用封存2X底层描述符函数，不调用2X三文件reader。所有调用者路径逐级NOFOLLOW；根4096字符/64层，引用512 ASCII字符/32层，不允许空/dot/dotdot/绝对引用/别名。
Linux O_PATH先固定叶inode，确认普通文件和大小后才通过固定/proc/<pid>/fd下数字描述符重新打开；这是唯一有意跟随的内核magic-link，不是调用者symlink。缺少平台能力或proc即拒绝，无降级。
父8MiB/最小目录8192/raw36893/observation65536字节上限，cap+1读取防增长。没有硬实时IO保证。
先后fstat与路径边检查；调用2Y前后均复核所有仍持有的描述符。finally反序关闭，无close重试。
顶层projection_reader_refused覆盖嵌套positive；local_regular_file_reads_complete仅代表四文件读取与采样复核完成，不能用嵌套positive绕过顶层拒绝。
继承旧错误标签与所有旧20false权限标志，新增边界标志亦false，曝光/旧执行位置null。

**四文件顺序样本不是原子快照、来源认证、输入绑定、权限或租约。** 路径/元数据采样可发现持续替换与读中变化，但不能排除硬链接、bind mount、祖先瞬时替换恢复、恶意OS/proc。多次调用不提供一次性消费。
调用者可以替换整组文件并重pin；不相关封存raw/observation以及opaque/非JSON/空observation仍可能相符；不执行观测语义/旧validator。派生目录自身不要求是父清单封存成员。
2Y原synthetic derivative引用不冒充物理路径，本轮baseline只重绑定引用名至其真实封存content-addressed blob，pin/字节不变。
旧2W 12000项上限、12266项真实拒绝、2X首轮1064开发失败以及旧16未知位置/66错误差异/四包装器差异/2L晚重签不改变。
2M14需求/25场景仍未实现/未执行；普通直走验收未完成。现有2B能力不记为新增。
