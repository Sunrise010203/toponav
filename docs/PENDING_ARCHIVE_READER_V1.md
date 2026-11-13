# 2X：隔离只读档案读取器 v1（Linux opt-in）

入口 `tools.pending_archive_reader.read_archive_pair(absolute_root, claim)`，新profile
`toponav_pending_archive_reader_v1`。claim仍是2W六字段声明，**不改2W封存代码/错误/schema**。
旧profile的20个权限/来源/执行标志仍全false，包括旧`archive_filesystem_read_observed`；
本轮另用`local_regular_file_reads_complete`记录局部顺序读取事实，不回写旧结果。

## 实际读取边界

只接收精确str绝对root（最多4096字符/64层，无空分量、`.`/`..`/NUL/代理码点），
claim先闭合形状、512字符ASCII引用检查，新增读取器最多32层；互异的三个引用均相对root。
逐级`openat(O_DIRECTORY|O_NOFOLLOW)`，保留目录描述符而不是拼接后realpath再打开。
叶子以Linux `O_PATH|O_NOFOLLOW`固定inode，fstat确认为普通文件且尺寸不过界，
**才**通过固定的`/proc/<本进程pid>/fd/<固定描述符号>`取得只读描述符。
这里有意跟随Linux内核描述符magic-link，不是跟随调用者文件名的符号链接；
proc目录逐级NOFOLLOW，无普通open fallback。依赖正常Linux内核及挂载的/proc，
不宣称可对抗恶意OS/proc挂载。FIFO/目录/符号链接在获得可读fd前拒绝，不测试真实设备。

manifest/raw/observation上限沿用2W的4MiB/36893/65536字节。
实际读取最多cap+1以发现增长；每文件比较dev/inode/mode/size/mtime_ns/ctime_ns/nlink，
并核对读到的长度。每条已固定路径边核对当前目录项身份，叶子fd在2W纯检查前后均再检查。
所有自有描述符在finally关闭，无描述符/原文件字节返回，无文件创建/写入/网络/时钟/睡眠。
close出错也拒绝且不重试close，防止已释放fd复用。IO错误只输出稳定错误类别/role/errno。

## 结论限度

`reader_reference_checked`意味着三个局部普通文件样本读取、采样检查和2W引用检查均通过。
`reader_reference_refused`意味着局部读完但2W纯契约拒绝。
`reader_refused`意味着读取/采样/关闭失败；即使保留的nested reference_result相符，也不能
越过顶层拒绝。`final_recheck_agreed=null`表示最后一轮并未完成该检查，不伪造为成功。

**顺序采样不是原子三文件快照、文件锁、可信来源、一次性消费或运行时准入。**
目录只比较身份/mode，不把目录mtime变化当安全证明；未排除硬链接、bind mount、
检查间替换后恢复、恶意内核或阻塞的普通文件系统。O_NONBLOCK不提供硬实时IO时限。
root/manifest pin/配对仍由调用者选定。硬链接配对、整组重pin、重复调用仍可相符，
2W的opaque observation完全不解析，不升级为语义合法或旧实际执行位置观测。

## 验证

532原raw及1064个原Python/C++来源配对只读；origin=cpp不是执行新C++/真实后端。
正常配对使用2V封存的8914项目录。2W总交付清单12266项超过旧契约12000项上限，
另保留为真实`manifest_shape`拒绝案例，绝不放宽2W。首轮错误地期望该大目录通过的
1064项开发失败全部保留；修正的是新语料目录选择/期望，不是旧契约或读取器逻辑。
另在TemporaryDirectory创建封闭文件系统夹具：缺失、符号链接、FIFO、目录、尺寸界限、
硬链接、错误pin与摘要、读中修改/检查后替换/祖先或root替换等受控交错。
瞬时替换祖先并在再次采样前恢复可不被发现，作为采样局限保留，不伪造原子性。
夹具设置/交错仅写临时目录；真实归档及生产目录从不改写。
生产模块不接受注入回调；测试/审计在进程内临时包装受控函数，保留交错phase记录。
临时夹具重建会改变inode/时间，因此只在重复比较时排除这两组7项元数据；
哈希/长度/阶段/错误/权限/最终检查及交错声明全部仍须一致，原归档输出则逐字段重复一致。

16旧位置未知、66错误差异、旧四包装器差异、拒绝/整组替换/重复调用/unknown迟到不升级、
2L原两端晚重签不变且不重跑/重签。不冒充2B能力。
2M14需求未实现、25运行时场景未执行；无runtime owner/permit/token/可信poll。
不接生产native/Gazebo/设备/真实运动，不改0.8s/0.4s/门控/权限/速度/制动/恢复。
无物理停车证明，普通直走验收仍未完成。
