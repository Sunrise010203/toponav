# 显式离线多 scope 独立 RGB 摘要核对（任务 2I）

新入口 `tools/offline_scope_pixels.py` 是 opt-in 包装器；原 v1 index 和 2G reader 字节及默认语义不改。
仅 stdlib 和原纯数据契约/reader；不解码图像、不接受图像路径、不构造运行时 IO/探针/控制器、不使用时钟/线程/回调。

```sh
.venv/bin/python -m tools.offline_scope_pixels --root ROOT \
  --manifest INDEX --manifest-sha256 INDEPENDENT_INDEX_SHA256 \
  --payload PIXELS --payload-sha256 INDEPENDENT_PAYLOAD_SHA256
```

独立文件（与 index 同样 ≤256KiB，逐级 NOFOLLOW、regular-only、双读；要求静止闭合文件）：

```json
{"format":"offline_scope_pixels_v1","index_sha256":"<exact index bytes sha256>",
 "scopes":[{"session_id":"…","scope_id":"…","external_inputs":null}]}
```

`scopes` 必须与 index 展平顺序、数量、session/scope 身份完全对应，不得缺项/重复/插入或混入路径。
无独立材料显式为 `null`；有材料则为 1—256 项完整输入列表，格式等同 2G `external_inputs`：
每项只有 `expected` 和 `composite_sha256`；expected 只有 `frame_id`、非负整数 `captured_sim_ns`、
`view_digests`（完整相机顺序 `[view,sha256]` 列表）。frame_id 不重复，摘要必须有效。
原 2G 负责核对完整保存输入集合、每分路摘要、合成摘要及身份；不能仅传成功子集。
所有 schema 完成后才开始额外的纯数据子审查。frame 列表顺序可不同，view 顺序必须相同。

调用者必须通过独立渠道持有 payload 和 index 的确切字节摘要；同目录本地生成 pins 只是 fixture，**不是认证**。
读取器不从 index 自动寻找像素、也不从 `inputs.json` 复制摘要当独立输入。
PNG 解码只在显式 cases driver：公共三 case 各前4帧，256×256 RGB，
四个128×128分块按 front/left/right/back 顺序独立计算像素摘要；合成 RGB 同时摘要。
原 decisions 仅定位 input 及核对身份，不作为控制输入、不重跑 controller.update。

## 结果语义

- `base` 原样保留完整 v1 结果（包括全部声明子项、日志不完整和原 body/worker/cleanup/诊断故障）。
- `entries` 对每个可安全解析的 base 子项保留一个结果；payload 非法也不抹掉 base 子项。
- 未给 payload：`not_compared_not_supplied`；显式 null：`not_compared_no_material`。
  supplied payload/schema 无效：`not_compared_invalid_payload`；均为 `external_inputs_match=null`。
- 实际调用 2G 后为 `matched_declared_inputs` 或 `comparison_refused`；失败时仍保留 2G 的原始
  classification、diagnostics、external_inputs_match（常为 null），**不能捏造成 false/mismatch**。
- `compared_scopes`/`matched_scopes` 只是局部计数。`pixel_coverage=none/partial/all`，
  `all_declared_inputs_match` 只有全父结果一致、全部 scope 有独立材料并成功核对时才 true，其余 null。
- payload 缺省/null 不制造拒绝，但也不制造像素通过；`evidence_consistent` 仅表示声明诊断一致，
  不要求全像素覆盖，可能包含原 recorded_failure。必须同时查看 base、issues、coverage 和每个 scope。
- 任意 base 失败、payload 错误或提供材料的 scope 核对失败，父结果拒绝；不得提升成功子集。
- index/payload/子文件在审查后重读绑定；发现改变保留已有结果、父拒绝，不宣称原子快照。
- stdout JSON；退出0仅表示上述诊断一致，不是曝光/场景完成/运动放行。

所有层曝光恒 null，来源认证、shadow admission、native live、motion 四授权恒 false。
公开 PNG 重读只能证明这些公开字节与声明对应，不能证明真实曝光、设备时钟、可信采集或原生 token。
共同重写 session/status、两层 pins 和独立声明仍可局部一致；重复同一输入也可一致。
没有跨运行状态、防重放 owner、来源认证、完整会话档案、敌对文件系统/崩溃耐久或实时上界保证。
