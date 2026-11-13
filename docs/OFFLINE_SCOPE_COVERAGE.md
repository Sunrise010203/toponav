# 显式离线像素覆盖需求（任务2J）

新增 opt-in `tools/offline_scope_coverage.py`。v1/2I 默认语义及文件字节不改；不接生产控制，不是实时安全门控。

```sh
.venv/bin/python -m tools.offline_scope_coverage --root ROOT \
  --manifest INDEX --manifest-sha256 INDEX_SHA256 \
  --payload PIXELS --payload-sha256 PIXELS_SHA256 \
  --requirements REQUIREMENTS --requirements-sha256 REQUIREMENTS_SHA256
```

独立冻结需求格式：
```json
{"format":"offline_scope_coverage_v1","index_sha256":"<exact index byte SHA256>",
 "payload_sha256":"<exact payload byte SHA256>",
 "scopes":[{"session_id":"…","scope_id":"…","required":true,"expected_input_count":4}]}
```

- 需求文件必须由调用者显式选择并提供独立持有的字节摘要。不得自动从匹配成功项生成需求。
- scopes 必须与 index 展平顺序及全部 session/scope 身份精确对应，不能删项/重复/倒序；无路径或动态默认字段。
- `required` 必须是严格bool；true要求独立声明完整输入数量1—256（非bool/浮点），false要求count=null。
- 至少一个required=true，禁止空需求/零帧制造真空成功。数量要求只限制诊断覆盖，不是曝光时长要求。
- 省略 payload 时命令须同时省略其摘要，需求中的payload_sha256为null；所有必核对项因缺材料拒绝。
- 每个必核对项必须有非null独立材料、2I完整核对成功且数量精确等于要求；2G原有完整输入集/相机顺序/分路与合成摘要核对不变。
- 可选缺口显式保留not_required（locally_met=null），不计入已满足数量。成功仅满足所声明需求，不表示所有scope像素覆盖。
- 即使所有必核对项局部匹配，任意原index/2I失败（包括非必核对项提供了无效材料）都会让父requirements_satisfied=false。

## 结果与限制

完整原2I结果保留在pixels中，原v1及全部失败在pixels.base中；包括每项原body/worker/cleanup/log/diagnostic故障和原分类。
非法需求仍保留全部安全可解析的原子项，在新entries中为not_evaluated_invalid_requirements，required/count/locally_met均null。
合法需求为每scope保留required、expected_input_count、来自绑定独立payload的independent_input_count和状态：
not_required / required_material_missing / required_comparison_refused / required_input_count_mismatch / matched_required_inputs。
locally_met_scopes只计局部匹配；必核对材料缺失时locally_met=false，但底层external_inputs_match仍保留原null，不伪造像素不匹配。
requirements_satisfied/evidence_consistent只有整个父结果一致且所有非空必核对项满足时为true。
原recorded_failure仍可能满足离线覆盖要求；**这不是场景成功，也不是将故障洗白。**
CLI stdout JSON，退出0仅表示显式声明需求被满足；否则退出2，不自动改写输入或缩小需求。

仅stdlib和原纯数据reader；无PIL/图像路径/IO或controller构造、当前时间、线程、callback、owner注册或原生token。
需求/index/payload采用同样≤256KiB的regular-only、逐级NOFOLLOW双读及末尾重读。要求静止闭合文件。
重复读取不是原子快照、敌对文件系统防护或崩溃耐久保证，没有实时上界。

各层曝光恒null，provenance/shadow/live/motion授权恒false。独立pins只约束特定字节，不认证需求制定者/设备/像素。
固定需求pin能发现把required偷偷改成false；若需求、索引和外部pins共同重写/缩窄，则仍可能局部一致。
只声明窗口无法恢复被共同删除的scope；重复同包依旧可满足，没有跨运行防重放状态。
公共RGB仍只由显式cases driver独立解码原三组各前4帧，绝不从保存inputs声明复制摘要当独立来源。
