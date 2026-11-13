"""PURE closed-observation checker. Not a TPT1 validator or runtime authority.

It checks a registered fixed-case expectation and preserves every error value;
there is no rule to ignore arbitrary differences or to upgrade an old outcome.
"""
import json

FALSE_FIELDS=('provenance_authenticated','exposure_age_bounded','shadow_admission_authorized',
    'native_live_observation_authorized','motion_authority','independent_enforcement_observed',
    'physical_stop_proved','production_guard_fixed','runtime_pending_installed','runtime_result_consumed',
    'runtime_one_time_consumption_enforced','cross_run_replay_prevented')
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False,ensure_ascii=True)


def valid(doc):
    if type(doc) is not dict or set(doc)!={'profile','classification','domain_accepted','stage','consistent','result','error','actual_exposure_wall_s',*FALSE_FIELDS}:return False
    if doc['profile']!='toponav_pending_common_domain_v1' or doc['classification']!='offline_common_domain_not_admission':return False
    stage=doc['stage']
    if stage not in ('domain_refused','review_refused','review_returned'):return False
    if doc['domain_accepted'] is not (stage!='domain_refused') or doc['consistent'] is not (stage=='review_returned'):return False
    if doc['actual_exposure_wall_s'] is not None or any(doc[k] is not False for k in FALSE_FIELDS):return False
    if stage!='review_returned':return doc['result'] is None and type(doc['error']) is str and bool(doc['error'])
    result=doc['result']
    if doc['error'] is not None or type(result) is not dict or set(result)!={'review','repeated_review'} or result['review'] is None:return False
    for r in result.values():
        if r is None:continue
        if type(r) is not dict or set(r)!={'classification','message_kind','synthetic_reference_matches','backend_claim','actual_exposure_wall_s',*FALSE_FIELDS[:10]}:return False
        if r['classification'] not in ('decoded_schema_not_admission','frame_free_stop_schema_only','caller_snapshot_reference_only','declared_refusal_not_execution','synthetic_reply_claim_only'):return False
        if r['message_kind'] not in ('reserve','seal','result_ref','stop','ack','status') or type(r['synthetic_reference_matches']) is not bool:return False
        if r['backend_claim'] not in (None,'python','native'):return False
        if r['actual_exposure_wall_s'] is not None or any(r[k] is not False for k in FALSE_FIELDS[:10]):return False
    return True


def assess(expectation,python,cpp,python_repeat,cpp_repeat):
    safe=type(expectation) is dict and all(valid(d) for d in (python,cpp,python_repeat,cpp_repeat));match=False;exact=False;stable=False
    relation='unexpected_observation'
    if safe:
        exact=canonical(python)==canonical(cpp)
        stable=canonical(python)==canonical(python_repeat) and canonical(cpp)==canonical(cpp_repeat)
        stage=expectation.get('expected_stage');pair=expectation.get('error_pair')
        stage_ok=python['stage']==cpp['stage']==stage
        error_ok=pair is None and stage=='review_returned' or type(pair) is list and len(pair)==2 and [python['error'],cpp['error']]==pair
        labels_only=(not exact and stage=='review_refused' and type(pair) is list and len(pair)==2 and pair[0]!=pair[1]
            and canonical({k:v for k,v in python.items() if k!='error'})==canonical({k:v for k,v in cpp.items() if k!='error'}))
        match=bool(stage_ok and error_ok and stable and (exact or labels_only))
        if match:relation='exact' if exact else 'registered_inner_error_difference'
    return dict(expectation_matches=match,authority_safe=safe,exact_equal=exact,repeated_calls_equal=stable,relation=relation,
        classification='offline_boundary_observation_not_admission',actual_exposure_wall_s=None,**{k:False for k in FALSE_FIELDS})
