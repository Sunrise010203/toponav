"""PURE fixed-case differential classification, NOT a validator or permission.

Never discard a ledger, index, stage or value to manufacture equivalence.
Only explicitly registered unequal error labels at one fixed event may differ.
"""
import json

FALSE_FIELDS=('provenance_authenticated','exposure_age_bounded','shadow_admission_authorized',
    'native_live_observation_authorized','motion_authority','independent_enforcement_observed',
    'physical_stop_proved','production_guard_fixed','runtime_pending_installed','runtime_result_consumed',
    'runtime_one_time_consumption_enforced','cross_run_replay_prevented')
canonical=lambda x:json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False)


def valid_fact(f,mode):
    if mode not in ('schema','reference','trace'):return False
    if type(f) is not dict or set(f)!={'returned_result','consistent','result','error'}:return False
    if type(f['returned_result']) is not bool or type(f['consistent']) is not bool:return False
    if not f['returned_result']:return f['consistent'] is False and f['result'] is None and type(f['error']) is str and bool(f['error'])
    if f['error'] is not None or type(f['result']) is not dict:return False
    if mode=='schema':return f['consistent'] is True
    if mode not in ('reference','trace') or set(f['result'])!={'review','repeated_review'}:return False
    for review in f['result'].values():
        if review is None:
            if mode=='trace':return False
            continue
        fields=FALSE_FIELDS if mode=='trace' else FALSE_FIELDS[:10]
        if type(review) is not dict or review.get('actual_exposure_wall_s','missing') is not None or any(review.get(k) is not False for k in fields):return False
        if mode=='trace':
            expected={'consistent','disposition','event_count','violations','failures','submissions','classification','actual_exposure_wall_s',*fields}
            if set(review)!=expected or review['classification']!='caller_transcript_order_only':return False
            if any(type(review[k]) is not list for k in ('violations','failures','submissions')):return False
            if any(type(v) is not list or len(v)!=2 or type(v[0]) is not int or type(v[1]) is not str for v in review['violations']):return False
        else:
            expected={'classification','message_kind','synthetic_reference_matches','backend_claim','actual_exposure_wall_s',*fields}
            if set(review)!=expected or type(review['synthetic_reference_matches']) is not bool:return False
    if f['result']['review'] is None:return False
    if mode=='trace':
        return all(type(r.get('consistent')) is bool and r['consistent'] is f['consistent'] for r in f['result'].values())
    return f['consistent'] is True


def assess(mode,python,cpp,rule):
    relation='unregistered_difference';changes=[];ledger_equal=None
    safe=valid_fact(python,mode) and valid_fact(cpp,mode)
    if safe and mode=='trace' and python['returned_result'] and cpp['returned_result']:
        ledger_equal=all(canonical({k:r[k] for k in ('submissions','failures')})==canonical(
            {k:cpp['result'][name][k] for k in ('submissions','failures')}) for name,r in python['result'].items())
    exact=safe and canonical(python)==canonical(cpp)
    if exact:relation='exact'
    elif safe and not python['returned_result'] and not cpp['returned_result']:
        pair=[python['error'],cpp['error']]
        if pair==rule.get('error_pair') and pair[0]!=pair[1]:
            relation='refusal_error_only';changes=[dict(path=['error'],python=pair[0],cpp=pair[1])]
    elif safe and mode=='trace' and python['returned_result'] and cpp['returned_result']:
        left=json.loads(canonical(python));right=json.loads(canonical(cpp));allowed=rule.get('error_pair')
        ok=ledger_equal and python['consistent'] is cpp['consistent']
        for name in ('review','repeated_review'):
            a=left['result'][name]['violations'];b=right['result'][name]['violations']
            if len(a)!=len(b):ok=False;continue
            for index,(x,y) in enumerate(zip(a,b)):
                if canonical(x)==canonical(y):continue
                if len(x)!=2 or len(y)!=2 or type(x[0]) is not int or type(y[0]) is not int or x[0]!=y[0] or x[0]!=rule.get('violation_index') or [x[1],y[1]]!=allowed:
                    ok=False;continue
                changes.append(dict(path=['result',name,'violations',index,1],event_index=x[0],python=x[1],cpp=y[1]))
                x[1]=y[1]='REGISTERED_LABEL_DIFFERENCE'
        if ok and changes and canonical(left)==canonical(right):relation='trace_violation_codes_only'
    elif safe and mode=='reference' and python['returned_result'] and not cpp['returned_result']:
        r=python['result']['review']
        if rule.get('relation')=='reference_envelope_refusal' and cpp['error']==rule.get('cpp_error') and r['classification']==rule.get('python_classification') and r['message_kind']==rule.get('message_kind'):
            relation='reference_envelope_refusal'
            changes=[dict(path=['returned_result'],python=True,cpp=False),dict(path=['error'],python=None,cpp=cpp['error'])]
    # A registered difference is evidence of a limitation, not repaired parity.
    return dict(relation=relation,exact_equal=bool(exact),ledger_equal=ledger_equal,
        authority_safe=bool(safe),registered_expectation_matches=safe and relation==rule.get('relation'),changes=changes,
        classification='offline_difference_review_not_admission',actual_exposure_wall_s=None,
        **{k:False for k in FALSE_FIELDS})
