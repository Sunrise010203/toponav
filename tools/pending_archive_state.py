"""Pure, invocation-local OFFLINE trace model. Never a runtime latch or owner.

Caller labels, input JSON and hashes are unauthenticated. Fresh invocations
rebuild empty state; duplicates are modeled, not consumed or blocked at runtime.
"""
import hashlib
import json
import re
from tools import pending_archive_decision as decision

PROFILE = 'toponav_pending_archive_state_v1'
TRACE_LIMIT = 2 * 1024 * 1024
DECISION_LIMIT = 16384
EVENT_LIMIT = 256
SLOT_LIMIT = 128
FALSE_FIELDS = decision.FALSE_FIELDS
EXTRA_FALSE = (*decision.EXTRA_FALSE, 'caller_scope_authenticated', 'persistent_ledger_installed',
    'cross_process_owner_enforced', 'runtime_terminal_latch_installed')
DECISION_KEYS = set(('profile stage error offline_reference_review_ready diagnostic_only stateful_consumption '
    'decision_basis input_record_sha256 reader_stage reader_error reader_local_reads_complete expected_claim_snapshot '
    'sampled_artifact_sha256 observed_validation_site actual_exposure_wall_s').split()) | set(FALSE_FIELDS) | set(decision.EXTRA_FALSE)
sha = lambda b: hashlib.sha256(b).hexdigest()


def _pairs(pairs):
    result = {}
    for k, v in pairs:
        if k in result: raise ValueError('duplicate')
        result[k] = v
    return result


def _number(v):
    raise ValueError('noninteger_json_number')


def _load(raw):
    return json.loads(raw, object_pairs_hook=_pairs, parse_constant=_number, parse_float=_number)


def _label(v, limit):
    return type(v) is str and 0 < len(v) <= limit and v not in ('.', '..') and re.fullmatch('[A-Za-z0-9._+-]+', v) is not None


def _classify(text):
    result = dict(phase='local_unknown', decision_sha256=None, decision_stage=None, decision_error=None,
        reader_stage=None, reader_error=None, payload_error=None, source_input_record_sha256=None)
    if text is None: return result
    result['decision_sha256'] = sha(text.encode('utf-8'))
    def refuse(error):
        result.update(phase='local_refused', payload_error=error)
        return result
    try: value = _load(text)
    except (ValueError, RecursionError): return refuse('state_decision_json')
    if type(value) is not dict or value.get('profile') != decision.PROFILE: return refuse('state_decision_shape')
    for k in ('decision_stage', 'decision_error', 'reader_stage', 'reader_error'):
        source = k.removeprefix('decision_') if k.startswith('decision_') else k
        if type(value.get(source)) is str: result[k] = value[source]
    if decision._digest(value.get('input_record_sha256')):
        result['source_input_record_sha256'] = value['input_record_sha256']
    # Refusal/unknown are terminal model facts, not a route to nested readiness.
    if value.get('stage') == 'decision_refused':
        result['phase'] = 'local_refused'; return result
    if value.get('stage') != 'offline_reference_consistent': return result
    if set(value) != DECISION_KEYS: return refuse('state_decision_shape')
    if (value['error'] is not None or value['offline_reference_review_ready'] is not True
            or value['diagnostic_only'] is not True or value['stateful_consumption'] is not False
            or value['decision_basis'] != 'caller_supplied_record_and_claim_not_trust_anchor'
            or value['reader_stage'] != 'projection_reader_checked' or value['reader_error'] is not None
            or value['reader_local_reads_complete'] is not True
            or value['observed_validation_site'] is not None or value['actual_exposure_wall_s'] is not None
            or any(value[k] is not False for k in (*FALSE_FIELDS, *decision.EXTRA_FALSE))
            or not decision._digest(value['input_record_sha256'])):
        return refuse('state_decision_state')
    c = decision._claim(value['expected_claim_snapshot']); hashes = value['sampled_artifact_sha256']
    if (c is None or type(hashes) is not dict or set(hashes) != {'parent', 'projection', 'raw', 'observation'}
            or any(not decision._digest(h) for h in hashes.values())
            or hashes['parent'] != c['parent_manifest_sha256'] or hashes['projection'] != c['projection_sha256']):
        return refuse('state_decision_claim')
    result['phase'] = 'local_recorded'
    return result


def review_state_trace(trace_raw):
    """Fold bounded caller-local labels. No live actions, persistence or clock.

    Full envelope validation precedes folding: a bad suffix yields NO partial
    ledger. First refused/unknown never upgrades. Changed bytes after recorded
    close the slot as local_conflict; that conflict never reopens. Equality is
    exact original text/null, NOT canonical JSON or digest equality alone.
    """
    out = dict(profile=PROFILE, stage='state_trace_refused', error=None, model_only=True,
        input_trace_sha256=None, caller_scope_label=None, scope_basis='caller_declared_not_owner',
        local_model_review_complete=False, fresh_invocation_starts_empty=True, slots=[], events=[],
        counts=dict(local_recorded=0, local_refused=0, local_unknown=0, local_conflict=0),
        observed_validation_site=None, actual_exposure_wall_s=None,
        **{k: False for k in (*FALSE_FIELDS, *EXTRA_FALSE)})
    def refuse(error): out['error'] = error; return out
    if type(trace_raw) is not bytes: return refuse('state_trace_bytes')
    if len(trace_raw) > TRACE_LIMIT: return refuse('state_trace_byte_bound')
    out['input_trace_sha256'] = sha(trace_raw)
    try: trace = _load(trace_raw.decode('utf-8'))
    except (ValueError, RecursionError): return refuse('state_trace_json')
    if type(trace) is not dict or set(trace) != {'profile', 'scope', 'events'} or trace['profile'] != PROFILE:
        return refuse('state_trace_shape')
    if not _label(trace['scope'], 64): return refuse('state_scope_shape')
    events = trace['events']
    if type(events) is not list or len(events) > EVENT_LIMIT: return refuse('state_event_bound')
    labels = set()
    for event in events:
        if type(event) is not dict or set(event) != {'slot', 'decision_utf8'} or not _label(event['slot'], 128):
            return refuse('state_event_shape')
        text = event['decision_utf8']
        if text is not None:
            if type(text) is not str: return refuse('state_decision_text')
            try: raw = text.encode('utf-8')
            except UnicodeEncodeError: return refuse('state_decision_utf8')
            if len(raw) > DECISION_LIMIT: return refuse('state_decision_byte_bound')
        labels.add(event['slot'])
        if len(labels) > SLOT_LIMIT: return refuse('state_slot_bound')
    states, first_texts = {}, {}
    for index, event in enumerate(events):
        name, text = event['slot'], event['decision_utf8']; incoming = _classify(text)
        before = states[name]['phase'] if name in states else None
        if name not in states:
            first_texts[name] = text
            states[name] = dict(incoming, slot=name, initial_phase=incoming['phase'], first_event_index=index,
                duplicate_count=0, changed_bytes_count=0)
            action = 'first_local_registration'
        else:
            state = states[name]; same = text == first_texts[name]
            state['duplicate_count' if same else 'changed_bytes_count'] += 1
            if before == 'local_conflict': action = 'conflict_terminal_retained'
            elif same: action = 'duplicate_unchanged'
            elif before in ('local_refused', 'local_unknown'): action = 'terminal_not_upgraded'
            else:
                state['phase'] = 'local_conflict'; action = 'changed_bytes_closed_slot'
        out['events'].append(dict(index=index, slot=name, action=action, before_phase=before,
            after_phase=states[name]['phase'], decision_sha256=incoming['decision_sha256'],
            incoming_phase=incoming['phase'], incoming_decision_stage=incoming['decision_stage'],
            incoming_decision_error=incoming['decision_error'], incoming_reader_error=incoming['reader_error'],
            incoming_payload_error=incoming['payload_error']))
    out['slots'] = list(states.values())
    for state in out['slots']: out['counts'][state['phase']] += 1
    out.update(stage='state_trace_reviewed', local_model_review_complete=True, caller_scope_label=trace['scope'])
    return out
