"""Opt-in read-only integration DESIGN structure/source-text review, NOT a gate.

No guard/controller/owner/native runtime imports or construction. The plan is
non-executable: tests are future requirements, not results of new experiments.
Local pins and lexical anchors cannot prove trust, control flow or enforcement.
"""
import argparse
import ast
import copy
import hashlib
import json
from pathlib import Path

from tools.offline_scope_index import read_index
from tools.offline_scope_review import parse
from tools.precompute_deadline_contract import check, shape

DESIGN='docs/PRECOMPUTE_INTEGRATION_REVIEW.json'
DOCUMENT='docs/PRECOMPUTE_INTEGRATION_REVIEW.md'
G='simulation/gazebo_cross_validation.py'
A='simulation/gazebo_actuator.py'
P='ros2_ws/src/toponav_bridge/toponav_bridge/protocol.py'
R='simulation/gazebo_receiver.py'
N='simulation/native/'
OLD='reports/straight-followup-20261102-21/work-budget-audit/'
ANCHORS={
 'ready_return':(G,'GazeboIO','next_frame','return frame[0], frame[1], self.now_ns / 1e9'),
 'main_png':(G,None,'run','PNG.fromarray(rgb).save(output / "public" / relative)'),
 'main_update':(G,None,'run','d = controller.update(frame, now)'),
 'main_arm':(G,None,'run','gate.arm(frame["frame_id"], frame["captured_sim_s"], wall)'),
 'main_context':(G,None,'run','issued_steady_s=issued_wall, expires_steady_s=gate.hold_deadline'),
 'failure_zero':(G,None,'_failure_stop','io.command(0., 0.)'),
 'failure_cancel':(G,None,'_failure_stop','controller.cancel(io.now_ns / 1e9 if io is not None else 0.)'),
 'interrupt_cancel':(G,None,'run','controller.cancel(io.now_ns / 1e9 if io else 0.)'),
 'guard_short_expires':(A,'LeaseGuard','handle','g.hold_deadline = min(expires, issued+LEASE_S)'),
 'client_ack_expiry':(A,'ActuatorClient','_validate_ack',"if not latched and received >= expected['deadline_steady_s']:"),
 'client_post_send_ledger':(A,'ActuatorClient','request',"observer(decode(raw.decode('utf-8')), sent)"),
 'observer_strict_fields':(R,'ReceiverEvidence','record_submission',"if set(packet) != base | {'command', 'sim_s', 'issued_steady_s', 'expires_steady_s'}:"),
 'gate_relative_pending':(P,'MotorGate','arm','self.deadline=wall+self.lease_s'),
 'ros_existing_pending':('ros2_ws/src/toponav_bridge/toponav_bridge/nodes.py','World','frame',"if not terminal:self.gate.arm(f['frame_id'],self.sim_s,time.monotonic())"),
 'native_seq_poll':(N+'receiver_lease.hh',None,None,'if (!latched && seq && now >= deadline) Trip("command_lease_expired");'),
 'native_short_expires':(N+'receiver_lease.hh',None,None,'deadline = std::min(p.expires, p.issued+.8);'),
 'wire_exact_eight':(N+'receiver_envelope.hh',None,None,'if (msg.header().data_size() != 8'),
 'native_status':(N+'receiver_status.hh',None,None,'<< ",\\\"deadline_steady_s\\\":" << guard.deadline'),
 'native_status_dedupe':(N+'gazebo_receiver_interlock.cc',None,None,'if (sim == last_sim_ && guard_->seq == last_seq_ && guard_->reason == last_reason_) return;'),
 'prior_exposure_shadow':('tools/frame_deadline_proposal.py','DeadlineProposal','admit','deadline = min(lower_bounds)+LEASE_S'),
}
SOURCE_FILES=tuple(sorted({v[0] for v in ANCHORS.values()}))
EVIDENCE_FILES=('reports/straight-followup-20261102-20/precompute-audit/source-audit.json',OLD+'summary.json')+tuple(
    OLD+name+'/'+file for name in ('late_joint_resign_counterexample','owner_stall_before_send','owner_stall_no_poll')
    for file in ('input.json','result.json'))
INPUT_FILES=SOURCE_FILES+EVIDENCE_FILES+(DESIGN,DOCUMENT)
STEPS=('freeze_origin','reserve_both','reserve_ack','hash_all_views','seal_binding_both','seal_ack',
    'save_png','controller_update','poll_and_validate_result','issue_original_deadline','consume_both','observe_ack')
REQUIREMENTS=('origin','reservation','barrier','binding','result','replacement','expiry','stop','failure',
    'wire','status','ledger','capacity','exposure')
TRANSITIONS=('first_reserve','subsequent_reserve','seal_once','timely_result','old_hold_due','new_pending_due',
    'stop_without_frame','normal_zero','lost_ack','cancel','unknown_origin','restart')
TESTS=('first_timely','first_slow','exact_boundary','late_registration','reserve_ack_missing','reserve_ack_wrong',
    'hash_slow','seal_rewrite','view_subset','result_swap','old_hold_first','timely_replace','joint_resign',
    'duplicate_result','zero_hold','terminal_without_frame','png_failure','cancel_before_compute',
    'ack_after_send','status_pending_change','capacity','owner_stall','clock_regression','restart','unknown_exposure')
FALSE_FIELDS=('implementation_ready','production_guard_fixed','physical_stop_proved','exposure_age_bounded',
    'provenance_authenticated','shadow_admission_authorized','native_live_observation_authorized','motion_authority',
    'independent_enforcement_observed','production_changes','native_transport','movement')
BLOCKERS=('trusted_origin_missing','downstream_pending_missing','authenticated_binding_missing',
    'versioned_wire_and_status_missing','runtime_poll_liveness_unproved','exposure_provider_missing')
INVALID=('authority','status','limits','exposure_fabricated','missing_origin','result_sets_deadline',
    'hash_before_reserve','compute_before_seal','wrong_ack_kind','fallback','record_deadline_mutable',
    'missing_views','missing_requirement','duplicate_requirement','implemented_requirement',
    'missing_transition','stop_needs_frame','old_hold_not_checked','missing_test','duplicate_test',
    'executed_test_claim','unknown_test_reference','unknown_anchor','missing_blocker')


def write(path,value):
    with Path(path).open('x') as stream:json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def identifiers(items,expected,code):
    check(type(items) is list and all(type(x) is dict and type(x.get('id')) is str for x in items),code)
    ids=[x['id'] for x in items];check(len(ids)==len(set(ids)) and set(ids)==set(expected),code)


def refs(value,allowed,code):
    check(type(value) is list and bool(value) and all(type(v) is str for v in value)
          and len(value)==len(set(value)) and set(value)<=set(allowed),code)


def validate_plan(doc):
    shape(doc,'format status authority limits ownership record steps handshake requirements transitions test_plan blockers preserved','review_schema')
    check(doc['format']=='precompute_integration_review_v1' and doc['status']=='review_only_not_implemented','review_status')
    check(type(doc['authority']) is dict and set(doc['authority'])==set(FALSE_FIELDS)
        and all(v is False for v in doc['authority'].values()),'no_authority')
    check(type(doc['limits']) is dict and all(type(doc['limits'].get(k)) is int for k in ('lease_ns','audit_ack_ns','sim_age_ns')), 'limit_integer_types')
    check(doc['limits']==dict(lease_ns=800_000_000,audit_ack_ns=400_000_000,sim_age_ns=250_001_000,
        work_anchor='original_complete_frame_availability',exposure_wall_s=None), 'fixed_limits_not_exposure')
    check(doc['ownership']==dict(origin='trusted_capture_boundary_not_implemented',registry='independent_owner_not_implemented',
        result_writer='controller_binding_only',deadline_writer='origin_owner_only',backend_lookup='registered_original_budget',
        authenticity='not_established_by_this_review'),'ownership_boundary')
    shape(doc['record'],'immutable pending_stages required_views source_truth','record_schema')
    check(doc['record']['immutable']==['session','host_clock_boot','frame','sim_identity','required_view_set','origin_available','original_deadline'], 'immutable_original_budget')
    check(doc['record']['pending_stages']==['reserved_no_command','sealed_once_no_deadline_rewrite','consumed_once'],'reserve_seal_consume')
    check(doc['record']['required_views']=='frozen_config_all_views_not_controller_subset'
        and doc['record']['source_truth']=='not_proved_by_uuid_hash_or_dataclass','view_and_trust_boundary')
    check(doc['steps']==list(STEPS),'prework_barriers_order')
    check(type(doc['handshake']) is dict and all(type(doc['handshake'].get(k)) is bool for k in ('python_and_native_required','deadline_rebased','compatibility_fallback')), 'handshake_bool_types')
    check(doc['handshake']==dict(kind='pending_install_and_seal_status_not_audit_flush_ack',
        python_and_native_required=True,deadline_rebased=False,compatibility_fallback=False,
        late_or_unknown_outcome='stop_and_preserve_unconfirmed_submission',wait_bound='min_original_pending_and_old_hold'), 'handshake_contract')
    identifiers(doc['requirements'],REQUIREMENTS,'requirement_coverage')
    covered=set()
    for r in doc['requirements']:
        shape(r,'id anchors tests status change','requirement_schema')
        check(r['status']=='not_implemented','requirement_not_implementation')
        refs(r['anchors'],ANCHORS,'source_anchor_reference');covered.update(r['anchors'])
        refs(r['tests'],TESTS,'test_reference')
        check(type(r['change']) is str and 20<=len(r['change'])<=4000,'change_description')
    check(covered==set(ANCHORS),'anchor_coverage')
    identifiers(doc['transitions'],TRANSITIONS,'transition_coverage')
    for t in doc['transitions']:
        shape(t,'id disposition old_hold_checked frame_required creates_authority tests','transition_schema')
        check(type(t['disposition']) is str and len(t['disposition'])>=20,'transition_description')
        check(t['old_hold_checked'] is True and t['creates_authority'] is False and type(t['frame_required']) is bool,'transition_guards')
        check(t['frame_required']==(t['id'] not in ('stop_without_frame','cancel','restart')),'stop_frame_independent')
        refs(t['tests'],TESTS,'transition_test_reference')
    identifiers(doc['test_plan'],TESTS,'test_plan_coverage')
    for t in doc['test_plan']:
        shape(t,'id status reuse expected','test_plan_schema')
        check(t['status']=='planned_not_executed','no_new_experiment_claim')
        refs(t['reuse'],('2B','2K','2L','source_only'),'prior_evidence_reference')
        check(type(t['expected']) is str and 20<=len(t['expected'])<=4000,'test_expectation')
    check(type(doc['blockers']) is list and all(type(v) is str for v in doc['blockers']) and len(doc['blockers'])==len(set(doc['blockers']))
        and set(doc['blockers'])==set(BLOCKERS),'readiness_blockers')
    check(doc['preserved']==['controllers_models_calibration_sensors','production_protocol_permissions',
        'speed_braking_recovery','lease_0.8_audit_ack_0.4','ros_existing_pending','all_failures_and_refusals'], 'preservation_contract')
    return dict(classification='offline_design_structure_only',requirements=len(REQUIREMENTS),transitions=len(TRANSITIONS),
        future_test_scenarios=len(TESTS),new_runtime_experiments=0,blockers=list(BLOCKERS),actual_exposure_wall_s=None,
        **{k:False for k in FALSE_FIELDS})


def invalid_plan(doc,name):
    check(name in INVALID,'unknown_mutation');d=copy.deepcopy(doc)
    if name=='authority':d['authority']['motion_authority']=True
    if name=='status':d['status']='implemented'
    if name=='limits':d['limits']['lease_ns']+=1
    if name=='exposure_fabricated':d['limits']['exposure_wall_s']=0
    if name=='missing_origin':d['ownership']['origin']='receipt_now'
    if name=='result_sets_deadline':d['ownership']['deadline_writer']='controller'
    if name=='hash_before_reserve':d['steps'][0],d['steps'][3]=d['steps'][3],d['steps'][0]
    if name=='compute_before_seal':d['steps'][5],d['steps'][7]=d['steps'][7],d['steps'][5]
    if name=='wrong_ack_kind':d['handshake']['kind']='audit_flush_ack'
    if name=='fallback':d['handshake']['compatibility_fallback']=True
    if name=='record_deadline_mutable':d['record']['immutable'].remove('original_deadline')
    if name=='missing_views':d['record']['required_views']='controller_subset'
    if name=='missing_requirement':d['requirements'].pop()
    if name=='duplicate_requirement':d['requirements'].append(copy.deepcopy(d['requirements'][0]))
    if name=='implemented_requirement':d['requirements'][0]['status']='implemented'
    if name=='missing_transition':d['transitions'].pop()
    if name=='stop_needs_frame':next(t for t in d['transitions'] if t['id']=='stop_without_frame')['frame_required']=True
    if name=='old_hold_not_checked':d['transitions'][0]['old_hold_checked']=False
    if name=='missing_test':d['test_plan'].pop()
    if name=='duplicate_test':d['test_plan'].append(copy.deepcopy(d['test_plan'][0]))
    if name=='executed_test_claim':d['test_plan'][0]['status']='passed'
    if name=='unknown_test_reference':d['requirements'][0]['tests']=['unknown']
    if name=='unknown_anchor':d['requirements'][0]['anchors']=['unknown']
    if name=='missing_blocker':d['blockers'].pop()
    return d


def audit_anchors(raw):
    rows=[]
    for identity,(path,cls,func,needle) in ANCHORS.items():
        text=raw[path].decode();start=0;body=text
        if func is not None:
            tree=ast.parse(text);nodes=tree.body
            if cls is not None:
                matches=[n for n in nodes if isinstance(n,ast.ClassDef) and n.name==cls]
                check(len(matches)==1,'class_anchor:'+identity);nodes=matches[0].body
            matches=[n for n in nodes if isinstance(n,ast.FunctionDef) and n.name==func]
            check(len(matches)==1,'function_anchor:'+identity);node=matches[0];start=node.lineno-1
            body='\n'.join(text.splitlines()[start:node.end_lineno])
        check(body.count(needle)==1,'text_anchor_not_unique:'+identity)
        line=start+body[:body.index(needle)].count('\n')+1
        rows.append(dict(id=identity,path=path,line=line,needle=needle,scope='pinned lexical text only; no control-flow/runtime proof'))
    return rows


def inspect_prior(raw):
    source=parse(raw[EVIDENCE_FILES[0]]);summary=parse(raw[OLD+'summary.json'])
    check(source['original_lease_s']==.8 and source['original_ack_s']==.4 and len(source['claims'])==6,'old_source_contract')
    check(summary['case_count']==20 and summary['expectations_match'],'old_probe_contract')
    for n in ('late_joint_resign_counterexample','owner_stall_before_send','owner_stall_no_poll'):
        row=parse(raw[OLD+n+'/result.json']);doc=parse(raw[OLD+n+'/input.json'])
        check(row['case']==n and row['expectations_match'],'prior_case_contract')
        check(hashlib.sha256(raw[OLD+n+'/input.json']).hexdigest()==row['input_sha256'],'prior_input_hash')
        check(row==next(x for x in summary['cases'] if x['case']==n),'prior_case_summary_binding')
        for backend in ('python','receiver'):
            trace=row[backend]
            if n=='late_joint_resign_counterexample':check(trace[0]['accepted'] and trace[0]['deadline_steady_s']>10.8,'resign_gap')
            elif n=='owner_stall_before_send':check(trace[-1]['seq']==0 and trace[-1]['deadline_steady_s'] is None and not trace[-1]['latched'],'pre_send_gap')
            else:check(len(trace)==1 and not trace[0]['latched'] and doc['schedule']['review_ns']/1e9>trace[0]['deadline_steady_s'],'no_poll_gap')
    return dict(prior_source_claims=6,prior_cases_read=3,new_guard_execution=False,prior_counterexamples_preserved=True)


def run(root,output,pins_path,pins_digest):
    root=Path(root);output=Path(output);pin_raw=read_index(pins_path)
    check(hashlib.sha256(pin_raw).hexdigest()==pins_digest,'pin_file_hash')
    pins=parse(pin_raw);check(type(pins) is dict and set(pins)==set(INPUT_FILES),'input_pin_schema')
    raw={n:read_index(root/n) for n in INPUT_FILES}
    check(all(hashlib.sha256(raw[n]).hexdigest()==pins[n] for n in INPUT_FILES),'input_pin_mismatch')
    output.mkdir(exist_ok=False);doc=parse(raw[DESIGN]);result=validate_plan(doc)
    anchors=audit_anchors(raw);prior=inspect_prior(raw)
    write(output/'design-review.json',result);write(output/'source-anchors.json',anchors);write(output/'prior-evidence.json',prior)
    refusals=[]
    for name in INVALID:
        mutated=invalid_plan(doc,name);write(output/(name+'.input.json'),mutated)
        try:validate_plan(mutated)
        except ValueError as exc:row=dict(case=name,refused=True,error=str(exc))
        else:row=dict(case=name,refused=False,error=None)
        write(output/(name+'.result.json'),row);refusals.append(row)
    check(read_index(pins_path)==pin_raw and all(read_index(root/n)==raw[n] for n in INPUT_FILES),'input_changed')
    summary=dict(design=result,anchors=anchors,prior_evidence=prior,refusals=refusals,
        input_sha256=pins,all_mutations_refused=all(x['refused'] for x in refusals),formal_retries=0,
        review_scope='structure, declared coverage and pinned source text/prior evidence only; NOT semantic completeness or implementation proof')
    write(output/'summary.json',summary);return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--input-pins',type=Path,required=True);p.add_argument('--input-pins-sha256',required=True);args=p.parse_args()
    result=run(Path(__file__).resolve().parents[1],args.output,args.input_pins,args.input_pins_sha256)
    print(json.dumps(dict(requirements=result['design']['requirements'],transitions=result['design']['transitions'],
        future_test_scenarios=result['design']['future_test_scenarios'],refusals=len(result['refusals']),
        all_mutations_refused=result['all_mutations_refused'],implementation_ready=False)))
    raise SystemExit(0 if result['all_mutations_refused'] else 1)
