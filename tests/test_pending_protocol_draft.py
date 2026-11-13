"""Only bounded bytes and caller-supplied snapshots; no runtime backend or owner."""
import ast
from dataclasses import asdict, FrozenInstanceError
import hashlib
import json
from pathlib import Path

import pytest

from tools import pending_protocol_draft as p
from tools import pending_protocol_cases as cases

ROOT=Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('name',cases.VALID)
def test_valid_synthetic_schema_or_snapshot_not_authority(name):
    f=cases.fixture(name);result=cases.evaluate(f);r=result['review']
    assert r['actual_exposure_wall_s'] is None and all(r[k] is False for k in p.FALSE_FIELDS)
    assert r['classification'] in ('caller_snapshot_reference_only','frame_free_stop_schema_only',
        'synthetic_reply_claim_only','declared_refusal_not_execution','decoded_schema_not_admission')


@pytest.mark.parametrize('name',cases.INVALID)
def test_bad_schema_reference_or_reply_refused(name):
    with pytest.raises(p.DraftError):cases.evaluate(cases.fixture(name))


def test_all_case_names_unique_and_closed():
    assert len(cases.VALID+cases.INVALID)==len(set(cases.VALID+cases.INVALID))
    with pytest.raises(p.DraftError):cases.fixture('unknown')


def test_repeat_is_explicitly_not_consumption_control():
    result=cases.evaluate(cases.fixture('repeat_snapshot_counterexample'))
    assert result['review']==result['repeated_review']
    assert not result['review']['runtime_result_consumed']


def test_joint_replacement_can_pass_without_trusted_original_context():
    f=cases.fixture('joint_context_replacement_counterexample')
    request=p.decode_message(f['message']);original=p.decode_message(cases.fixture('result_reference')['message'])
    assert request.record.deadline_ns>original.record.deadline_ns
    assert cases.evaluate(f)['review']['synthetic_reference_matches']
    assert not cases.evaluate(f)['review']['provenance_authenticated']


def test_single_packet_schema_is_not_timely_admission():
    f=cases.fixture('expired_schema_is_not_admission');d=p.decode_message(f['message'])
    assert d.record.deadline_ns==800_000_000
    r=cases.evaluate(f)['review'];assert not r['synthetic_reference_matches'] and not r['motion_authority']


def test_refusal_claim_does_not_assert_unchecked_record_matches():
    r=cases.evaluate(cases.fixture('declared_refusal'))['review']
    assert r['classification']=='declared_refusal_not_execution' and not r['synthetic_reference_matches']


def test_stop_does_not_require_frame_configuration_or_reserve():
    raw=cases.fixture('stop_no_frame')['message']
    r=p.review_reference(raw,b'invalid ignored context',b'invalid ignored context',required_views=None)
    assert r.classification=='frame_free_stop_schema_only' and not r.motion_authority


@pytest.mark.parametrize('raw',[None,True,{},bytearray(b'{}'),'{}',b'',b'\xef\xbb\xbf{}',b'{} {}'])
def test_input_must_be_one_bounded_utf8_bytes_document(raw):
    with pytest.raises(p.DraftError):p.decode_message(raw)


@pytest.mark.parametrize('value',[True,-1,1.,2**63,'10',None])
def test_noninteger_or_invalid_review_time_refused(value):
    f=cases.fixture('result_ack_native');f['kwargs']['review_ns']=value
    with pytest.raises(p.DraftError):cases.evaluate(f)


@pytest.mark.parametrize('value',[True,-1,1.,2**63,'10'])
def test_old_hold_requires_bounded_integer_same_declared_domain(value):
    f=cases.fixture('result_ack_native');f['kwargs']['old_hold_deadline_ns']=value
    with pytest.raises(p.DraftError):cases.evaluate(f)


@pytest.mark.parametrize('kind',['reserve','seal','result_ref','ack','status','stop'])
def test_missing_and_extra_keys_never_fall_back_to_production(kind):
    source={'reserve':'reserve','seal':'seal','result_ref':'result_reference','ack':'result_ack_native',
        'status':'disarmed_status','stop':'stop_no_frame'}[kind]
    f=cases.fixture(source);raw=f['reply'] if kind=='ack' else f['message'];doc=json.loads(raw)
    for key in tuple(doc):
        bad=dict(doc);bad.pop(key)
        with pytest.raises(p.DraftError):p.decode_message(cases.wire(bad))
    for key in ('version','issued_steady_s','expires_steady_s','v_m_s','omega_rad_s','motion_authority'):
        bad=dict(doc);bad[key]=0
        with pytest.raises(p.DraftError):p.decode_message(cases.wire(bad))


def test_nested_dataclasses_and_sequences_are_immutable_not_authenticated():
    m=p.decode_message(cases.fixture('seal')['message'])
    assert type(m.record.required_views) is tuple and type(m.view_digests) is tuple
    with pytest.raises(FrozenInstanceError):m.record.deadline_ns+=1
    with pytest.raises(FrozenInstanceError):m.kind='reserve'
    with pytest.raises(p.DraftError):p.decode_message(m)


def test_frozen_review_authority_cannot_be_constructor_argument():
    with pytest.raises(TypeError):p.DraftReview('fake','reserve',True,motion_authority=True)


def test_deep_json_is_rejected_before_json_loader(monkeypatch):
    def forbidden(*a,**k):pytest.fail('loader must not see excessive depth or bytes')
    monkeypatch.setattr(p.json,'loads',forbidden)
    for raw in (b'['*(p.MAX_DEPTH+1)+b'0'+b']'*(p.MAX_DEPTH+1),b' '*(p.MAX_BYTES+1)):
        with pytest.raises(p.DraftError):p.decode_message(raw)


def test_braces_in_strings_do_not_count_as_container_depth():
    # A long invalid identifier is rejected by schema, not a false depth count.
    d=json.loads(cases.fixture('reserve')['message']);d['session_id']='{'*40
    with pytest.raises(p.DraftError,match='message_identity'):p.decode_message(cases.wire(d))


def test_config_cannot_be_a_mutable_list_or_drop_view():
    f=cases.fixture('result_reference')
    for views in (['front','left','right','back'],('front','left','right')):
        with pytest.raises(p.DraftError):p.review_reference(f['message'],f['reserve'],f['seal'],required_views=views)


def test_maximum_nonoverflowing_record_timestamp_is_supported():
    f=cases.fixture('reserve');d=json.loads(f['message']);d['record'].update(available_ns=p.MAX_NS-p.LEASE_NS,deadline_ns=p.MAX_NS)
    assert p.decode_message(cases.wire(d)).record.deadline_ns==p.MAX_NS
    d['record']['available_ns']+=1
    with pytest.raises(p.DraftError):p.decode_message(cases.wire(d))


def test_zero_generation_and_request_are_not_disarmed_authority():
    for n in ('zero_generation','zero_request'):
        with pytest.raises(p.DraftError):cases.evaluate(cases.fixture(n))


def test_private_namespace_and_fixed_limits_do_not_modify_production():
    assert p.PROTOCOL!='1' and p.SCOPE=='offline_schema_only'
    assert p.LEASE_NS==800_000_000 and p.AUDIT_ACK_NS==400_000_000 and p.MAX_BYTES==8192
    tree=ast.parse((ROOT/'tools/pending_protocol_draft.py').read_text())
    modules={n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)}
    modules|={a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names}
    assert modules=={'dataclasses','json'}
    calls={ast.unparse(n.func) for n in ast.walk(tree) if isinstance(n,ast.Call)}
    assert not calls.intersection({'open','time.monotonic','time.sleep','socket.socket','subprocess.run'})


def test_formal_pin_errors_refuse_before_output_or_cases(tmp_path,monkeypatch):
    pins=tmp_path/'pins.json';pins.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(cases,'fixture',lambda *a:pytest.fail('no case execution before pins'))
    for digest in ('0'*64,hashlib.sha256(pins.read_bytes()).hexdigest()):
        with pytest.raises(p.DraftError):cases.run(ROOT,out,pins,digest)
    assert not out.exists()
