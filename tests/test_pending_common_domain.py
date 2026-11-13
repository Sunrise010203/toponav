"""Common-domain acceptance is not old API equivalence, execution or authority."""
import ast
import hashlib
import json
from pathlib import Path
import pytest
from tools import pending_common_domain as c
from tools import pending_common_domain_cases as d
from tools import pending_protocol_cases as p
from tools import pending_cpp_reference_cases as q

ROOT=Path(__file__).resolve().parents[1];CASES=d.corpus(ROOT)


@pytest.fixture(scope='module')
def binary(tmp_path_factory):return d.compile_probe(ROOT,tmp_path_factory.mktemp('pending-common-domain'))


@pytest.mark.parametrize('f',CASES,ids=[f['case'] for f in CASES])
def test_common_domain_stages_results_and_registered_raw_errors(binary,f):
    py=c.review_envelope(f['data']);run=d.cpp_run(binary,f['data'])
    assert not run['stderr'] and run['returncode']==(0 if py['consistent'] else 2)
    d.compare(f,py,run['document'])
    assert c.review_envelope(f['data'])==py  # no one-time consumption


@pytest.mark.parametrize('raw',[None,'TPT1',bytearray(b'TPT1'),memoryview(b'TPT1'),True,{},[]])
def test_python_typed_byte_boundary_is_explicit(raw):
    out=c.review_envelope(raw)
    assert out['stage']=='domain_refused' and out['error']=='domain_bytes'


@pytest.mark.parametrize('name',['stop_float_views','refusal_float_views','stop_surrogate_views','stop_large_ignored_reserve'])
def test_old_python_ignored_arguments_still_return_but_new_profile_refuses(name):
    old=d.old;reg=old.registry((ROOT/'docs/PENDING_CPP_DIFFERENCES.json').read_bytes())
    f=next(r for r in old.corpus(reg) if r['family']==name)
    assert q.python_review(f['fixture'])['consistent']
    assert c.review_envelope(d.encode_fixture(f['fixture']))['stage']=='domain_refused'
    # The four original observed mismatches remain sealed, not retrospectively fixed.
    rows=json.loads((ROOT/'reports/straight-followup-20261103-28/cpp-differential-audit/summary.json').read_bytes())['cases']
    row=next(r for r in rows if r['case']=='wrapper--'+name)
    assert row['review']['relation']=='reference_envelope_refusal' and not row['review']['exact_equal']


@pytest.mark.parametrize('role',['message','reserve','seal','reply'])
def test_opaque_raw_bytes_not_reserialized_or_null_collapsed(role):
    f=p.fixture('stop_no_frame');f[role]=b'\xff\x00{bad'
    controls,packets=c.preflight(d.encode_fixture(f))
    assert packets[('message','reserve','seal','reply').index(role)]==f[role]
    assert controls['views_frozen'] is True


@pytest.mark.parametrize('name',['joint_context_replacement_counterexample','repeat_snapshot_counterexample','declared_refusal','expired_schema_is_not_admission'])
def test_new_profile_does_not_upgrade_old_offline_limits(name):
    out=c.review_envelope(d.encode_fixture(p.fixture(name)))
    assert out['consistent'] and out['result']['review']['actual_exposure_wall_s'] is None
    if name=='joint_context_replacement_counterexample':
        assert json.loads(p.fixture(name)['message'])['record']['deadline_ns']==11_500_000_000
    if name=='repeat_snapshot_counterexample':assert out['result']['review']==out['result']['repeated_review']
    if name=='declared_refusal':assert out['result']['review']['synthetic_reference_matches'] is False


def test_no_old_trace_ledger_or_runtime_scenarios_rewritten():
    root=ROOT/'reports/straight-followup-20261103-28/cpp-differential-audit'
    summary=json.loads((root/'summary.json').read_bytes())
    assert summary['trace_ledgers_compared']==82 and summary['trace_ledger_mismatches']==0
    for name in ('unknown_result_native','late_reply_retains_unknown','edge--late_unupgraded_future_observation','edge--late_unupgraded_wrong_original_record'):
        row=next(r for r in summary['cases'] if r['case']=='preserve--'+name)
        assert row['review']['ledger_equal'] and row['review']['exact_equal']
        py=json.loads((root/row['case']/'python.json').read_bytes())
        assert any(s['outcome']=='unknown_outcome' for s in py['result']['review']['submissions'])
    design=json.loads((ROOT/'docs/PRECOMPUTE_INTEGRATION_REVIEW.json').read_bytes())
    assert len(design['requirements'])==14 and all(r['status']=='not_implemented' for r in design['requirements'])
    assert len(design['test_plan'])==25 and all(r['status']=='planned_not_executed' for r in design['test_plan'])


def test_original_late_joint_resign_counterexample_only_read_as_archived_bytes():
    base=ROOT/'reports/straight-followup-20261102-21/work-budget-audit/late_joint_resign_counterexample'
    r=json.loads((base/'result.json').read_bytes())
    assert hashlib.sha256((base/'input.json').read_bytes()).hexdigest()==r['input_sha256']
    assert all(r[b][0]['accepted'] and r[b][0]['deadline_steady_s']>10.8 for b in ('python','receiver'))


def test_pure_python_entry_has_no_io_runtime_import_or_clock_reads():
    source=(ROOT/'tools/pending_common_domain.py').read_text();tree=ast.parse(source)
    imports=[n.module for n in ast.walk(tree) if isinstance(n,ast.ImportFrom)]
    imports += [a.name for n in ast.walk(tree) if isinstance(n,ast.Import) for a in n.names]
    assert set(imports)=={'dataclasses','json','struct','tools'}
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id in ('open','eval','exec','__import__') for n in ast.walk(tree))


def test_fixed_corpus_is_deterministic_and_packet_limits_distinguish_schema_from_domain():
    assert CASES==d.corpus(ROOT) and len({r['case'] for r in CASES})==len(CASES)
    assert {r['expected_stage'] for r in CASES}=={'domain_refused','review_refused','review_returned'}
    row=next(r for r in CASES if r['case']=='schema-bound--reference')
    assert c.review_envelope(row['data'])['domain_accepted']


def test_input_pins_checked_before_compilation_or_output(tmp_path,monkeypatch):
    pin=tmp_path/'pins.json';pin.write_text('{}');out=tmp_path/'out'
    monkeypatch.setattr(d,'compile_probe',lambda *a:pytest.fail('compiler before pin validation'))
    with pytest.raises(AssertionError):d.run(ROOT,out,pin,hashlib.sha256(pin.read_bytes()).hexdigest())
    assert not out.exists()
