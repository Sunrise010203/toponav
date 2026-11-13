"""Startup diagnostics are evidence attribution, not motion acceptance."""
import copy
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import pytest

from roadvision import continuous_straight as control
from test_straight_task2_permissions import setup, frame
from tools import diagnose_straight_task1 as diagnostic


def startup_row():
    frames = [dict(frame_id=f'f{i}', captured_sim_s=i * .1, accepted=i != 0,
                   reason='reference_texture_ambiguous_or_unsupported' if i == 0 else 'reference_texture_supported',
                   failed_checks=['rectified_supported_cell_count'] if i == 0 else [],
                   checks=dict(rectified_supported_cell_count=i != 0),
                   cells=[dict(rectified_accepted=i != 0)],
                   reference_supported_cells=9, rectified_supported_cells=5 if i == 0 else 6,
                   rgb_sha256='a' * 64) for i in range(3)]
    return dict(tick=2, sim_s=.2, input=dict(frame_id='f2', captured_sim_s=.2),
                decision=dict(state='aborted', reason='startup_texture_unverified',
                              v_m_s=0., omega_rad_s=0., captured_sim_s=.2,
                              control_frame_id='f2', decision_sim_s=.2,
                              permission=dict(startup_texture=dict(accepted=False, evaluated_sim_s=.2, frames=frames))))


def test_startup_rejection_is_not_reported_as_no_blockers():
    row = startup_row()
    summary, records = diagnostic.summarize([row])
    assert records[0]['startup_texture_current']
    assert records[0]['blockers'] == ['startup_texture_unverified']
    assert summary['blocker_counts'] == {'startup_texture_unverified': 1}
    assert summary['first_evidence_failure']['tick'] == 2
    assert summary['fresh_perception_count'] == 0
    assert summary['startup_assessment_count'] == 1
    assert records[0]['startup_texture_frames'][0]['failed_checks'] == ['rectified_supported_cell_count']


@pytest.mark.parametrize('change', ['input_id', 'input_time', 'no_input', 'decision_id',
                                  'decision_time', 'evaluation_time', 'unrelated_abort'])
def test_cached_startup_result_is_not_new_evidence(change):
    row = startup_row()
    if change == 'input_id': row['input']['frame_id'] = 'other'
    elif change == 'input_time': row['input']['captured_sim_s'] = .1
    elif change == 'no_input': row['input'] = None
    elif change == 'decision_id': row['decision']['control_frame_id'] = 'other'
    elif change == 'decision_time': row['decision']['decision_sim_s'] = .3
    elif change == 'evaluation_time': row['decision']['permission']['startup_texture']['evaluated_sim_s'] = .1
    elif change == 'unrelated_abort': row['decision']['reason'] = 'replayed_frame'
    result = diagnostic.evidence(row)
    assert not result['startup_texture_current']
    assert result['blockers'] == []


def test_delayed_but_current_startup_is_not_confused_with_stale_cache():
    row = startup_row(); row['sim_s'] = .4
    row['decision']['decision_sim_s'] = .4
    row['decision']['permission']['startup_texture']['evaluated_sim_s'] = .4
    assert diagnostic.evidence(row)['startup_texture_current']


def test_summary_does_not_count_same_startup_assessment_twice():
    row = startup_row()
    summary, _ = diagnostic.summarize([row, copy.deepcopy(row)])
    assert summary['startup_assessment_count'] == 1
    assert summary['blocker_counts'] == {'startup_texture_unverified': 1}


def test_controller_preserves_all_three_cell_results_and_pixel_identity(monkeypatch):
    c, _ = setup(monkeypatch)
    outputs = []
    def texture(rgb, calibration):
        i = len(outputs)
        result = dict(accepted=i != 0, reason='synthetic', checks=dict(native=i != 0),
                      failed_checks=['native'] if i == 0 else [], cells=[dict(index=i)],
                      reference_supported_cells=8 if i == 0 else 9, rectified_supported_cells=6)
        outputs.append(copy.deepcopy(result)); return result
    monkeypatch.setattr(control, 'assess_startup_texture', texture)
    inputs = [frame(i) for i in range(3)]
    for f in inputs: out = c.update(f, f['captured_sim_s'] + .05)
    startup = out['permission']['startup_texture']
    assert not startup['accepted'] and out['v_m_s'] == 0
    assert startup['evaluated_sim_s'] == .25
    for f, expected, recorded in zip(inputs, outputs, startup['frames']):
        assert recorded['captured_sim_s'] == f['captured_sim_s']
        assert recorded['rgb_sha256'] == hashlib.sha256(f['rgb'].tobytes()).hexdigest()
        assert recorded['checks'] == expected['checks']
        assert recorded['cells'] == expected['cells']
    assert startup['frames'][0]['cells'] != startup['latest']['cells']


def recorded_fixture(tmp_path):
    source = tmp_path / 'capture'; (source / 'public/frames').mkdir(parents=True)
    pixels = np.arange(256 * 256 * 3, dtype=np.uint8).reshape(256, 256, 3)
    Image.fromarray(pixels).save(source / 'public/frames/0002.png')
    row = startup_row()
    row['input'].update(rgb_path='frames/0002.png', rgb_sha256=hashlib.sha256(pixels.tobytes()).hexdigest())
    (source / 'public/sequence.json').write_text(json.dumps(dict(frames=[row['input']])))
    (source / 'decisions.jsonl').write_text(json.dumps(row) + '\n')
    return source, row


def test_recorded_diagnosis_is_read_only_and_not_acceptance(tmp_path):
    source, _ = recorded_fixture(tmp_path)
    before = {str(p): p.read_bytes() for p in source.rglob('*') if p.is_file()}
    result = diagnostic.diagnose_recorded(source, tmp_path / 'diagnosis')
    assert result['summary']['blocker_counts'] == {'startup_texture_unverified': 1}
    assert not result['formal_navigation_acceptance']
    assert result['verified_public_frames'] == 1
    assert before == {str(p): p.read_bytes() for p in source.rglob('*') if p.is_file()}
    with pytest.raises(FileExistsError): diagnostic.diagnose_recorded(source, tmp_path / 'diagnosis')


@pytest.mark.parametrize('change', ['pixels', 'input_id', 'sequence_missing', 'traversal', 'symlink', 'empty'])
def test_recorded_diagnosis_refuses_unverified_public_evidence(tmp_path, change):
    source, row = recorded_fixture(tmp_path)
    if change == 'pixels': Image.fromarray(np.zeros((256, 256, 3), np.uint8)).save(source / 'public/frames/0002.png')
    elif change == 'input_id': row['input']['frame_id'] = 'wrong'
    elif change == 'sequence_missing': (source / 'public/sequence.json').unlink()
    elif change in ('traversal', 'symlink'):
        outside = tmp_path / 'outside.png'; Image.fromarray(np.zeros((256, 256, 3), np.uint8)).save(outside)
        if change == 'traversal': row['input']['rgb_path'] = '../../outside.png'
        else:
            (source / 'public/frames/link.png').symlink_to(outside)
            row['input']['rgb_path'] = 'frames/link.png'
        (source / 'public/sequence.json').write_text(json.dumps(dict(frames=[row['input']])))
    elif change == 'empty': row = None; (source / 'public/sequence.json').write_text('{"frames": []}')
    (source / 'decisions.jsonl').write_text(json.dumps(row) + '\n' if row else '')
    with pytest.raises((ValueError, FileNotFoundError)):
        diagnostic.diagnose_recorded(source, tmp_path / 'bad-diagnosis')
    assert not (tmp_path / 'bad-diagnosis').exists()


def test_recorded_diagnosis_cannot_write_inside_capture(tmp_path):
    source, _ = recorded_fixture(tmp_path)
    with pytest.raises(ValueError): diagnostic.diagnose_recorded(source, source / 'diagnosis')
