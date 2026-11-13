from simulation.ground_history_acceptance import assess_metrics


def sample():
    return (dict(evaluation_group='straight'),
            dict(window_count=5,ground_status_counts=dict(conditional_ground_motion=5),corridor_status_counts=dict(conditional_corridor_geometry=5),front_status_counts=dict(unknown=5)),
            {k:dict(count=5,max=.001) for k in ('translation_error_m','yaw_error_rad','offset_error_m','heading_error_rad','width_error_m','front_error_m')})


def test_all_rejected_is_not_normal_function_success():
    case,summary,metrics=sample();summary['ground_status_counts']={'unknown':5};summary['corridor_status_counts']={'unknown':5}
    assert assess_metrics(case,summary,metrics)


def test_coverage_and_error_are_both_required():
    case,summary,metrics=sample();assert not assess_metrics(case,summary,metrics)
    metrics['translation_error_m']['max']=.016
    assert 'translation_error_m' in ';'.join(assess_metrics(case,summary,metrics))


def test_near_front_has_a_real_nonzero_completion_denominator():
    case,summary,metrics=sample();case['evaluation_group']='near_front'
    assert 'front_coverage' in ';'.join(assess_metrics(case,summary,metrics))
    summary['front_status_counts']={'conditional_front_surface':3,'unknown':2}
    assert not assess_metrics(case,summary,metrics)
