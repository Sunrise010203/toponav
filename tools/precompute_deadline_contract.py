"""Pure synthetic schedule comparison ONLY, never an owner, permit or live gate.

Receipt/availability-relative work budgets do NOT bound true exposure age.
No current clock, callbacks, IO, runtime imports or transport. Integer ns are
caller-supplied fictional values in ONE synthetic monotonic domain.
"""
from dataclasses import dataclass, field

LEASE_NS=800_000_000
AUDIT_ACK_NS=400_000_000
MAX_NS=2**63-1
FORMAT='synthetic_precompute_schedule_v1'


class ContractError(ValueError):pass


def check(ok,code):
    if not ok:raise ContractError(code)


def shape(obj,keys,code):
    check(type(obj) is dict and set(obj)==set(keys.split()),code)


def stamp(value):return type(value) is int and 0<=value<=MAX_NS


def binding(value):
    shape(value,'frame_id captured_sim_ns view_digests','binding_schema')
    check(type(value['frame_id']) is str and 0<len(value['frame_id'])<=128 and stamp(value['captured_sim_ns']),'binding_identity')
    views=value['view_digests']
    check(type(views) is list and 4<=len(views)<=6 and all(type(v) is list and len(v)==2
        and type(v[0]) is str and type(v[1]) is str and len(v[1])==64
        and all(c in '0123456789abcdef' for c in v[1]) for v in views),'binding_views')
    names=[v[0] for v in views]
    check(names[:4]==['front','left','right','back'] and len(set(names))==len(names)
        and set(names[4:])<={'up','down'},'binding_camera_order')


@dataclass(frozen=True)
class WindowComparison:
    name: str
    new_pending_deadline_before_work_ns: int | None
    effective_deadline_during_work_ns: int | None
    work_deadline_due_at_review: bool | None
    reported_result_within_window: bool | None
    hypothetical_hold_after_timely_result_ns: int | None


@dataclass(frozen=True)
class ScheduleComparison:
    classification: str
    frame_id: str
    captured_sim_ns: int
    work_elapsed_since_available_ns: int | None
    capture_wait_ns: int
    reported_binding_matches: bool | None
    windows: tuple[WindowComparison,...]
    new_frame_budget_covers_capture_wait: bool = field(default=False,init=False)
    actual_exposure_wall_s: None = field(default=None,init=False)
    exposure_age_bounded: bool = field(default=False,init=False)
    provenance_authenticated: bool = field(default=False,init=False)
    shadow_admission_authorized: bool = field(default=False,init=False)
    native_live_observation_authorized: bool = field(default=False,init=False)
    motion_authority: bool = field(default=False,init=False)
    independent_enforcement_observed: bool = field(default=False,init=False)
    physical_stop_proved: bool = field(default=False,init=False)
    production_guard_fixed: bool = field(default=False,init=False)


def compare_schedule(doc):
    """Compare arithmetic, NOT actual runtime acceptance, timing or stopping.

    Legacy first result can obtain a new post-compute .8s window. ROS already
    has a pending deadline after capture, before PNG/DDS, but timely accept
    starts its held-command lease. Candidate retains the ORIGINAL availability
    deadline after a timely result. Old held limits constrain ALL work windows;
    only a timely new bound result can hypothetically replace an old hold.
    """
    shape(doc,'format clock_domain capture_started_ns frame_available_ns serialized_ns compute_started_ns '
          'compute_returned_ns review_ns held_deadline_ns input_binding result_binding','schedule_schema')
    check(doc['format']==FORMAT and doc['clock_domain']=='synthetic_monotonic_ns','synthetic_domain_required')
    names=('capture_started_ns','frame_available_ns','serialized_ns','compute_started_ns','review_ns')
    check(all(stamp(doc[n]) for n in names),'invalid_host_stamp')
    capture,available,serialized,started,review=(doc[n] for n in names)
    check(capture<=available<=serialized<=started<=review,'host_stage_order')
    returned=doc['compute_returned_ns'];held=doc['held_deadline_ns']
    check(held is None or stamp(held),'invalid_held_deadline')
    check(returned is None or stamp(returned) and started<=returned<=review,'return_stage_order')
    check(available<=MAX_NS-LEASE_NS and (returned is None or returned<=MAX_NS-LEASE_NS),'deadline_overflow')
    binding(doc['input_binding']);result=doc['result_binding']
    if returned is None:check(result is None,'result_without_return')
    else:binding(result)
    matched=None if result is None else result==doc['input_binding']
    original=available+LEASE_NS;windows=[]
    for name,pending in (('gazebo_postcompute',None),('ros_existing_pending',original),('candidate_fixed_availability',original)):
        limits=[v for v in (held,pending) if v is not None];effective=min(limits) if limits else None
        within=None if returned is None else matched and (effective is None or returned<effective)
        after=None if within is not True else original if name=='candidate_fixed_availability' else returned+LEASE_NS
        windows.append(WindowComparison(name,pending,effective,None if effective is None else review>=effective,within,after))
    return ScheduleComparison('synthetic_arithmetic_only',doc['input_binding']['frame_id'],doc['input_binding']['captured_sim_ns'],
        None if returned is None else returned-available,available-capture,matched,tuple(windows))
