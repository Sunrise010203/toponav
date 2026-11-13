"""Executable SHADOW DESIGN ONLY: capture provenance and pre-compute deadlines.

No production entrypoint imports this module. It cannot publish/drive anything;
all permits explicitly carry motion_authority=False. Supplying a dataclass is
NOT proof of trusted acquisition or clock mapping. No real provider exists here.
"""
from dataclasses import asdict, dataclass, field
import math
import secrets

from simulation.gazebo_actuator import LEASE_S

MAX_FRAMES = 4096  # Prototype registry cap, not a new production capacity.
MAX_VIEWS = 6  # Existing four-view composite plus up/down auxiliary route views.


class Rejected(ValueError):
    pass


def finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value) and value >= 0
    except OverflowError:
        return False


def identity(value):
    return type(value) is str and 0 < len(value) <= 256


@dataclass(frozen=True)
class ViewEvidence:
    view: str
    rgb_sha256: str
    received_steady_s: float
    source: str = 'receipt_only'
    exposure_lower_s: float | None = None
    exposure_upper_s: float | None = None


@dataclass(frozen=True)
class FrameEvidence:
    session_id: str
    frame_id: str
    captured_sim_s: float
    clock_domain_id: str
    views: tuple[ViewEvidence, ...]


@dataclass(frozen=True)
class ResultBinding:
    session_id: str
    frame_id: str
    captured_sim_s: float
    view_digests: tuple[tuple[str, str], ...]


def binding_for(frame):
    return ResultBinding(frame.session_id, frame.frame_id, frame.captured_sim_s,
                         tuple((v.view, v.rgb_sha256) for v in frame.views))


@dataclass(frozen=True)
class ShadowPermit:
    binding: ResultBinding
    seq: int
    issued_steady_s: float
    expires_steady_s: float
    motion_authority: bool = field(default=False, init=False)


class DeadlineProposal:
    """Single-owner state model, NOT an actuator gate or runtime watchdog.

    Capture lower bounds are assumed to come from a trusted owner, never from
    controller output. This assumption is deliberately NOT implemented as an
    authenticity claim. A stopped owner cannot poll or enforce physical zero.
    """
    def __init__(self, session_id, clock_domain_id, required_views=('forward',)):
        if not identity(session_id) or not identity(clock_domain_id):
            raise ValueError('Invalid owner identity')
        if (type(required_views) is not tuple or not 1 <= len(required_views) <= MAX_VIEWS
                or not all(identity(v) for v in required_views) or len(set(required_views)) != len(required_views)):
            raise ValueError('Invalid owner view configuration')
        self.session_id, self.clock_domain_id = session_id, clock_domain_id
        self.required_views = frozenset(required_views)
        self.latched, self.reason = False, 'disarmed'
        self.last_wall = -1.
        self.pending = None
        self.hold_deadline = None
        self.seq = 0
        self.last_sim = -1.
        self.frames = set()

    def trip(self, reason):
        if not self.latched:
            self.reason = reason
        self.latched = True

    def reject(self, reason):
        self.trip(reason)
        raise Rejected(self.reason)

    @property
    def effective_deadline(self):
        bounds = [self.hold_deadline]
        if self.pending is not None:
            bounds.append(self.pending['deadline'])
        return min((v for v in bounds if v is not None), default=None)

    def poll(self, now):
        if not finite(now) or now < self.last_wall:
            self.trip('clock_invalid_or_regressed')
            return
        self.last_wall = now
        deadline = self.effective_deadline
        if not self.latched and deadline is not None and now >= deadline:
            self.trip('precompute_or_hold_expired')

    def admit(self, frame, now):
        self.poll(now)
        if self.latched:
            self.reject(self.reason)
        if self.pending is not None:
            self.reject('pending_frame_overwrite')
        if (type(frame) is not FrameEvidence or not identity(frame.frame_id)
                or not identity(frame.session_id) or frame.session_id != self.session_id or not finite(frame.captured_sim_s)
                or not identity(frame.clock_domain_id) or frame.clock_domain_id != self.clock_domain_id
                or type(frame.views) is not tuple or not 1 <= len(frame.views) <= MAX_VIEWS):
            self.reject('invalid_frame_or_clock_binding')
        if frame.frame_id in self.frames or frame.captured_sim_s <= self.last_sim:
            self.reject('reused_frame')
        if len(self.frames) >= MAX_FRAMES:
            self.reject('frame_capacity_exceeded')
        lower_bounds, names = [], set()
        for view in frame.views:
            if (type(view) is not ViewEvidence or not identity(view.view) or view.view in names
                    or type(view.rgb_sha256) is not str or len(view.rgb_sha256) != 64
                    or any(c not in '0123456789abcdef' for c in view.rgb_sha256)
                    or not finite(view.received_steady_s) or view.received_steady_s > now):
                self.reject('invalid_view_binding')
            names.add(view.view)
            # Receipt alone, simulator time, and a caller's re-signing time do
            # not establish an exposure-age bound. Missing => shadow no_motion.
            if type(view.source) is not str or view.source != 'bounded_exposure_mapping':
                self.reject('exposure_age_unknown')
            lo, hi = view.exposure_lower_s, view.exposure_upper_s
            if (not finite(lo) or not finite(hi)
                    or not lo <= hi <= view.received_steady_s):
                self.reject('invalid_exposure_interval')
            lower_bounds.append(lo)
        if names != self.required_views:
            self.reject('required_view_set_mismatch')
        # Oldest possible exposure, across ALL required views; never upper
        # bound, newest view, receipt time, compute completion or clock grace.
        deadline = min(lower_bounds)+LEASE_S
        if now >= deadline:
            self.reject('capture_budget_exhausted')
        self.frames.add(frame.frame_id)
        self.last_sim = frame.captured_sim_s
        token = secrets.token_hex(16)
        self.pending = dict(token=token, binding=binding_for(frame), deadline=deadline)
        self.reason = 'awaiting_shadow_result'
        return token

    def complete(self, token, binding, now):
        self.poll(now)
        if self.latched:
            self.reject(self.reason)
        pending = self.pending
        if (pending is None or type(token) is not str or token != pending['token']
                or type(binding) is not ResultBinding or not identity(binding.session_id)
                or not identity(binding.frame_id) or not finite(binding.captured_sim_s)
                or type(binding.view_digests) is not tuple
                or any(type(x) is not tuple or len(x) != 2
                       or type(x[0]) is not str or type(x[1]) is not str for x in binding.view_digests)
                or binding != pending['binding']):
            self.reject('unbound_or_replayed_result')
        # Only the owner sets issued/expires. Completion cannot restore budget.
        permit = ShadowPermit(binding, self.seq+1, now, min(pending['deadline'], now+LEASE_S))
        self.seq += 1
        self.hold_deadline = permit.expires_steady_s
        self.pending = None
        self.reason = 'shadow_result_eligible_NOT_motion'
        return permit

    def snapshot(self):
        return dict(latched=self.latched, reason=self.reason, seq=self.seq,
            pending_deadline=self.pending['deadline'] if self.pending else None,
            hold_deadline=self.hold_deadline, effective_deadline=self.effective_deadline,
            retained_frames=len(self.frames), motion_authority=False)


def as_wire_binding(value):
    return asdict(value)
