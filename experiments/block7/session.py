"""Explicit atomic-only ablations of the existing coordinator.

No planner, localizer, event validator or transition logic is reimplemented.
Production SessionConfig, NavigationSession and continuous control stay intact.
"""
from dataclasses import dataclass, replace

from simulation.navigation_session import NavigationSession, SessionConfig, TERMINAL


VARIANTS = {'M0': (1, False), 'M1': (3, True), 'M2': (1, True), 'M3': (3, False)}


@dataclass(frozen=True)
class ExperimentConfig(SessionConfig):
    passive_recovery: bool = True

    def __post_init__(self):
        if type(self.passive_recovery) is not bool:
            raise ValueError('passive_recovery must be boolean')
        if type(self.confirmation_frames) is int and self.confirmation_frames == 1:
            # Validate EVERY other inherited field using the original validator.
            # The clone has two votes solely for validation; self remains one.
            replace(self, confirmation_frames=2)
        else:
            super().__post_init__()

    @classmethod
    def for_method(cls, method, **limits):
        if method not in VARIANTS:
            raise ValueError('Unknown experimental method')
        if {'confirmation_frames', 'passive_recovery'} & limits.keys():
            raise ValueError('Method mechanisms cannot be overridden by limits')
        frames, recovery = VARIANTS[method]
        return cls(confirmation_frames=frames, passive_recovery=recovery, **limits)


class _PolicyStop(Exception):
    """Unwind the parent's observe immediately; never resume its vote logic."""


class ExperimentSession(NavigationSession):
    def __init__(self, *args, config=None, **kwargs):
        if not isinstance(config, ExperimentConfig):
            raise ValueError('Explicit atomic ExperimentConfig required')
        super().__init__(*args, config=config, **kwargs)

    def _recover(self, reason, clear=True):
        # Poll ticks and still-unconfirmed execution are NOT evidence failures.
        if (not self.config.passive_recovery and reason not in
                {'no_new_observation', 'execution_unconfirmed'}):
            self._fail('passive_recovery_disabled:' + reason)
            raise _PolicyStop()
        return super()._recover(reason, clear=clear)

    def _call(self, function, *args):
        try:
            return function(*args)
        except _PolicyStop:
            return self._record('experiment_policy_stop')

    def observe(self, frame):
        return self._call(super().observe, frame)

    def feedback(self, event):
        return self._call(super().feedback, event)

    def tick(self):
        return self._call(super().tick)

    def missing_capture(self):
        """A scheduled camera capture was absent, unlike an ordinary poll tick."""
        if not self.config.passive_recovery and self.pending is None:
            if self._guard():
                self.metrics['events'] += 1
                self._fail('passive_recovery_disabled:scheduled_capture_missing')
            return self._record('scheduled_capture_missing')
        return self.tick()

