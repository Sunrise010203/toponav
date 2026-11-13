"""Bind the true old projection across the WHOLE controller, not just H/I.

The local wrapper also applies this context because the original benchmark's
local cases do not enter its controller replace_aliases context.
"""
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace

from roadvision import corridor_rgb_probe as project_module, paired_descriptor_checks as pair
from tools import straight_paired_population_benchmark as bench
from tools.straight_pipeline_profile import discover_bindings, pipeline_bindings

HERE = Path(__file__).resolve().parent
ORIGINAL_LOAD = bench.load_before
ORIGINAL_REPLACE = bench.replace_aliases
CURRENT_PAIR = pair.filter_pair


class ProjectionBaseline:
    def __init__(self, path):
        self.old = ORIGINAL_LOAD(path)._project
        self.new = project_module._project
        pipeline_bindings()  # preload every supported controller before discovery
        self.bindings = discover_bindings([(self.new, 'projection')])
        owners = {owner.__name__ for owner, _, _ in self.bindings}
        if not {'roadvision.corridor_rgb_probe', 'roadvision.rgb_motion_checks',
                'roadvision.rgb_resampling_checks'}.issubset(owners):
            raise ValueError('missing_projection_aliases')

    @contextmanager
    def context(self):
        with ORIGINAL_REPLACE(self.bindings, self.old):
            yield

    def local_before(self, *args, **kwargs):
        with self.context():
            return CURRENT_PAIR(*args, **kwargs)

    def load_before(self, path):
        if Path(path).resolve() != (HERE/'before/roadvision/corridor_rgb_probe.py').resolve():
            raise ValueError('unexpected_baseline_source')
        return SimpleNamespace(filter_pair=self.local_before)

    @contextmanager
    def replace_aliases(self, bindings, fn):
        # This outer context spans the entire complete-sequence pass, including
        # project calls outside filter_pair. All H/I filter identities stay put.
        with ORIGINAL_REPLACE(bindings, fn):
            if fn == self.local_before:
                with self.context():
                    yield
            else:
                yield

    def record(self):
        return dict(original_projection_source=str(HERE/'before/roadvision/corridor_rgb_probe.py'),
                    aliases=[owner.__name__+'.'+name for owner, name, _ in self.bindings],
                    complete_controller_projection_binding=True, local_projection_binding=True,
                    same_paired_filter_implementation=True, same_original_h_i_filters=True,
                    original_benchmark_source_unmodified=True)
