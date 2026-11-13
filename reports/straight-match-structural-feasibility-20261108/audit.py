"""Read-only feasibility metadata and unchanged-file verification; no matcher runs.

Run with an exclusive JSON output path. Does not set OpenCV/backend/thread flags,
load controllers, run numerical candidates, benchmarks, tests or physical devices.
"""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(output):
    pins = json.loads((HERE / 'BEFORE_PINS.json').read_text())['files']
    mismatches = [name for name, digest in pins.items()
                  if not (ROOT / name).is_file() or sha(ROOT / name) != digest]
    assert not mismatches, mismatches
    prior = ROOT / 'reports/straight-ncc-unit-clip-20261104/FINAL_PINS.json'
    previous = json.loads(prior.read_text())['files']
    assert all(sha(ROOT / name) == digest for name, digest in previous.items())
    cpp = ROOT / 'reports/straight-match-representation-20261104/source-review/templmatch.cpp'
    source = cpp.read_text()
    markers = [
        'void crossCorr(', 'static void common_matchTemplate(',
        'integral(img, sum, sqsum, CV_64F);',
        'static bool ipp_crossCorr(', 'static bool ipp_matchTemplate(',
        'CV_IPP_RUN_FAST(ipp_matchTemplate(img, templ, result, method))',
        'void cv::matchTemplate(',
    ]
    locations = {}
    for marker in markers:
        lines = [i for i, line in enumerate(source.splitlines(), 1) if marker in line]
        assert lines, marker
        locations[marker] = lines
    python_locations = {}
    for name in ('roadvision/paired_descriptor_checks.py',
                 'roadvision/prepared_descriptor_search.py'):
        tree = ast.parse((ROOT / name).read_bytes())
        python_locations[name] = {
            node.name: {'start': node.lineno, 'end': node.end_lineno}
            for node in ast.walk(tree)
            if isinstance(node, (ast.FunctionDef, ast.ClassDef))
        }
    package = Path(cv2.__file__).resolve().parent
    binaries = list(package.glob('cv2*.so'))
    assert len(binaries) == 1
    pkg = subprocess.run(['pkg-config', '--modversion', 'opencv4'],
                         capture_output=True, text=True, check=False)
    info = cv2.getBuildInformation()
    result = {
        'scope': 'Static evidence and runtime metadata, not a numerical equivalence or performance test',
        'preexisting_files_verified': len(pins),
        'previous_final_pins_verified': len(previous),
        'preexisting_files_changed': [],
        'production_candidate_created': False,
        'round_2_started': False,
        'decision': 'stop_current_micro_optimization_route_after_feasibility',
        'opencv_version': cv2.__version__, 'numpy_version': np.__version__,
        'opencv_package': str(package),
        'opencv_binary_sha256': sha(binaries[0]),
        'opencv_threads_read_only': cv2.getNumThreads(),
        'optimized_read_only': cv2.useOptimized(),
        'ipp_read_only': cv2.ipp.useIPP(),
        'match_template_signature': cv2.matchTemplate.__doc__.splitlines()[0],
        'public_names_matching_crosscorr_or_matchtemplate': [
            name for name in dir(cv2)
            if 'crosscorr' in name.lower() or 'matchtemplate' in name.lower()],
        'build_details': [line.strip() for line in info.splitlines()
                          if any(key in line for key in (
                              'Built as dynamic libs', 'C++ Compiler',
                              'C++ flags (Release)', 'Intel IPP:'))],
        'default_pkg_config': {'exit_code': pkg.returncode,
                               'stdout': pkg.stdout, 'stderr': pkg.stderr},
        'upstream_source_sha256': sha(cpp),
        'upstream_source_is_installed_binary_provenance_proof': False,
        'source_marker_lines': locations,
        'python_source_locations': python_locations,
        'match_template_executions': 0, 'controller_executions': 0,
        'benchmarks': 0, 'replays': 0, 'physical_audits': 0,
        'tests_run_this_round': 0,
        'exactness_proved_for_new_implementation': False,
        'performance_improvement_measured_this_round': False,
        'navigation_acceptance': False, 'real_time_budget_proved': False,
    }
    with Path(output).open('x') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main(sys.argv[1])
