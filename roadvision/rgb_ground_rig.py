"""Read-only image tracking and conditional common-rig motion hypotheses.

No simulator, scene label, metric height, model or actuator input. All physical
outputs remain unknown. Co-located views provide no stereo baseline. A ground
warp is a level-plane *hypothesis*, not a segmentation/ground observation.
OpenCV RANSAC is seeded and must be called serially, like the frozen probe.
"""
import itertools
import math

import cv2
import numpy as np

from roadvision.corridor_rgb_probe import (
    POLICY, _unique_corners, _lk, _project, _grid_count, _spread_ratio,
    probe_rgb_window, _homography, _rotation_residual, plane_hypotheses,
)

RIG_POLICY = {'rotation_gap_deg': 3., 'translation_axis_gap_deg': 12.,
              'plane_separation_deg': 20., 'max_reported_hypotheses': 64}


def ground_map():
    """(lateral/height, forward/height, 1) -> original image homogeneous pixel.

    y_camera/height=1; no numerical camera height is supplied or recovered.
    """
    return np.array([[64., 64., 0.], [0., 64., 64.], [0., 1., 0.]])


def _rectification():
    canvas_to_ground = np.array([[3./128, 0., -1.5], [0., -3.3/128, 4.5], [0., 0., 1.]])
    return ground_map() @ canvas_to_ground


def rig_basis(view):
    """Optical x-right/y-down/z-forward to rig x-forward/y-left/z-up."""
    angle = {'front': 0., 'left': math.pi/2, 'right': -math.pi/2, 'back': math.pi}[view]
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[s, 0., c], [-c, 0., s], [0., -1., 0.]])


def _advance(first_image, second_image, points, live, seed=None):
    """Never track failed coordinates or send failed placeholders downstream."""
    target, valid = points.copy(), np.zeros(len(points), bool)
    if live.any():
        p = points[live].astype(np.float32)
        if seed is None:
            q, ok = _lk(first_image, second_image, p)
        else:
            initial = (p + np.array(seed, np.float32)).reshape(-1,1,2)
            q, status, _ = cv2.calcOpticalFlowPyrLK(
                first_image, second_image, p.reshape(-1,1,2), initial,
                flags=cv2.OPTFLOW_USE_INITIAL_FLOW, winSize=(POLICY['lk_window_px'],)*2,
                maxLevel=POLICY['lk_levels'], criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                POLICY['lk_iterations'], POLICY['lk_epsilon']))
            if q is None or status is None:
                q, ok = p.copy(), np.zeros(len(p),bool)
            else:
                q = q.reshape(-1,2)
                ok = status.ravel().astype(bool) & np.isfinite(q).all(axis=1)
                ok &= (q >= 4).all(axis=1) & (q < 124).all(axis=1)
                q[~ok] = 0
        target[live], valid[live] = q, ok
    return target, valid


def _region_models(first, middle, last, coverage_map):
    reasons, diagnostics = [], {}
    h, inliers = _homography(first, last)
    diagnostics["homography_inlier_count"] = int(inliers.sum())
    diagnostics["homography_inlier_ratio"] = float(inliers.mean())
    diagnostics["homography_inlier_mask"] = inliers.tolist()
    if h is None or inliers.sum() < POLICY["min_tracks"] or inliers.mean() < POLICY["min_inlier_ratio"]:
        return diagnostics, ["planar_model_unsupported"]
    # Six cells are counted in the fixed ROI (not the whole original frame)
    # for ground regions; spread/flow/residual remain ORIGINAL pixel units.
    # This is a new coverage definition, NOT an unchanged frozen threshold.
    # Same cardinality does not imply identical acceptance strictness.
    if (min(_grid_count(_project(coverage_map, first[inliers])), _grid_count(_project(coverage_map, last[inliers]))) < POLICY["min_grid_cells"] or
            min(_spread_ratio(first[inliers]), _spread_ratio(last[inliers])) < POLICY["min_spread_ratio"]):
        return diagnostics, ["planar_inliers_poorly_distributed"]
    predicted = _project(h, first[inliers])
    if predicted is None:
        return diagnostics, ["homography_projection_invalid"]
    diagnostics["homography"] = h.tolist()
    diagnostics["homography_residual_median_px"] = float(np.median(np.linalg.norm(predicted - last[inliers], axis=1)))
    diagnostics["plane_hypotheses"] = plane_hypotheses(h)
    first_h, first_mask = _homography(first, middle)
    last_h, last_mask = _homography(middle, last)
    if (first_h is None or last_h is None or min(first_mask.mean(), last_mask.mean()) < POLICY["min_inlier_ratio"]):
        reasons.append("adjacent_planar_models_unsupported")
        diagnostics["composition_median_px"] = None
    else:
        composed = _project(last_h @ first_h, first[inliers])
        error = None if composed is None else float(np.median(np.linalg.norm(composed - predicted, axis=1)))
        diagnostics["composition_median_px"] = error
        if error is None or error > POLICY["composition_max_px"]:
            reasons.append("temporal_models_inconsistent")
    displacement = float(np.median(np.linalg.norm(last[inliers] - first[inliers], axis=1)))
    residual, rotation = _rotation_residual(first[inliers], last[inliers])
    diagnostics.update(displacement_median_px=displacement, rotation_residual_median_px=residual,
                       best_fit_rotation=rotation.tolist())
    if displacement < POLICY["min_displacement_px"]:
        reasons.append("insufficient_image_displacement")
    if residual is None or residual <= POLICY["min_rotation_residual_px"]:
        reasons.append("rotation_or_low_parallax")
    return diagnostics, reasons


def track_region(images, *, rectified=False):
    if len(images) != 3 or any(im.shape != (128,128,3) or im.dtype != np.uint8 for im in images):
        raise ValueError('Expected three uint8 128x128 RGB views')
    gray = [cv2.cvtColor(np.ascontiguousarray(im), cv2.COLOR_RGB2GRAY) for im in images]
    warp = _rectification() if rectified else np.eye(3)
    if rectified:
        gray = [cv2.warpPerspective(im, warp, (128,128), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
                for im in gray]
    raw, first, ambiguous = _unique_corners(gray[0])
    out = {'status':'unknown', 'reasons':[], 'rectified_ground_assumption':rectified,
           'stage_counts': {'detected':len(raw), 'unique':len(first), 'ambiguous':ambiguous},
           'tracks':[], 'models':{}}
    counts = out['stage_counts']
    seeds = {}
    if rectified:
        window = cv2.createHanningWindow((128,128), cv2.CV_32F)
        for i,j in ((0,1),(1,0),(1,2),(2,1),(0,2),(2,0)):
            shift, response = cv2.phaseCorrelate(gray[i].astype(np.float32),gray[j].astype(np.float32),window)
            # Initial guess only; all LK/FB/path/model gates still apply.
            if np.isfinite(shift).all() and max(abs(v) for v in shift) < 48:
                seeds[(i,j)] = shift
        out['phase_seeds_px'] = {str(k):list(v) for k,v in seeds.items()}
    def advance(i,j,points,live):
        return _advance(gray[i],gray[j],points,live,seeds.get((i,j)))
    live = np.ones(len(first), bool)
    if rectified and len(first):
        original = _project(warp, first)
        live &= (original >= 6).all(axis=1) & (original < 122).all(axis=1)
    counts['roi_valid'] = int(live.sum())
    middle, live = advance(0,1,first,live)
    counts['lk01'] = int(live.sum())
    back0, ok = advance(1,0,middle,live)
    live &= ok
    counts['lk10'] = int(live.sum())
    last, live = advance(1,2,middle,live)
    counts['lk12'] = int(live.sum())
    back1, ok = advance(2,1,last,live)
    live &= ok
    counts['lk21'] = int(live.sum())
    direct, ok = advance(0,2,first,live)
    live &= ok
    counts['lk02'] = int(live.sum())
    back2, ok = advance(2,0,direct,live)
    live &= ok
    counts['lk20'] = int(live.sum())
    # Consistency thresholds are evaluated in ORIGINAL image pixels even when
    # tracking is rectified. No relaxation by changing the pixel unit.
    coordinates = [_project(warp, p) if len(p) else p.copy()
                   for p in (first, middle, last, back0, back1, direct, back2)]
    if any(p is None for p in coordinates):
        out['reasons'] = ['invalid_projection']
        counts['fb'] = counts['path'] = counts['retained'] = 0
        return out
    a,b,c,ba,bb,d,bd = coordinates
    if len(a):
        for p in (a,b,c,d):
            live &= (p >= 4).all(axis=1) & (p < 124).all(axis=1)
    counts['original_bounds'] = int(live.sum())
    fb = np.maximum.reduce([np.linalg.norm(ba-a,axis=1), np.linalg.norm(bb-b,axis=1),
                            np.linalg.norm(bd-a,axis=1)])
    path = np.linalg.norm(d-c,axis=1)
    live &= fb <= POLICY['fb_max_px']
    counts['fb'] = int(live.sum())
    live &= path <= POLICY['path_max_px']
    counts['path'] = counts['retained'] = int(live.sum())
    a,b,c,fb,path = [p[live] for p in (a,b,c,fb,path)]
    out['tracks'] = [{'first':p.tolist(),'middle':q.tolist(),'last':r.tolist(),
                      'fb_max_px':float(f),'path_error_px':float(e)} for p,q,r,f,e in zip(a,b,c,fb,path)]
    if len(a) < POLICY['min_tracks']:
        out['reasons'] = ['insufficient_consistent_tracks']
        return out
    out['grid_cells'] = {'first':_grid_count(_project(np.linalg.inv(warp),a)), 'last':_grid_count(_project(np.linalg.inv(warp),c))}
    out['coverage_coordinate_system'] = 'fixed_rectified_roi' if rectified else 'original_image'
    out['spread_ratio'] = {'first':_spread_ratio(a), 'last':_spread_ratio(c)}
    if min(out['grid_cells'].values()) < POLICY['min_grid_cells'] or min(out['spread_ratio'].values()) < POLICY['min_spread_ratio']:
        out['reasons'] = ['poor_spatial_support']
        return out
    out['models'], out['reasons'] = _region_models(a,b,c,np.linalg.inv(warp))
    if not out['reasons']:
        out['status'] = 'planar_flow_candidate'
    return out


def _line_angle(a,b):
    return math.degrees(math.acos(float(np.clip(abs(np.dot(a,b)), 0., 1.))))


def _rotation_angle(a,b):
    return math.degrees(math.acos(float(np.clip((np.trace(a.T @ b)-1)/2, -1., 1.))))


def shared_motion(regions):
    """Enumerate jointly compatible decomposition branches (sign/scale free).

    Each branch pair must agree in rotation and unoriented translation axis;
    >=2 views and >=2 nonparallel normals are required. t/d magnitudes are NOT
    equalized across planes. No cheirality, metric scale, static-scene proof,
    calibration certification or unique motion solution is claimed.
    """
    branches = {}
    for key, region in regions.items():
        b = rig_basis(region['view'])
        values = []
        for index, branch in enumerate(region['plane_hypotheses']):
            r = np.asarray(branch['rotation'],float)
            t = np.asarray(branch['translation_over_plane_distance'],float)
            n = np.asarray(branch['normal_camera'],float)
            if (r.shape != (3,3) or t.shape != (3,) or n.shape != (3,) or
                    not all(np.isfinite(v).all() for v in (r,t,n))):
                continue
            if np.linalg.norm(t) < 1e-8 or np.linalg.norm(n) < 1e-8:
                continue
            values.append({'region':key,'view':region['view'],'branch':index,
                           'r':b@r@b.T, 't':b@t/np.linalg.norm(t), 'n':b@n/np.linalg.norm(n)})
        if values: branches[key] = values
    result = {'status':'unknown','max_supported_regions':0,'hypotheses':[], 'metric_scale':None,
              'eligible_regions':list(branches), 'scope':'conditional static-rig explanation, not ego-motion evidence'}
    best, solutions = 0, []
    def visit(keys, chosen):
        nonlocal best, solutions
        if len(chosen)+len(keys) < max(2,best): return
        if not keys:
            if len(chosen) < 2 or len({v['view'] for v in chosen}) < 2: return
            separation = max(_line_angle(a['n'],b['n']) for a,b in itertools.combinations(chosen,2))
            if separation < RIG_POLICY['plane_separation_deg']: return
            if len(chosen) > best: best, solutions = len(chosen), []
            if len(chosen) == best:
                pairs = list(itertools.combinations(chosen,2))
                solutions.append({'branches':[{'region':v['region'],'branch':v['branch']} for v in chosen],
                    'rotation_gap_max_deg':max(_rotation_angle(a['r'],b['r']) for a,b in pairs),
                    'translation_axis_gap_max_deg':max(_line_angle(a['t'],b['t']) for a,b in pairs),
                    'plane_separation_max_deg':separation})
            return
        key, *rest = keys
        for v in branches[key]:
            if all(_rotation_angle(v['r'],p['r']) <= RIG_POLICY['rotation_gap_deg'] and
                   _line_angle(v['t'],p['t']) <= RIG_POLICY['translation_axis_gap_deg'] for p in chosen):
                visit(rest,chosen+[v])
        visit(rest,chosen)
    visit(list(branches),[])
    result.update(max_supported_regions=best, compatible_branch_count=len(solutions),
                  hypotheses=solutions[:RIG_POLICY['max_reported_hypotheses']],
                  hypotheses_truncated=len(solutions)>RIG_POLICY['max_reported_hypotheses'])
    if solutions: result['status'] = 'shared_motion_candidate'
    result['reason'] = ('competing_sign_scale_branches_retained' if solutions else
                        'no_joint_multiview_multiplane_support')
    return result


def probe_ground_rig(frames, *, now_s, calibration_id):
    # Frozen strict validation and physical contract; baseline is also the
    # untouched original-image ablation, evaluated on the exact same pixels.
    baseline = probe_rgb_window(frames, now_s=now_s, calibration_id=calibration_id)
    regions, models = {}, {}
    for view,(y,x) in {'front':(0,0),'left':(0,128),'right':(128,0)}.items():
        images = [f['rgb'][y:y+128,x:x+128] for f in frames]
        for rectified in (False,True):
            key = view + ('_ground' if rectified else '_raw')
            region = track_region(images,rectified=rectified)
            region['view'] = view
            regions[key] = region
            if region['status'] == 'planar_flow_candidate':
                models[key] = {'view':view, 'plane_hypotheses':region['models']['plane_hypotheses']}
    return {'version':'rgb_ground_rig_v1', 'regions':regions, 'shared_motion':shared_motion(models),
            'ablations':{name:shared_motion({k:v for k,v in models.items() if k.endswith(suffix)})
                         for name,suffix in [('raw_only','_raw'),('ground_only','_ground')]},
            'baseline':baseline, 'evidence_packet':baseline['evidence_packet'], 'review':baseline['review'],
            'limitations':['level_calibration_assumed_not_verified','ground_warp_not_surface_identity',
                          'shared_motion_not_static_scene_or_ego_motion_proof','no_stereo_or_metric_scale',
                          'dynamic_single_plane_nonidentifiable']}
