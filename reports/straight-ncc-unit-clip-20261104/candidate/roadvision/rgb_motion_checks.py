"""Read-only ambiguity rejection and held-out, anchored motion corroboration.

No scene/motion/plane labels, metric height, neural model, control or simulator
inputs. A shared explanation is NOT static-scene/ego-motion/arrival evidence.
"""
import math

import cv2
import numpy as np
from scipy.optimize import least_squares

from roadvision.corridor_rgb_probe import (
    K,POLICY,_project,_homography,_rotation_residual,_grid_count,_spread_ratio,plane_hypotheses,
)
from roadvision.rgb_ground_rig import (
    probe_ground_rig,_rectification,_region_models,rig_basis,shared_motion,
)

CHECK_POLICY={'patch_px':11,'minimum_ncc':.70,'minimum_margin':.06,
              'self_similarity_max':.94,'exclude_radius_px':11.,
              'minimum_split_tracks':16,'minimum_train':8,'minimum_validation':4,
              'validation_median_px':.75,'validation_p90_px':1.5,
              'validation_inlier_fraction':.8,'support_rotation_gain_px':.20,
              'joint_max_evaluations':60,'plane_separation_deg':20.,'max_reported_hypotheses':64}


def _patch(gray, point):
    size=CHECK_POLICY['patch_px'];half=size//2
    p=np.asarray(point,float)
    if p.shape!=(2,) or not np.isfinite(p).all() or np.any(p<half+1) or np.any(p>=128-half-1): return None
    patch=cv2.getRectSubPix(gray,(size,size),tuple(map(float,p))).astype(np.float32)
    return patch if float(patch.std())>=2. else None


def _ncc(a,b):
    a=a.astype(float).ravel();b=b.astype(float).ravel()
    a-=a.mean();b-=b.mean();den=np.linalg.norm(a)*np.linalg.norm(b)
    if den<1e-9:return None
    score=a@b/den
    # clip is bitwise identity here, including signed zero. Keep the original
    # operation for out-of-range/nonfinite scores and unusual scalar types.
    if type(score) is np.float64 and -1. <= score <= 1.:
        return float(score)
    return float(np.clip(score,-1,1))


def _alternative(template, target, expected):
    score=cv2.matchTemplate(target.astype(np.float32),template,cv2.TM_CCOEFF_NORMED)
    y,x=np.indices(score.shape);half=CHECK_POLICY['patch_px']//2
    outside=(x+half-expected[0])**2+(y+half-expected[1])**2>CHECK_POLICY['exclude_radius_px']**2
    valid=outside & np.isfinite(score)
    if not valid.any(): return None
    return float(np.max(score[valid]))


def patch_evidence(first,second,source,target,*,_searches=None):
    """Global alternative search, not merely a forward/backward local match.

    Endpoints use appearance-only normalized grayscale patches. Large local
    scale/rotation/lighting changes may reject valid matches; count that loss.
    """
    a,b=_patch(first,source),_patch(second,target)
    result={'accepted':False,'observed_ncc':None,'alternative_ncc':None,
            'reverse_alternative_ncc':None,'self_alternative_ncc':None,'margin':None}
    if a is None or b is None:
        result['reason']='patch_boundary_or_flat';return result
    observed=_ncc(a,b)
    if _searches is None:
        alt=_alternative(a,second,target);reverse=_alternative(b,first,source)
        self_a,self_b=_alternative(a,first,source),_alternative(b,second,target)
    else:
        first_search=_searches[0].at(source);second_search=_searches[1].at(target)
        alt=second_search(a);reverse=first_search(b)
        self_a,self_b=first_search(a),second_search(b)
    if any(v is None for v in (observed,alt,reverse,self_a,self_b)):
        result['reason']='invalid_descriptor_search';return result
    self_score=max(self_a,self_b)
    margin=observed-max(alt,reverse)
    result.update(observed_ncc=observed,alternative_ncc=alt,reverse_alternative_ncc=reverse,
                  self_alternative_ncc=self_score,margin=margin)
    failures=[]
    if observed<CHECK_POLICY['minimum_ncc']:failures.append('appearance_changed')
    if margin<CHECK_POLICY['minimum_margin']:failures.append('competing_match')
    if self_score>=CHECK_POLICY['self_similarity_max']:failures.append('self_repetition')
    result.update(accepted=not failures,reason='unique_patch_support' if not failures else '+'.join(failures))
    return result


def filter_region(images, region, *, _prepared_search=False):
    rectified=region['rectified_ground_assumption']
    warp=_rectification() if rectified else np.eye(3)
    gray=[cv2.cvtColor(np.ascontiguousarray(im),cv2.COLOR_RGB2GRAY) for im in (images[0],images[2])]
    if rectified:
        gray=[cv2.warpPerspective(im,warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP) for im in gray]
    inverse=np.array(region.get('coverage_map',np.linalg.inv(warp)),float)
    # Pixel tracking coordinates remain original for upper-image regions.
    tracking_inverse=np.linalg.inv(warp)
    tracks=region['tracks']; evidence=[]; kept=[]
    train,validation=split_tracks(tracks)
    aligned=np.eye(3)
    if len(train)>=8:
        first,last=[np.array([tracks[i][k] for i in train],float) for k in ('first','last')]
        h,mask=_homography(first,last)
        if h is not None and mask.mean()>=POLICY['min_inlier_ratio']:
            aligned=tracking_inverse@h@warp
            gray[1]=cv2.warpPerspective(gray[1],aligned,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    out_alignment={'homography':aligned.tolist(),'fit_track_indices':train,'scope':'training-coordinate prealignment only'}
    searches=None
    if _prepared_search:
        from roadvision.prepared_descriptor_search import PreparedAlternative
        searches=tuple(PreparedAlternative(im,patch_px=CHECK_POLICY['patch_px'],
            half=CHECK_POLICY['patch_px']//2,exclude_radius=CHECK_POLICY['exclude_radius_px']) for im in gray)
    for i,track in enumerate(tracks):
        p,q=_project(tracking_inverse,np.array([track['first'],track['last']],float))
        mapped=_project(np.linalg.inv(aligned),q.reshape(1,2))
        e=({'accepted':False,'reason':'invalid_prealignment_projection'} if mapped is None
           else patch_evidence(gray[0],gray[1],p,mapped[0],**({'_searches':searches} if _prepared_search else {})))
        e['track_index']=i;evidence.append(e)
        if e['accepted']:kept.append({**track,'feature_split':'train' if i in train else 'validation'})
    out={'view':region['view'],'rectified_ground_assumption':rectified,'status':'unknown',
         'coverage_map':inverse.tolist(),'reasons':[],'tracks':kept,'models':{},'descriptor_evidence':evidence,'descriptor_alignment':out_alignment,
         'stage_counts':{'input_tracks':len(tracks),'descriptor_accepted':len(kept),'retained':len(kept)}}
    if len(kept)<POLICY['min_tracks']:
        out['reasons']=['insufficient_unambiguous_tracks'];return out
    a,b,c=[np.array([t[k] for t in kept],float) for k in ('first','middle','last')]
    if (min(_grid_count(_project(inverse,p)) for p in (a,c))<POLICY['min_grid_cells'] or
            min(_spread_ratio(p) for p in (a,c))<POLICY['min_spread_ratio']):
        out['reasons']=['poor_spatial_support'];return out
    out['models'],out['reasons']=_region_models(a,b,c,inverse)
    if not out['reasons']:out['status']='planar_flow_candidate'
    return out


def split_tracks(tracks):
    """Fixed spatial order, alternating disjoint fit/check features.

    This is an internal same-window feature holdout, NOT a new dataset split.
    """
    if tracks and all('feature_split' in t for t in tracks):
        return ([i for i,t in enumerate(tracks) if t['feature_split']=='train'],
                [i for i,t in enumerate(tracks) if t['feature_split']=='validation'])
    order=sorted(range(len(tracks)),key=lambda i:(tracks[i]['first'][1],tracks[i]['first'][0],i))
    return order[::2],order[1::2]


def fit_plane_vector(source,target,rotation,direction):
    """Fit q in H=K(R+t q^T)K^-1, fixing R and unit t from another region."""
    a=np.column_stack((source,np.ones(len(source))))@np.linalg.inv(K).T
    b=np.column_stack((target,np.ones(len(target))))@np.linalg.inv(K).T
    rotated=a@rotation.T;t=np.asarray(direction,float)
    u,v=b[:,0]/b[:,2],b[:,1]/b[:,2]
    design=np.concatenate(((t[0]-u*t[2])[:,None]*a,(t[1]-v*t[2])[:,None]*a))
    rhs=np.concatenate((u*rotated[:,2]-rotated[:,0],v*rotated[:,2]-rotated[:,1]))
    if len(source)<4 or np.linalg.matrix_rank(design)<3:return None
    q=np.linalg.lstsq(design,rhs,rcond=None)[0]
    return q if np.isfinite(q).all() else None


def plane_projection(points,rotation,direction,plane):
    return _project(K@(rotation+np.outer(direction,plane))@np.linalg.inv(K),points)


def _residual_summary(predicted,target,rotation_predicted):
    if predicted is None or rotation_predicted is None:return {'accepted':False,'reason':'invalid_projection'}
    error=np.linalg.norm(predicted-target,axis=1)
    rotation=np.linalg.norm(rotation_predicted-target,axis=1)
    median=float(np.median(error));p90=float(np.quantile(error,.9))
    fraction=float(np.mean(error<=POLICY['ransac_px']))
    gain=float(np.median(rotation)-median)
    accepted=(median<=CHECK_POLICY['validation_median_px'] and p90<=CHECK_POLICY['validation_p90_px'] and
              fraction>=CHECK_POLICY['validation_inlier_fraction'] and gain>=CHECK_POLICY['support_rotation_gain_px'])
    return {'accepted':accepted,'median_px':median,'p90_px':p90,'inlier_fraction':fraction,
            'rotation_median_px':float(np.median(rotation)),'rotation_gain_px':gain}


def _prepare(key,region,audit=None):
    audit={} if audit is None else audit
    audit.update(region=key,accepted=False,stage="eligibility")
    # Weak single-view parallax is support-only. All other failures remain
    # excluded; the standalone 1px rotation gate is NOT lowered.
    if any(r!='rotation_or_low_parallax' for r in region['reasons']):return None
    tracks=region['tracks']
    audit.update(stage='feature_split',track_count=len(tracks))
    if len(tracks)<CHECK_POLICY['minimum_split_tracks']:return None
    train,validation=split_tracks(tracks)
    if len(train)<CHECK_POLICY['minimum_train'] or len(validation)<CHECK_POLICY['minimum_validation']:return None
    a,c=[np.array([t[k] for t in tracks],float) for k in ('first','last')]
    audit.update(stage='training_homography',train_indices=train,validation_indices=validation)
    h,inliers=_homography(a[train],c[train])
    if h is None or inliers.sum()<CHECK_POLICY['minimum_train'] or inliers.mean()<POLICY['min_inlier_ratio']:return None
    fit=np.array(train)[inliers]
    inverse=np.array(region.get('coverage_map',np.linalg.inv(_rectification()) if region['rectified_ground_assumption'] else np.eye(3)),float)
    if min(_grid_count(_project(inverse,a[fit])),_grid_count(_project(inverse,c[fit])))<4:return None
    if min(_spread_ratio(a[fit]),_spread_ratio(c[fit]))<POLICY['min_spread_ratio']:return None
    _,rotation=_rotation_residual(a[fit],c[fit])
    rotation_predicted=_project(K@rotation@np.linalg.inv(K),a[validation])
    metrics=_residual_summary(_project(h,a[validation]),c[validation],rotation_predicted)
    audit.update(stage='heldout_plane',metrics=metrics)
    if not metrics['accepted']:return None
    audit.update(accepted=True,stage='prepared')
    return {'key':key,'view':region['view'],'standalone':region['status']=='planar_flow_candidate','a':a,'c':c,'fit':fit,'validation':validation,
            'homography':h,'rotation_prediction':rotation_predicted,'metrics':metrics}


def _joint_pair(anchor,support,rotation,direction):
    """Optimize shared R/t + two plane vectors using FIT coordinates only."""
    pair={'accepted':False,'reason':'initialization_failed'}
    axis=int(np.argmax(np.abs(direction)));free=[i for i in range(3) if i!=axis]
    canonical=direction/direction[axis];canonical/=np.linalg.norm(canonical)
    def unit_t(params):
        t=np.ones(3);t[free]=params
        return t/np.linalg.norm(t)
    def camera(region,r,t):
        b=rig_basis(region['view']);return b.T@r@b,b.T@t
    vectors=[]
    for region in (anchor,support):
        rc,tc=camera(region,rotation,canonical);fit=region['fit']
        q=fit_plane_vector(region['a'][fit],region['c'][fit],rc,tc)
        if q is None:return pair
        vectors.extend(q)
    initial=np.r_[cv2.Rodrigues(rotation)[0].ravel(),canonical[free]/canonical[axis],vectors]
    def decode(x):return cv2.Rodrigues(x[:3])[0],unit_t(x[3:5])
    def residual(x):
        r,t=decode(x);errors=[]
        for j,region in enumerate((anchor,support)):
            rc,tc=camera(region,r,t);fit=region['fit']
            p=plane_projection(region['a'][fit],rc,tc,x[5+j*3:8+j*3])
            errors.extend((np.full((len(fit),2),1000.) if p is None else p-region['c'][fit]).ravel())
        return np.array(errors)
    fit=least_squares(residual,initial,loss='soft_l1',f_scale=.5,
                      max_nfev=CHECK_POLICY['joint_max_evaluations'])
    pair.update(optimizer_success=bool(fit.success),evaluations=int(fit.nfev))
    if not fit.success:
        pair['reason']='optimizer_not_converged';return pair
    r,t=decode(fit.x);normals=[];metrics=[]
    for j,region in enumerate((anchor,support)):
        rc,tc=camera(region,r,t);q=fit.x[5+j*3:8+j*3]
        if np.linalg.norm(q)<1e-8:return pair
        b=rig_basis(region['view']);normals.append(b@q/np.linalg.norm(q))
        val=region['validation']
        metrics.append(_residual_summary(plane_projection(region['a'][val],rc,tc,q),region['c'][val],region['rotation_prediction']))
    separation=math.degrees(math.acos(float(np.clip(abs(normals[0]@normals[1]),0,1))))
    pair.update(anchor_validation=metrics[0],support_validation=metrics[1],plane_separation_deg=separation,
                fit_indices={r['key']:r['fit'].tolist() for r in (anchor,support)},
                validation_indices={r['key']:r['validation'] for r in (anchor,support)},sign_and_scale_unresolved=True)
    pair['accepted']=all(m['accepted'] for m in metrics) and separation>=CHECK_POLICY['plane_separation_deg']
    pair['reason']='heldout_two_plane_corroboration' if pair['accepted'] else 'heldout_or_plane_diversity_failed'
    return pair


def anchored_support(regions):
    prepared=[];checks=[]
    for key,region in regions.items():
        check={};p=_prepare(key,region,check);checks.append(check)
        if p is not None:prepared.append(p)
    result={'status':'unknown','hypotheses':[],'prepared_regions':[],'region_checks':checks,'pair_checks':[], 'metric_scale':None,
            'scope':'held-out anchored residual corroboration, not ego motion or physical geometry'}
    for p in prepared:
        result['prepared_regions'].append({'region':p['key'],'train_indices':p['fit'].tolist(),
                                          'validation_indices':p['validation'],'metrics':p['metrics']})
    solutions=[]
    for anchor in prepared:
        if not anchor['standalone'] or anchor['metrics']['rotation_median_px']<=POLICY['min_rotation_residual_px']:continue
        for bi,branch in enumerate(plane_hypotheses(anchor['homography'])):
            t=np.array(branch['translation_over_plane_distance']);n=np.array(branch['normal_camera'])
            if np.linalg.norm(t)<1e-8 or np.linalg.norm(n)<1e-8:continue
            b=rig_basis(anchor['view']);r=b@np.array(branch['rotation'])@b.T
            direction=b@t/np.linalg.norm(t);normal=b@n/np.linalg.norm(n)
            for support in prepared:
                if support['view']==anchor['view']:continue
                pair=_joint_pair(anchor,support,r,direction)
                pair.update(anchor=anchor['key'],anchor_branch=bi,support=support['key'])
                result['pair_checks'].append(pair)
                if pair['accepted']:solutions.append(pair)
    result.update(compatible_branch_count=len(solutions),hypotheses=solutions[:CHECK_POLICY['max_reported_hypotheses']],
                  hypotheses_truncated=len(solutions)>CHECK_POLICY['max_reported_hypotheses'])
    if solutions:result['status']='anchored_motion_candidate'
    return result


def probe_motion_checks(frames,*,now_s,calibration_id):
    baseline=probe_ground_rig(frames,now_s=now_s,calibration_id=calibration_id)
    regions={}
    origins={'front':(0,0),'left':(0,128),'right':(128,0)}
    for key,region in baseline['regions'].items():
        y,x=origins[region['view']]
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        regions[key]=filter_region(images,region)
        if key.endswith('_raw'):
            # Fixed upper-image hypothesis, not a simulator wall mask. It may
            # contain sky/ground/other objects when the level assumption fails.
            upper={**region,'tracks':[t for t in region['tracks'] if all(8<=t[k][1]<80 for k in ('first','middle','last'))],
                   'coverage_map':[[1.,0.,0.],[0.,128/72,-8*128/72],[0.,0.,1.]]}
            regions[region['view']+'_upper']=filter_region(images,upper)
    models={k:{'view':r['view'],'plane_hypotheses':r['models']['plane_hypotheses']}
            for k,r in regions.items() if r['status']=='planar_flow_candidate'}
    anchored=anchored_support(regions)
    return {'version':'rgb_motion_checks_v1','baseline':baseline,'regions':regions,
            'shared_motion':shared_motion(models),'anchored_support':anchored,
            'ablations':{'shared_without_upper':shared_motion({k:v for k,v in models.items() if not k.endswith('_upper')}),
                         'anchored_without_upper':any(p['accepted'] and not p['anchor'].endswith('_upper') and not p['support'].endswith('_upper') for p in anchored['pair_checks'])},
            'evidence_packet':baseline['evidence_packet'],'review':baseline['review'],
            'limitations':['patch_descriptor_may_reject_valid_scale_or_lighting_change',
                          'feature_holdout_not_independent_dataset','low_parallax_support_not_standalone_acceptance',
                          'sign_scale_and_dynamic_scene_ambiguity_unresolved','calibration_assumed_not_verified']}
