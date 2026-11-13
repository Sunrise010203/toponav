"""Read-only native-pixel appearance with a rectified repetition veto.

Ground tracking still uses frozen G. Its appearance footprint is now a native
11px patch, not a stretched rectified 11px patch. This changes the descriptor
policy, NOT just its implementation, despite equal numerical NCC thresholds.
No physical/static-scene/calibration certification or actuator authorization.
"""
import cv2
import numpy as np

from roadvision.corridor_rgb_probe import (
    POLICY, _project, _homography, _grid_count, _spread_ratio,
)
from roadvision.rgb_ground_rig import _rectification, _region_models, shared_motion
from roadvision.rgb_motion_checks import (
    CHECK_POLICY, _patch, _ncc, _alternative, split_tracks,
    anchored_support, probe_motion_checks,
)

SAMPLING_POLICY = {
    'appearance_canvas': 'native_first_image', 'patch_px': 11,
    'target_resampling_passes': 1, 'rectified_self_repetition_veto': True,
    'minimum_ncc': .70, 'minimum_margin': .06, 'self_similarity_max': .94,
    'exclude_radius_px': 11.,
}


def prepare_canvases(images, warp, alignment):
    """Compose rectification and train-only alignment before RGB resampling.

    All appearance endpoints are expressed back in native first-image pixels.
    A remapped all-ones image tracks interpolation support; padded pixels must
    not supply an accepted patch or a global competing patch.
    """
    gray=[cv2.cvtColor(np.ascontiguousarray(im),cv2.COLOR_RGB2GRAY) for im in (images[0],images[2])]
    h=warp@alignment@np.linalg.inv(warp)
    gray[1]=cv2.warpPerspective(gray[1],h,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    ones=np.ones((128,128),np.float32)
    support=cv2.warpPerspective(ones,h,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    return gray,[np.ones((128,128),np.uint8),(support>=1.-1e-6).astype(np.uint8)]


def _supported_patch(gray, point, mask):
    patch=_patch(gray,point)
    if patch is None:return None
    valid=cv2.getRectSubPix(mask.astype(np.float32),(11,11),tuple(map(float,point)))
    return patch if np.all(valid>=1.-1e-6) else None


def _masked_alternative(template, target, expected, mask):
    scores=cv2.matchTemplate(target.astype(np.float32),template,cv2.TM_CCOEFF_NORMED)
    # A score-map top-left indexes a full 11px footprint, not its centre.
    support=cv2.erode(mask,np.ones((11,11),np.uint8),borderType=cv2.BORDER_CONSTANT,borderValue=0)[5:-5,5:-5]
    y,x=np.indices(scores.shape)
    outside=(x+5-expected[0])**2+(y+5-expected[1])**2>11.**2
    valid=outside & (support>0) & np.isfinite(scores)
    return (float(scores[valid].max()) if valid.any() else None,int(valid.sum()))


def appearance_evidence(first, second, source, target, first_mask, second_mask, *, _searches=None):
    """Same numerical H gates; bidirectional full-canvas valid competition."""
    a=_supported_patch(first,source,first_mask);b=_supported_patch(second,target,second_mask)
    result=dict(accepted=False,observed_ncc=None,alternative_ncc=None,
                reverse_alternative_ncc=None,self_alternative_ncc=None,margin=None)
    if a is None or b is None:
        result['reason']='patch_boundary_flat_or_padding';return result
    observed=_ncc(a,b)
    if _searches is None:
        alt,n1=_masked_alternative(a,second,target,second_mask)
        reverse,n2=_masked_alternative(b,first,source,first_mask)
        self_a,n3=_masked_alternative(a,first,source,first_mask)
        self_b,n4=_masked_alternative(b,second,target,second_mask)
    else:
        first_search=_searches[0].at(source);second_search=_searches[1].at(target)
        alt,n1=second_search(a);reverse,n2=first_search(b)
        self_a,n3=first_search(a);self_b,n4=second_search(b)
    result['alternative_search_counts']=[n1,n2,n3,n4]
    if any(v is None for v in (observed,alt,reverse,self_a,self_b)):
        result['reason']='invalid_descriptor_search';return result
    self_score=max(self_a,self_b);margin=observed-max(alt,reverse)
    result.update(observed_ncc=observed,alternative_ncc=alt,reverse_alternative_ncc=reverse,
                  self_alternative_ncc=self_score,margin=margin)
    failures=[]
    if observed<CHECK_POLICY['minimum_ncc']:failures.append('appearance_changed')
    if margin<CHECK_POLICY['minimum_margin']:failures.append('competing_match')
    if self_score>=CHECK_POLICY['self_similarity_max']:failures.append('self_repetition')
    result.update(accepted=not failures,reason='+'.join(failures) if failures else 'unique_patch_support')
    return result


def rectified_repetition(gray, source, target):
    """Keep H's ground-canvas self-repetition veto, not its endpoint NCC gate.

    Native patches alone can make perspective-repeated ground look unique.
    The rectification remains a level-ground hypothesis, not a semantic mask.
    """
    a,b=_patch(gray[0],source),_patch(gray[1],target)
    if a is None or b is None:return dict(accepted=False,reason='unavailable_rectified_patch',self_alternative_ncc=None)
    values=[_alternative(a,gray[0],source),_alternative(b,gray[1],target)]
    if any(v is None for v in values):return dict(accepted=False,reason='invalid_rectified_search',self_alternative_ncc=None)
    score=max(values)
    return dict(accepted=score<CHECK_POLICY['self_similarity_max'],reason='self_repetition' if score>=CHECK_POLICY['self_similarity_max'] else 'no_detected_repetition',self_alternative_ncc=score)


def _finish(region, kept, evidence, alignment, inverse):
    out=dict(view=region['view'],rectified_ground_assumption=region['rectified_ground_assumption'],
             coverage_map=inverse.tolist(),status='unknown',reasons=[],tracks=kept,models={},
             descriptor_evidence=evidence,descriptor_alignment=alignment,
             stage_counts=dict(input_tracks=len(region['tracks']),descriptor_accepted=len(kept),retained=len(kept)))
    if len(kept)<POLICY['min_tracks']:
        out['reasons']=['insufficient_unambiguous_tracks'];return out
    a,b,c=[np.array([t[k] for t in kept],float) for k in ('first','middle','last')]
    if (min(_grid_count(_project(inverse,p)) for p in (a,c))<POLICY['min_grid_cells'] or
            min(_spread_ratio(p) for p in (a,c))<POLICY['min_spread_ratio']):
        out['reasons']=['poor_spatial_support'];return out
    out['models'],out['reasons']=_region_models(a,b,c,inverse)
    if not out['reasons']:out['status']='planar_flow_candidate'
    return out


def filter_region(images, region, *, repetition_veto=True, _prepared_search=False):
    rectified=region['rectified_ground_assumption'];warp=_rectification() if rectified else np.eye(3)
    tracking_inverse=np.linalg.inv(warp)
    coverage=np.array(region.get('coverage_map',tracking_inverse),float)
    tracks=region['tracks'];train,_=split_tracks(tracks);train_set=set(train)
    aligned=np.eye(3)
    if len(train)>=8:
        first,last=[np.array([tracks[i][k] for i in train],float) for k in ('first','last')]
        h,mask=_homography(first,last)
        if h is not None and mask.mean()>=POLICY['min_inlier_ratio']:aligned=tracking_inverse@h@warp
    gray,masks=prepare_canvases(images,warp,aligned)
    h=warp@aligned@tracking_inverse;inv_h=np.linalg.inv(h)
    alignment=dict(homography=aligned.tolist(),native_homography=h.tolist(),fit_track_indices=train,
                   scope='training-coordinate prealignment only; one native target resampling')
    canonical=None
    if rectified:
        # Deliberately retain the exact old H self-check canvas as a veto.
        canonical=[cv2.cvtColor(np.ascontiguousarray(im),cv2.COLOR_RGB2GRAY) for im in (images[0],images[2])]
        canonical=[cv2.warpPerspective(im,warp,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP) for im in canonical]
        canonical[1]=cv2.warpPerspective(canonical[1],aligned,(128,128),flags=cv2.INTER_LINEAR|cv2.WARP_INVERSE_MAP)
    searches=None
    if _prepared_search:
        from roadvision.prepared_descriptor_search import PreparedAlternative
        searches=tuple(PreparedAlternative(im,patch_px=11,half=5,
            exclude_radius=11.,mask=mask) for im,mask in zip(gray,masks))
    evidence=[];kept=[]
    for i,track in enumerate(tracks):
        p=np.array(track['first'],float);mapped=_project(inv_h,np.array([track['last']],float))
        e=(dict(accepted=False,reason='invalid_prealignment_projection') if mapped is None
           else appearance_evidence(*gray,p,mapped[0],*masks,**({'_searches':searches} if _prepared_search else {})))
        if canonical is not None and mapped is not None:
            cp,cq=_project(tracking_inverse,np.array([p,mapped[0]]))
            check=rectified_repetition(canonical,cp,cq);e['rectified_repetition']=check
            if repetition_veto and not check['accepted']:
                e['accepted']=False;e['reason']='rectified_self_repetition' if check['reason']=='self_repetition' else check['reason']
        e['track_index']=i;evidence.append(e)
        if e['accepted']:kept.append({**track,'feature_split':'train' if i in train_set else 'validation'})
    return _finish(region,kept,evidence,alignment,coverage)


def _source_regions(baseline):
    for key,region in baseline['regions'].items():
        yield key,region
        if key.endswith('_raw'):
            yield region['view']+'_upper',{**region,
                'tracks':[t for t in region['tracks'] if all(8<=t[k][1]<80 for k in ('first','middle','last'))],
                'coverage_map':[[1.,0.,0.],[0.,128/72,-8*128/72],[0.,0.,1.]]}


def _explain(regions):
    models={k:dict(view=r['view'],plane_hypotheses=r['models']['plane_hypotheses'])
            for k,r in regions.items() if r['status']=='planar_flow_candidate'}
    return dict(regions=regions,shared_motion=shared_motion(models),anchored_support=anchored_support(regions))


def probe_resampling_checks(frames, *, now_s, calibration_id):
    # H performs strict input validation; retain its entire unmodified result.
    baseline=probe_motion_checks(frames,now_s=now_s,calibration_id=calibration_id)
    regions={};without_veto={}
    for key,region in _source_regions(baseline['baseline']):
        y,x={'front':(0,0),'left':(0,128),'right':(128,0)}[region['view']]
        images=[f['rgb'][y:y+128,x:x+128] for f in frames]
        regions[key]=filter_region(images,region)
        without_veto[key]=(filter_region(images,region,repetition_veto=False)
                           if region['rectified_ground_assumption'] else regions[key])
    return dict(version='rgb_resampling_checks_v1',baseline=baseline,**_explain(regions),
                ablations=dict(without_rectified_repetition_veto=_explain(without_veto)),
                evidence_packet=baseline['evidence_packet'],review=baseline['review'],
                limitations=['native_footprint_changes_descriptor_policy','native_and_rectified_checks_can_reject_valid_motion',
                             'feature_holdout_not_independent_dataset','static_scene_calibration_sign_scale_unverified',
                             'single_plane_dynamic_motion_indistinguishable','no_control_authorization'])
