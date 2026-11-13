"""Native lower-image ground HYPOTHESIS, not ground segmentation.

Corners are detected inside the declared image band, not selected afterward
from a whole-frame corner budget dominated by wall texture. Actual surface
identity must still survive cross-view metric fits and held-out checks.
"""
import cv2
import numpy as np

from roadvision.rgb_ground_rig import _advance, _region_models

BAND_POLICY=dict(x_min=12,x_max=116,y_min=104,y_max=119,corners=160,quality=.02,min_distance_px=4,
                 fb_max_px=.5,path_max_px=.75)


def track_ground_band(images,*,view):
    return track_fixed_roi(images,view=view,bounds=(12,116,104,119))


def track_fixed_roi(images,*,view,bounds):
    """Image-only fixed ROI; a hypothesis, never a ground/wall ID mask."""
    if view not in ('front','left','right','back'):raise ValueError('Unsupported view')
    if len(images)!=3 or any(im.shape!=(128,128,3) or im.dtype!=np.uint8 for im in images):raise ValueError('Expected three RGB views')
    gray=[cv2.cvtColor(np.ascontiguousarray(im),cv2.COLOR_RGB2GRAY) for im in images]
    x0,x1,y0,y1=bounds
    if not 4<=x0<x1<=124 or not 4<=y0<y1<=124:raise ValueError('Invalid fixed ROI')
    mask=np.zeros((128,128),np.uint8);mask[y0:y1,x0:x1]=255
    points=cv2.goodFeaturesToTrack(gray[0],mask=mask,maxCorners=160,qualityLevel=.02,minDistance=4,blockSize=5)
    a=np.empty((0,2),np.float32) if points is None else points.reshape(-1,2)
    live=np.ones(len(a),bool)
    b,live=_advance(gray[0],gray[1],a,live)
    ba,ok=_advance(gray[1],gray[0],b,live);live&=ok
    c,live=_advance(gray[1],gray[2],b,live)
    bb,ok=_advance(gray[2],gray[1],c,live);live&=ok
    direct,ok=_advance(gray[0],gray[2],a,live);live&=ok
    back,ok=_advance(gray[2],gray[0],direct,live);live&=ok
    fb=np.maximum.reduce([np.linalg.norm(ba-a,axis=1),np.linalg.norm(bb-b,axis=1),np.linalg.norm(back-a,axis=1)])
    path=np.linalg.norm(direct-c,axis=1)
    live&=(fb<=.5)&(path<=.75)
    # The ROI constrains DETECTION, not the motion of a tracked physical
    # point. Cropping every endpoint to the same band discarded correctly
    # receding ground features. Keep the actual image / horizon bounds;
    # appearance, FB/path, spatial and held-out metric gates still apply.
    for p in (a,b,c):live&=(p>=4).all(axis=1)&(p<124).all(axis=1)
    if y0>=88:
        for p in (b,c):live&=p[:,1]>72
    coverage=np.array([[128/(x1-x0),0,-x0*128/(x1-x0)],[0,128/(y1-y0),-y0*128/(y1-y0)],[0,0,1.]])
    out=dict(view=view,rectified_ground_assumption=False,status='unknown',reasons=[],models={},
             coverage_map=coverage.tolist(),fixed_roi=list(bounds),stage_counts=dict(detected=len(a),retained=int(live.sum())),
             tracks=[dict(first=p.tolist(),middle=q.tolist(),last=r.tolist(),fb_max_px=float(f),path_error_px=float(e))
                     for p,q,r,f,e in zip(a[live],b[live],c[live],fb[live],path[live])])
    if live.sum()<12:out['reasons']=['insufficient_ground_band_tracks'];return out
    out['models'],out['reasons']=_region_models(a[live],b[live],c[live],coverage)
    if not out['reasons']:out['status']='planar_flow_candidate'
    return out
