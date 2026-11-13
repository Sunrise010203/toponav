"""Bounded RGB floor geometry for the declared unmarked Gazebo calibration.

No simulator, topology, reference XY, odometry or pose imports. This is NOT a
learned/general floor detector: gray-blue floor and declared horizontal rig.
"""
import math
import numpy as np

VIEW_OFFSETS = dict(front=0.,left=math.pi/2,right=-math.pi/2,back=math.pi)


def split_views(rgb):
    if not isinstance(rgb,np.ndarray) or rgb.dtype!=np.uint8 or rgb.shape!=(256,256,3):
        raise ValueError('invalid_four_view_rgb')
    return {v:rgb[(i//2)*128:(i//2+1)*128,(i%2)*128:(i%2+1)*128] for i,v in enumerate(VIEW_OFFSETS)}


def floor_mask(rgb):
    r,g,b=rgb.astype(float).transpose(2,0,1)
    return (r>50)&(g-r>9)&(b-g>10)&(b-r>27)


def enclosed_side_walls(rgb):
    """Both lateral views see the calibrated close gray walls, not a branch.

    Independent geometry evidence of departure; never a model observation or
    a topology/node identity. Black/missing/blue inputs do not satisfy it.
    """
    views=split_views(rgb)
    for view in ('left','right'):
        r,g,b=views[view].astype(float).transpose(2,0,1)
        wall=(r>75)&(r<210)&(np.abs(r-g)<15)&(np.abs(g-b)<15)
        if wall.mean()<.95 or floor_mask(views[view]).mean()>.001:return False
    return True


def _fit(points, sign, width, camera_offset, centered=False):
    minimum=4 if centered else 7
    span=.08 if centered else .20
    if len(points)<minimum:return None
    p=np.asarray(points);x,y=p.T
    if np.ptp(x)<span:return None
    best=None
    for i in range(len(p)-3):
        for j in range(i+3,len(p)):
            if abs(x[j]-x[i])<span:continue
            a=(y[j]-y[i])/(x[j]-x[i]);b=y[i]-a*x[i]
            if abs(a)>2.5:continue
            # A vertical image silhouette at a junction projects to a line
            # through the optical center, NOT a corridor floor/wall join.
            # Reject it before voting, using only declared width/calibration.
            offset=(b-a*camera_offset)/math.sqrt(1+a*a)-sign*width/2
            if centered and abs(offset)>.23:continue
            residual=np.abs(y-a*x-b)/math.sqrt(1+a*a)
            good=residual<((.008+.008*x) if centered else (.018+.015*x))
            score=int(good.sum())
            if best is None or score>best[0]:best=(score,good)
    if best is None or best[0]<minimum:return None
    xg,yg=x[best[1]],y[best[1]]
    if np.ptp(xg)<span:return None
    a,b=np.polyfit(xg,yg,1)
    return float(a),float(b),int(len(xg))


def corridor(rgb, height=.45, width=.8, camera_offset=0., centered=False):
    mask=floor_mask(rgb);left=[];right=[]
    for v in range(69,126):
        u=np.flatnonzero(mask[v])
        if len(u)<3:continue
        x=height*64/(v-63.5)
        if x>4.:continue
        if u[0]>1:left.append((x,(63.5-u[0])*x/64))
        if u[-1]<126:right.append((x,(63.5-u[-1])*x/64))
    l,r=_fit(left,1,width,camera_offset,centered),_fit(right,-1,width,camera_offset,centered)
    if l is None and r is None:return None
    if l and r and abs(math.atan(l[0])-math.atan(r[0]))<.15:
        a=(l[0]+r[0])/2
        measured=(l[1]-r[1])/math.sqrt(1+a*a)
        if .60*width<measured<1.30*width:
            b=(l[1]+r[1])/2
            return dict(angle=math.atan(a),offset=(b-a*camera_offset)/math.sqrt(1+a*a),width=measured,support=l[2]+r[2],two_sided=True)
    # One exposed edge plus declared width. Junctions can hide the other edge.
    candidates=[]
    for fit,sign in [(l,1),(r,-1)]:
        if fit:
            a,b,n=fit
            offset=(b-a*camera_offset)/math.sqrt(1+a*a)-sign*width/2
            if abs(offset)<.65:
                candidates.append(dict(angle=math.atan(a),offset=offset,width=width,support=n,two_sided=False))
    return max(candidates,key=lambda c:c['support']) if candidates else None


def front_wall_range(rgb, height=.45, camera_offset=0.):
    """Range to visible floor/wall join; None if hidden/outside camera FOV."""
    mask=floor_mask(rgb)
    rows=[v for v in range(65,128) if np.count_nonzero(mask[v,48:80])>=24]
    if len(rows)<2:return None
    return float(height*64/(min(rows)-63.5)+camera_offset)


def measurements(rgb, height=.45, width=.8, camera_offset=0.):
    result={}
    for view,pixels in split_views(rgb).items():
        m=corridor(pixels,height,width,camera_offset)
        if view in ('front','back') and (m is None or abs(m['angle'])>math.radians(25) or abs(m['offset'])>.23):
            near=corridor(pixels,height,width,camera_offset,centered=True)
            if near is not None:m=near
        result[view]=m
    return result


def incoming_geometry(measured):
    candidates=[]
    for view,sign in [('front',1),('back',-1)]:
        m=measured[view]
        if m and abs(m['angle'])<math.radians(25) and abs(m['offset'])<.23:
            candidates.append((view,dict(angle=m['angle'],offset=sign*m['offset'],support=m['support'])))
    # Behind is less affected by the approaching junction. Never use pose.
    if not candidates:return None
    return next((m for v,m in candidates if v=='back'),candidates[0][1])


def station_distance(measured, relative_open, front_rgb, height=.45, width=.8, camera_offset=0.):
    candidates=[]
    for view,sign in [('left',-1),('right',1)]:
        m=measured[view]
        if relative_open.get(view) and m and abs(m['angle'])<math.radians(18):
            candidates.append(sign*m['offset'])
    if candidates and max(candidates)-min(candidates)<.12:
        return float(np.mean(candidates)), 'side_corridor_center'
    if not relative_open.get('front'):
        distance=front_wall_range(front_rgb,height,camera_offset)
        if distance is not None:
            # Dead-end station has a conservative .06m stand-off before the
            # nominal cap-center, whose inner wall is half a corridor away.
            return distance-(width/2+.06), 'front_cap_standoff'
    return None,None
