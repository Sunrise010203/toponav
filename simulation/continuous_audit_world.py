"""One-way physics-rate audit sidecar; never consumed by decision modules."""
import math
import numpy as np
import pybullet as p
from simulation.textured_corridor import TexturedCorridorWorld


class ContinuousAuditWorld(TexturedCorridorWorld):
    def __init__(self,*args,narrow_obstacle_width_m=.5,**kwargs):
        if type(narrow_obstacle_width_m) not in (int,float) or not math.isfinite(narrow_obstacle_width_m) or not .25<=narrow_obstacle_width_m<=.5:raise ValueError('Narrow blocker width outside declared .25–.5m domain')
        self.narrow_obstacle_width_m=narrow_obstacle_width_m
        self.physics_trace=[]
        super().__init__(*args,**kwargs)

    def _box(self,half,center,color):
        # Fixture-only construction parameter; NEVER part of RGB envelope.
        if self.config.obstacle=='narrow' and tuple(center)==(3.05,0,.8):
            half=(half[0],self.narrow_obstacle_width_m/2,half[2])
        return super()._box(half,center,color)

    def _step(self):
        super()._step()
        position,_=p.getBasePositionAndOrientation(self.robot,physicsClientId=self.client)
        linear,angular=p.getBaseVelocity(self.robot,physicsClientId=self.client)
        self.physics_trace.append(dict(sim_s=self.sim_s,position_m=list(position),
            speed_m_s=float(np.linalg.norm(linear[:2])),yaw_rate_rad_s=float(angular[2]),
            wall_contact_physics_steps=self.wall_contact_physics_steps))


class CornerAuditWorld(ContinuousAuditWorld):
    """Actual L-shaped static scene; the original robot/rig/motors are unchanged.

    Geometry is fixture/audit data only. Static walls are constructed before
    control starts, never by resetting robot pose/velocity during execution.
    """
    def __init__(self, *, assets, direction='left', width_m=1.8,
                 corner_x_m=2.5, exit_length_m=3.5, seed=313,
                 appearance=None, pattern='unique',junction='l',blocked_branch=None):
        from simulation.corridor_motion import CorridorConfig
        if direction not in ('left', 'right'):
            raise ValueError('Expected left/right corner')
        if junction not in ('l','t') or blocked_branch not in (None,'left','right') or (junction=='l' and blocked_branch is not None):
            raise ValueError('Invalid junction fixture/blocker')
        self.junction=junction;self.branch_blocker=None
        for value in (width_m, corner_x_m, exit_length_m):
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError('Expected finite corner dimensions')
        if not 1.6<=width_m<=2.2 or not 2.<=corner_x_m<=4. or not 2.8<=exit_length_m<=5.:
            raise ValueError('Outside declared L-corner fixture domain')
        self.corner_geometry=dict(direction=direction,width_m=width_m,
                                  corner_x_m=corner_x_m,exit_length_m=exit_length_m)
        super().__init__(CorridorConfig(length_m=4.,width_m=width_m),appearance,
                         assets=assets,seed=seed,pattern=pattern)
        try:
            for body in [self.floor,*self.walls]:
                p.removeBody(body,physicsClientId=self.client)
            h=width_m/2;outer=corner_x_m+h;inner=corner_x_m-h
            sign=1 if direction=='left' else -1
            def box(half,center):
                return self._box(half,(center[0],sign*center[1],center[2]),(.65,.62,.58))
            self.floor=box(((outer+.2)/2,h,.05),((outer-.2)/2,0.,-.05))
            self.exit_floor=box((h,(exit_length_m-h)/2,.05),
                                (corner_x_m,(exit_length_m+h)/2,-.05))
            self.walls=[
                box(((outer+.2)/2+.05,.05,.8),((outer-.2)/2,-h-.05,.8)),
                box(((inner+.2)/2,.05,.8),((inner-.2)/2,h+.05,.8)),
                box((.05,(exit_length_m+h)/2+.05,.8),(outer+.05,(exit_length_m-h)/2,.8)),
                box((.05,(exit_length_m-h)/2,.8),(inner-.05,(exit_length_m+h)/2,.8)),
                box((h,.05,.8),(corner_x_m,exit_length_m+.05,.8)),
                box((.05,h,.8),(-.25,0.,.8)),
            ]
            if junction=='t':
                for wall in self.walls:p.removeBody(wall,physicsClientId=self.client)
                p.removeBody(self.exit_floor,physicsClientId=self.client)
                # Adjacent slabs, not overlapping coplanar textured surfaces:
                # depth-buffer flicker would fabricate camera motion at rest.
                p.removeBody(self.floor,physicsClientId=self.client)
                self.floor=box(((inner+.2)/2,h,.05),((inner-.2)/2,0.,-.05))
                self.exit_floor=box((h,exit_length_m,.05),(corner_x_m,0.,-.05))
                self.walls=[
                    box(((inner+.2)/2,.05,.8),((inner-.2)/2,s*(h+.05),.8)) for s in (-1,1)]
                self.walls+=[box((.05,exit_length_m+.05,.8),(outer+.05,0.,.8))]
                self.walls+=[box((.05,(exit_length_m-h)/2,.8),(inner-.05,s*(exit_length_m+h)/2,.8)) for s in (-1,1)]
                self.walls+=[box((h,.05,.8),(corner_x_m,s*(exit_length_m+.05),.8)) for s in (-1,1)]
                self.walls+=[box((.05,h,.8),(-.25,0.,.8))]
                if blocked_branch is not None:
                    s=1 if blocked_branch=='left' else -1
                    self.branch_blocker=self._box((h,.06,.8),(corner_x_m,s*(h+.40),.8),(.65,.62,.58))
                    self.walls.append(self.branch_blocker)
        except Exception:
            self.close()
            raise

    def clear_branch_blocker(self):
        """Fixture event only; no scene event/labels enter RGB controller."""
        if self.branch_blocker is None:raise ValueError('No branch blocker to clear')
        self.walls.remove(self.branch_blocker)
        p.removeBody(self.branch_blocker,physicsClientId=self.client)
        self.branch_blocker=None


class RouteAuditWorld(ContinuousAuditWorld):
    """One continuous mapped corridor world, NEVER re-spawned between nodes.

    Public map photos are rendered before execution, using a separate nominal
    camera transform, not by teleporting the robot. Pose metadata stays here.
    """
    def __init__(self,*,physical_scene,assets,seed=8111,spawn_node,spawn_heading_deg,pattern='unique'):
        from simulation.corridor_motion import CorridorConfig
        self.route_scene=physical_scene
        geometry=physical_scene['geometry'];self.node_positions=geometry['node_positions_m']
        widths={d['width'] for d in geometry['channel_dimensions_m'].values()}
        if len(widths)!=1 or not 1.6<=next(iter(widths))<=2.2:raise ValueError('Route requires one declared fixture width')
        self.route_width=next(iter(widths))
        if spawn_node not in self.node_positions or type(spawn_heading_deg) not in (int,float) or spawn_heading_deg not in (0,90,180,270):raise ValueError('Invalid fixture spawn')
        centre=np.array(self.node_positions[spawn_node],float);angle=math.radians(spawn_heading_deg)
        self.spawn_xy=centre-2.*np.array([math.cos(angle),math.sin(angle)])
        self.spawn_heading_deg=spawn_heading_deg
        super().__init__(CorridorConfig(length_m=4.,width_m=self.route_width),assets=assets,seed=seed,pattern=pattern)

    def _construct_route_scene(self):
        from collections import defaultdict
        rectangles=[];h=self.route_width/2
        for channel in self.route_scene['channels']:
            a=np.array(self.node_positions[channel['source']],float);b=np.array(self.node_positions[channel['target']],float)
            if (abs(a[0]-b[0])<1e-8)==(abs(a[1]-b[1])<1e-8) or np.linalg.norm(a-b)<3.5:raise ValueError('Route fixture supports separated orthogonal channels only')
            rectangles.append((min(a[0],b[0])-h,max(a[0],b[0])+h,min(a[1],b[1])-h,max(a[1],b[1])+h))
        xs=sorted({v for a,b,c,d in rectangles for v in (a,b)});ys=sorted({v for a,b,c,d in rectangles for v in (c,d)})
        cells=set()
        for i in range(len(xs)-1):
            for j in range(len(ys)-1):
                x=(xs[i]+xs[i+1])/2;y=(ys[j]+ys[j+1])/2
                if any(a<x<b and c<y<d for a,b,c,d in rectangles):cells.add((i,j))
        if not any(a+.25<self.spawn_xy[0]<b-.25 and c+.25<self.spawn_xy[1]<d-.25 for a,b,c,d in rectangles):raise ValueError('Fixture spawn lacks initial body clearance')
        for body in [self.floor,*self.walls]:p.removeBody(body,physicsClientId=self.client)
        self.floor=self._box(((xs[-1]-xs[0])/2+.1,(ys[-1]-ys[0])/2+.1,.05),((xs[-1]+xs[0])/2,(ys[-1]+ys[0])/2,-.05),(.2,.29,.4))
        boundaries=defaultdict(list)
        for i,j in cells:
            for di,sign in [(-1,-1),(1,1)]:
                if (i+di,j) not in cells:boundaries[('v',xs[i if di<0 else i+1],sign)].append((ys[j],ys[j+1]))
            for dj,sign in [(-1,-1),(1,1)]:
                if (i,j+dj) not in cells:boundaries[('h',ys[j if dj<0 else j+1],sign)].append((xs[i],xs[i+1]))
        self.walls=[]
        for (axis,value,sign),intervals in sorted(boundaries.items()):
            merged=[]
            for a,b in sorted(intervals):
                if merged and abs(a-merged[-1][1])<1e-8:merged[-1]=(merged[-1][0],b)
                else:merged.append((a,b))
            for a,b in merged:
                half=(.05,(b-a)/2,.8) if axis=='v' else ((b-a)/2,.05,.8)
                centre=(value+sign*.05,(a+b)/2,.8) if axis=='v' else ((a+b)/2,value+sign*.05,.8)
                self.walls.append(self._box(half,centre,(.65,.62,.58)))

    def reference_atlas(self):
        """Map acquisition only. Returns labelled RGB, no metric/camera poses."""
        result=[];projection=p.computeProjectionMatrixFOV(90,1,.05,30)
        for node in self.route_scene['nodes']:
            x,y=self.node_positions[node['node_id']]
            for heading in (0,90,180,270):
                angle=math.radians(heading);origin=(x+.20*math.cos(angle),y+.20*math.sin(angle),.45)
                images=[]
                for turn in (0,math.pi/2,-math.pi/2,math.pi):
                    target=(origin[0]+math.cos(angle+turn),origin[1]+math.sin(angle+turn),origin[2])
                    view=p.computeViewMatrix(origin,target,(0,0,1))
                    image=p.getCameraImage(128,128,view,projection,renderer=p.ER_TINY_RENDERER,
                        lightDirection=(1,-1,3),physicsClientId=self.client)[2]
                    images.append(np.asarray(image,np.uint8).reshape(128,128,4)[...,:3])
                rgb=np.concatenate([np.concatenate(images[:2],axis=1),np.concatenate(images[2:],axis=1)],axis=0)
                result.append(dict(node_id=node['node_id'],heading_deg=heading,rgb=rgb))
        return result

    def _robot(self):
        # Same original masses, joints, friction and motors; SPAWN only.
        self._construct_route_scene()
        base_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.16, 0.11, 0.06), physicsClientId=self.client)
        base_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.16, 0.11, 0.06),
                                         rgbaColor=(0.2, 0.25, 0.3, 1), physicsClientId=self.client)
        rotation = p.getQuaternionFromEuler((math.pi / 2, 0, 0))
        wheel_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.08, height=0.04,
                                                collisionFrameOrientation=rotation, physicsClientId=self.client)
        wheel_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.08, length=0.04,
                                           visualFrameOrientation=rotation, rgbaColor=(0.1, 0.1, 0.1, 1),
                                           physicsClientId=self.client)
        caster = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03, physicsClientId=self.client)
        robot = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual, basePosition=(*self.spawn_xy, 0.16),
            baseOrientation=p.getQuaternionFromEuler((0, 0, math.radians(self.spawn_heading_deg))),
            linkMasses=[0.1, 0.1, 0.01, 0.01], linkCollisionShapeIndices=[wheel_collision, wheel_collision, caster, caster],
            linkVisualShapeIndices=[wheel_visual, wheel_visual, -1, -1],
            linkPositions=[(0, 0.15, -0.08), (0, -0.15, -0.08), (0.13, 0, -0.13), (-0.13, 0, -0.13)],
            linkOrientations=[(0, 0, 0, 1)] * 4, linkInertialFramePositions=[(0, 0, 0)] * 4,
            linkInertialFrameOrientations=[(0, 0, 0, 1)] * 4, linkParentIndices=[0] * 4,
            linkJointTypes=[p.JOINT_REVOLUTE, p.JOINT_REVOLUTE, p.JOINT_FIXED, p.JOINT_FIXED],
            linkJointAxis=[(0, 1, 0), (0, 1, 0), (0, 0, 0), (0, 0, 0)], physicsClientId=self.client)
        for link in (0, 1):
            p.changeDynamics(robot, link, lateralFriction=1.5, rollingFriction=0,
                             spinningFriction=0, physicsClientId=self.client)
        for link in (2, 3):
            p.changeDynamics(robot, link, lateralFriction=0.001, physicsClientId=self.client)
        return robot
