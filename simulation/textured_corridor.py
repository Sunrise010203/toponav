"""Same wheel physics/camera, moderately band-limited visual-only textures.

Development fixture for the calibrated RGB producer, NOT yet a controller or
acceptance result. No encoded IDs/markers or labels enter captured packets.
Texture seed/appearance/geometry remain simulator inputs and audit metadata.
"""
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import pybullet as p

from simulation.corridor_validation import ValidationWorld


class TexturedCorridorWorld(ValidationWorld):
    def __init__(self, config=None, appearance=None, *, assets, seed=313, texture_cell_m=.05,pattern='unique'):
        if type(seed) is not int or not 0<=seed<2**31:raise ValueError('Expected nonnegative integer texture seed')
        if type(texture_cell_m) not in (int,float) or not np.isfinite(texture_cell_m) or not .02<=texture_cell_m<=.2:
            raise ValueError('Expected declared physical texture cell .02 to .2m')
        if pattern not in ('unique','uniform','repeated'):raise ValueError('Unsupported texture pattern')
        self.assets=Path(assets);self.assets.mkdir(parents=True,exist_ok=False)
        self.texture_seed=seed;self.texture_count=0;self.texture_cell_m=texture_cell_m;self.texture_pattern=pattern
        super().__init__(config,appearance)

    def _box(self, half, center, color):
        color=self.appearance['floor_rgb'] if center[2]<0 else self.appearance['wall_rgb']
        if self.config.obstacle!='none' and tuple(center)==(3.05,0,.8):
            front=self.appearance['barrier_front_m']
            if front is not None:center=(front+half[0],center[1],center[2])
            if self.appearance['barrier_rgb'] is not None:color=self.appearance['barrier_rgb']
        rng=np.random.default_rng(self.texture_seed+self.texture_count)
        # Explicit isotropic physical spectrum, not a box-size-dependent
        # 32x32 stretch. These scene changes are NOT algorithm improvements.
        extent=2*max(half);cells=max(4,int(np.ceil(extent/self.texture_cell_m)))
        raw=rng.integers(60,240,(cells,cells),dtype=np.uint8)
        if self.texture_pattern=='uniform':raw[:]=128
        elif self.texture_pattern=='repeated':
            yy,xx=np.indices(raw.shape);raw=((xx+yy)%2*150+60).astype(np.uint8)
        gray=cv2.GaussianBlur(cv2.resize(raw,(256,256),interpolation=cv2.INTER_LINEAR),(5,5),.8)
        path=self.assets/f'texture-{self.texture_count}.png';self.texture_count+=1
        Image.fromarray(np.repeat(gray[:,:,None],3,axis=2)).save(path)
        # GEOM_BOX's renderer UV convention stretched the first development
        # texture almost flat. Explicit UVs preserve metres per texture cell.
        # The collision box, pose, friction, mass and motor model stay the same.
        x,y,z=half
        vertices=[(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),(-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)]
        faces=[((1,4,3,2),(0,0,-1)),((5,6,7,8),(0,0,1)),((1,2,6,5),(0,-1,0)),
               ((4,8,7,3),(0,1,0)),((1,5,8,4),(-1,0,0)),((2,3,7,6),(1,0,0))]
        text=''.join('v '+' '.join(map(str,v))+'\n' for v in vertices)
        for indices,_ in faces:
            pts=np.array([vertices[i-1] for i in indices]);u=np.linalg.norm(pts[1]-pts[0])/extent;v=np.linalg.norm(pts[3]-pts[0])/extent
            text+=f'vt 0 0\nvt {u} 0\nvt {u} {v}\nvt 0 {v}\n'
        text+=''.join('vn '+' '.join(map(str,n))+'\n' for _,n in faces)
        for ni,(indices,_) in enumerate(faces,1):
            for a,b,c in ((0,1,2),(0,2,3)):
                text+='f '+' '.join(f'{indices[j]}/{4*(ni-1)+j+1}/{ni}' for j in (a,b,c))+'\n'
        mesh=path.with_suffix('.obj');mesh.write_text(text)
        collision=p.createCollisionShape(p.GEOM_BOX,halfExtents=half,physicsClientId=self.client)
        visual=p.createVisualShape(p.GEOM_MESH,fileName=str(mesh),rgbaColor=(*color,1),physicsClientId=self.client)
        body=p.createMultiBody(baseMass=0,baseCollisionShapeIndex=collision,baseVisualShapeIndex=visual,
                               basePosition=center,physicsClientId=self.client)
        texture=p.loadTexture(str(path),physicsClientId=self.client)
        p.changeVisualShape(body,-1,textureUniqueId=texture,physicsClientId=self.client)
        return body
