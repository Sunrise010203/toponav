"""Unmarked six-node fixture with a declared model-compatible camera rig.

No guide paint, tags, extra cameras or geometry passed to the controller.
The floor/wall appearance and camera calibration are bounded engineering priors.
"""
import math
from simulation.gazebo_cross_validation import box, tag

CAMERA_HEIGHT_M = .8/3
CAMERA_OFFSET_M = .18


def augment(tree, rectangles):
    world=tree.find('world')
    base=world.find("model[@name='robot']/link[@name='base']")
    for sensor in base.findall("sensor[@type='camera']"):
        fields=sensor.find('pose').text.split()
        yaw=float(fields[5]);fields[0]=str(CAMERA_OFFSET_M*math.cos(yaw));fields[1]=str(CAMERA_OFFSET_M*math.sin(yaw));fields[2]=str(CAMERA_HEIGHT_M-.16)
        sensor.find('pose').text=' '.join(fields)
    # Keep original .8m roads. h/width=1/3, ceiling=3h (training ratios).
    # Cameras sit outside the body so the hood does not obscure the floor.
    for index,(a,b,c,d) in enumerate(rectangles):
        name=f'ceiling_{index}'
        box(world,name,(b-a,d-c,.02),((a+b)/2,(c+d)/2,.81),(.5,.5,.5))
        tag(world.find(f"model[@name='{name}']/link/visual"),'visibility_flags',2)
    observer=world.find("model[@name='observer']/link/sensor/camera")
    tag(observer,'visibility_mask',1)
    observer.find('image/width').text='768'
    observer.find('image/height').text='448'
    observer.find('horizontal_fov').text='.85'
    return tree
