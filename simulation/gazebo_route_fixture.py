"""Explicitly augmented fixture: real ceiling tags + floor guide lines.

Only the builder reads geometry. The driver receives ID labels/size, not XY.
"""
import xml.etree.ElementTree as ET
import cv2
from simulation.gazebo_cross_validation import box, camera, tag

MARKER_SIZE = .32


def registry(graph):
    return {str(i): n['node_id'] for i, n in enumerate(graph['nodes'], 1)}


def augment(tree, graph, scene, hidden=()):
    world = tree.find('world')
    base = world.find("model[@name='robot']/link[@name='base']")
    camera(base, 'up', '0 0 .44 0 -1.5707963267948966 0', '/toponav/camera/up', width=256)
    camera(base, 'down', '.25 0 .44 0 1.5707963267948966 0', '/toponav/camera/down', width=256)
    positions = scene['geometry']['node_positions_m']
    observer = world.find("model[@name='observer']/link/sensor/camera")
    tag(observer, 'visibility_mask', 1)  # observer-only cutaway, never a driver input
    observer.find('image/width').text = '768'
    observer.find('image/height').text = '448'
    observer.find('horizontal_fov').text = '.85'

    def visual(name, size, xyz, colour):
        box(world, name, size, xyz, colour)
        link = world.find(f"model[@name='{name}']/link")
        link.remove(link.find('collision'))  # painted fixture; no fake road collision
        if name.startswith('tag_'):
            tag(link.find('visual'), 'visibility_flags', 2)

    for edge in graph['edges']:
        a, b = positions[edge['source']], positions[edge['target']]
        size = (abs(b[0]-a[0])+.04, .035, .002) if a[1] == b[1] else (.035, abs(b[1]-a[1])+.04, .002)
        visual('guide_'+edge['channel_id'], size, ((a[0]+b[0])/2, (a[1]+b[1])/2, .002), (1., .85, .02))
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id, node in registry(graph).items():
        if node in hidden:
            continue
        x,y = positions[node]
        visual('tag_back_'+node, (.40,.40,.004), (x,y,1.4), (1.,1.,1.))
        cells = cv2.aruco.generateImageMarker(dictionary, int(marker_id), 6)
        for row in range(6):
            for col in range(6):
                if cells[row,col] == 0:
                    visual(f'tag_{node}_{row}_{col}', (MARKER_SIZE/6,MARKER_SIZE/6,.002),
                           (x+(row-2.5)*MARKER_SIZE/6, y-(col-2.5)*MARKER_SIZE/6,1.396), (0.,0.,0.))
    return tree
