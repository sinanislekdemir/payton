"""Subdivide a cube twice, then smooth the result."""
import math

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.tools.mesh import laplacian_smooth, subdivide


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


cube = Cube()
cube.position = [-2, 0, 0]

# Subdivide twice to get more geometry
sub1 = subdivide(cube)
sub2 = subdivide(sub1)
sub2.position = [0, 0, 0]

# Smooth the subdivided result
smoothed = laplacian_smooth(sub2, iterations=3, factor=0.4)
smoothed.position = [2, 0, 0]
smoothed.toggle_wireframe()

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("cube", cube)
scene.add_object("subdivided", sub2)
scene.add_object("smoothed", smoothed)
scene.run(start_clocks=True)
