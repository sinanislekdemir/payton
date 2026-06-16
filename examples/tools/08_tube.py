"""Tube along a sine-wave 3D curve."""
import math

from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import tube


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


# Sine-wave path
path_verts = []
for t in range(101):
    u = t / 100.0
    path_verts.append([
        u * 4.0 - 2.0,
        math.sin(u * 6 * math.pi) * 1.5,
        math.cos(u * 4 * math.pi) * 1.0,
    ])
path = Line(vertices=path_verts)

mesh = tube(path, radius=0.15, segments=12)

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("path", path)
scene.add_object("mesh", mesh)
scene.run(start_clocks=True)
