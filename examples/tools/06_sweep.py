"""Sweep a star-shaped profile along a helical path."""
import math

from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import sweep


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


# Star-shaped profile (6-pointed star in the XY plane)
profile_verts = []
for i in range(13):
    angle = 2 * math.pi * i / 12
    r = 0.3 if i % 2 == 0 else 0.15
    profile_verts.append([r * math.cos(angle), r * math.sin(angle), 0.0])
profile = Line(vertices=profile_verts)

# Helical path
path_verts = []
for t in range(101):
    u = t / 100.0
    path_verts.append([
        2.0 * math.cos(u * 4 * math.pi),
        2.0 * math.sin(u * 4 * math.pi),
        u * 3.0,
    ])
path = Line(vertices=path_verts)

mesh = sweep(profile, path)

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("profile", profile)
scene.add_object("path", path)
scene.add_object("mesh", mesh)
scene.run(start_clocks=True)
