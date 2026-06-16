"""Loft between a square, a hexagon, and a circle."""
import math

from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import loft


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


def make_profile(n, radius, z):
    verts = []
    for i in range(n + 1):
        a = 2 * math.pi * i / n
        verts.append([radius * math.cos(a), radius * math.sin(a), z])
    return Line(vertices=verts)


square = make_profile(4, 1.0, -2.0)
hexagon = make_profile(6, 1.2, 0.0)
circle = make_profile(32, 1.0, 2.0)

mesh = loft([square, hexagon, circle])

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("square", square)
scene.add_object("hexagon", hexagon)
scene.add_object("circle", circle)
scene.add_object("mesh", mesh)
scene.run(start_clocks=True)
