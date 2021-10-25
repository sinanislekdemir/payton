import math
import os

from payton.scene import SHADOW_NONE, Scene
from payton.scene.geometry import Sphere
from payton.scene.gui import info_box
from payton.scene.light import Light


def motion(period, total):
    global space
    angle = (total * 10) % 360
    px = math.cos(math.radians(angle)) * 8
    py = math.sin(math.radians(angle)) * 8
    space.objects["earth"].children["moon"].position = [px, py, 0]
    space.objects["earth"].rotate_around_z(math.radians(1.0))

    sx = math.cos(math.radians(angle * 10)) * 2  # 10 times faster
    sy = math.sin(math.radians(angle * 10)) * 2
    space.objects["earth"].children["moon"].children["moon_moon"].position = [
        sx,
        sy,
        0,
    ]

    space.lights[0].position = [px, py, 0]
    space.lights[1].position = [-px, -py, 0]


space = Scene()
space.shadow_quality = SHADOW_NONE
space.lights.append(Light(position=[12, 12, 12]))
space.cameras[0].position = [20, 20, 20]
space.grid.resize(40, 40, 1)

texture_file = os.path.join(os.path.dirname(__file__), "map.png")

earth = Sphere(radius=5, parallels=36, meridians=36)
earth.material.texture = texture_file
moon = Sphere()
moon.position = [8, 0, 0]

moon_moon = Sphere(radius=0.5)
moon_moon.position = [0, 2, 0]

earth.add_child("moon", moon)
moon.add_child("moon_moon", moon_moon)

space.add_object("earth", earth)

space.create_clock("motion", 0.01, motion)
space.add_object(
    "info",
    info_box(left=10, top=10, label="Hit SPACE to start animation"),
)

space.run(True)
