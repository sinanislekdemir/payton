import math
import os

from payton.scene import SHADOW_HIGH, Scene
from payton.scene.geometry import Cube, Plane

scene = Scene()
scene.shadow_quality = SHADOW_HIGH


def move(period, total):
    angle = (total * 60) % 360
    px = math.cos(math.radians(angle)) * 8
    py = math.sin(math.radians(angle)) * 8
    scene.lights[0].position = [px, py, 4.0, 1.0]


cube = Cube()
cube.position = [2, 1, 0.5]

ground = Plane(width=30, height=30)
wall1 = Plane(width=30, height=10)
wall1.rotate_around_x(math.radians(90))
wall1.position = [0, -10, 5]

scene.lights[0].position = [5.0, 5.0, 6.0]
cube_by_corners = Cube(from_corner=[-3, -3, 1], to_corner=[-1, -1, 3])
scene.create_clock("mm", 0.001, move)

texture_file = os.path.join(os.path.dirname(__file__), "barrel.jpg")
cube_by_corners.material.texture = texture_file
ground.material.texture = texture_file

scene.add_object("wall", wall1)
scene.add_object("cube", cube)
scene.add_object("cube_by_corners", cube_by_corners)
scene.add_object("ground", ground)
scene.grid.visible = False

scene.run(start_clocks=True)
