import math
import os

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane


scene = Scene()


def move(period, total):
    angle = (total * 40) % 360
    px = math.cos(math.radians(angle)) * 8
    py = math.sin(math.radians(angle)) * 8
    scene.lights[0].position = [px, py, 2.0]


cube = Cube()
ground = Cube(width=30, height=30, depth=30)
ground = Cube(width=30, height=30, depth=30)

scene.lights[0].position = [1.0, 1.0, 9.0]
scene.lights[0].position = [5.0, 5.0, 2.0]
cube_by_corners = Cube(from_corner=[-3, -3, 1], to_corner=[-1, -1, 3])
scene.create_clock('mm', 0.01, move)

texture_file = os.path.join(os.path.dirname(__file__), "barrel.jpg")
cube.material.texture = texture_file
cube_by_corners.material.texture = texture_file
ground.material.texture = texture_file

cube.position = [0, 0, 0.5]
scene.add_object("cube", cube)
scene.add_object("cube_by_corners", cube_by_corners)
scene.add_object("ground", ground)
scene.grid.visible = False

scene.run()
