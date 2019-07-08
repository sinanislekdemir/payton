import os
import math
from payton.scene import Scene
from payton.scene.geometry import Cube


def rotate(period, total):
    global scene
    y = math.radians(period * 100)
    y = -y if int(total) % 2 == 0 else y

    scene.objects["cube"].rotate_around_x(math.radians(period * 50))
    scene.objects["cube"].rotate_around_y(y)
    scene.objects["cube"].rotate_around_z(math.radians(period * 150))


scene = Scene()
cube = Cube()
texture_file = os.path.join(os.path.dirname(__file__), "cube.png")

scene.observers[0].distance_to_target(3)
cube.material.texture = texture_file
scene.add_object("cube", cube)

scene.create_clock("rotate", 0.01, rotate)
scene.run()
