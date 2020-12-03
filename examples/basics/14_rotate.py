import math
import os

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane
from payton.scene.gui import info_box


def rotate(period, total):
    global scene
    y = math.radians(period * 100)
    y = -y if int(total) % 2 == 0 else y

    scene.objects["cube"].rotate_around_x(math.radians(period * 50))
    scene.objects["cube"].rotate_around_y(y)
    scene.objects["cube"].rotate_around_z(math.radians(period * 150))


scene = Scene()
cube = Cube()
ground = Plane(10, 10)
cube.position = [0, 0, 1.0]
texture_file = os.path.join(os.path.dirname(__file__), "cube.png")

scene.cameras[0].distance_to_target(5)
cube.material.texture = texture_file
scene.add_object("cube", cube)
scene.add_object("ground", ground)

scene.create_clock("rotate", 0.01, rotate)
scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        label="Hit SPACE to start animation",
    ),
)

scene.run()
