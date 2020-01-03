import math
import os

from payton.scene import Scene
from payton.scene.geometry import Cylinder, Plane
from payton.scene.gui import info_box


def rotate(period, total):
    global scene
    scene.objects["cylinder"].rotate_around_z(math.radians(1))


scene = Scene()
cyl = Cylinder(height=2.0)
cyl.position = [0, 0, 1.0]
ground = Plane(10, 10)

scene.create_clock("rotate", 0.01, rotate)

texture_file = os.path.join(os.path.dirname(__file__), "barrel.jpg")

cyl.material.texture = texture_file

scene.add_object("cylinder", cyl)
scene.add_object(
    "info", info_box(left=10, top=10, width=220, height=100, label="Hit SPACE\nto start animation",),
)
scene.add_object("ground", ground)

scene.run()
