"""Load a Wavefront model and decimate it to reduce polygon count."""
import math
import os

from payton.scene import Scene
from payton.scene.geometry import Wavefront
from payton.tools.mesh import decimate


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


object_file = os.path.join(os.path.dirname(__file__), "../basics/monkey.obj")

original = Wavefront(filename=object_file)
original.position = [-1.5, 0, 0]

simplified = decimate(original, ratio=0.3)
simplified.position = [1.5, 0, 0]

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("original", original)
scene.add_object("simplified", simplified)
scene.run(start_clocks=True)
