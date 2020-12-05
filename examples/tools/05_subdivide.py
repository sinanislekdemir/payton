import os

from payton.scene import Scene
from payton.scene.geometry import Wavefront
from payton.tools.mesh.geometry import subdivide

texture_file = os.path.join(os.path.dirname(__file__), "../basics/cube.png")
object_file = os.path.join(os.path.dirname(__file__), "../basics/monkey.obj")


monkey = Wavefront(filename=object_file)
for i in range(4):
    monkey = subdivide(monkey)

normal_monkey = Wavefront(filename=object_file)
normal_monkey.position = [2, 0, 0]

scene = Scene()
scene.add_object("cube", monkey)
scene.add_object("normal_cube", normal_monkey)
scene.run()
