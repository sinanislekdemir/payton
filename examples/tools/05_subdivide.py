import os

from payton.scene import Scene
from payton.scene.geometry import Wavefront
from payton.tools.mesh.geometry import subdivide

texture_file = os.path.join(os.path.dirname(__file__), "../basics/cube.png")
object_file = os.path.join(os.path.dirname(__file__), "../basics/monkey.obj")


# 4-way midpoint subdivision: each pass multiplies face count by 4.
# Use 3 passes (968 → ~62k faces) for a good balance of detail vs performance;
# 4 passes yields ~248k faces which is visually identical but slower to build.
monkey = Wavefront(filename=object_file)
for _ in range(3):
    monkey = subdivide(monkey)

normal_monkey = Wavefront(filename=object_file)
normal_monkey.position = [2, 0, 0]

scene = Scene()
scene.add_object("cube", monkey)
scene.add_object("normal_cube", normal_monkey)
monkey.toggle_wireframe()
normal_monkey.toggle_wireframe()
scene.run()
