import os

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.material import WIREFRAME
from payton.tools.mesh.geometry import subdivide

texture_file = os.path.join(os.path.dirname(__file__), "../basics/cube.png")

cube = Cube()
cube = subdivide(cube, 4)
cube.material.texture = texture_file

cube.material.display = WIREFRAME

normal_cube = Cube()
normal_cube.material = cube.material
normal_cube.position = [2, 0, 0]

scene = Scene()
scene.add_object("cube", cube)
scene.add_object("normal_cube", normal_cube)
scene.run()
