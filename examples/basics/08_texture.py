import os

from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube = Cube()

texture_file = os.path.join(os.path.dirname(__file__), "cube.png")
cube.material.texture = texture_file
scene.add_object("cube", cube)

scene.run()
