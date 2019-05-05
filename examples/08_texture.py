from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube = Cube()
cube.material.texture = "cube.png"
scene.add_object("cube", cube)

scene.run()
