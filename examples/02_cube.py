from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube = Cube()
scene.add_object('cube', cube)

scene.run()
