from payton.scene import Scene
from payton.scene.geometry import Plane


scene = Scene()
plane = Plane(width=2, height=2)
scene.add_object("plane", plane)
scene.run()
