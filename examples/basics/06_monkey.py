import os
from payton.scene import Scene
from payton.scene.wavefront import Wavefront

object_file = os.path.join(os.path.dirname(__file__), "monkey.obj")

scene = Scene()
monkey = Wavefront(filename=object_file)

scene.add_object("monkey", monkey)
scene.run()
