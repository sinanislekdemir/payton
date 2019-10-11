import os
from payton.scene import Scene
from payton.scene.geometry import Wavefront
from payton.scene.geometry.wavefront import export

object_file = os.path.join(os.path.dirname(__file__), "monkey.obj")

scene = Scene()
monkey = Wavefront(filename=object_file)

export(monkey, "output.obj")
exported_mesh = Wavefront(filename="output.obj")

scene.add_object("monkey", exported_mesh)
scene.run()
