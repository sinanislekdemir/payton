import os

from payton.scene import Scene
from payton.scene.geometry import Plane, Wavefront
from payton.scene.geometry.wavefront import export

object_file = os.path.join(os.path.dirname(__file__), "monkey.obj")

scene = Scene()
ground = Plane(10, 10)
monkey = Wavefront(filename=object_file)

export(monkey, "output.obj")
exported_mesh = Wavefront(filename="output.obj")
exported_mesh.position = [0, 0, 1.0]

scene.add_object("monkey", exported_mesh)
scene.add_object("ground", ground)
scene.run()
