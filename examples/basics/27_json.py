import os

from payton.scene.geometry import Cube, Sphere, Wavefront
from payton.scene.geometry.export import export_json, import_json
from payton.scene.scene import Scene

object_file = os.path.join(os.path.dirname(__file__), "monkey.obj")

monkey = Wavefront(filename=object_file)
monkey.add_child("cube", Cube())
monkey.children["cube"].position = [0, 0, 3]
monkey.children["cube"].add_child("sphere", Sphere())
monkey.children["cube"].children["sphere"].position = [1, 0, 0]

export_json(monkey, "test.json")
new = import_json("test.json")

breakpoint()
scene = Scene()
scene.add_object("monkey", new)
scene.run()
