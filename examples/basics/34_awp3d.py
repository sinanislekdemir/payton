"""Original model from mixamo.com"""
import os

from payton.scene import Scene
from payton.scene.geometry import Plane
from payton.scene.geometry.awp3d import AWP3D

object_file = os.path.join(os.path.dirname(__file__), "AWP3D", "fall-to-roll.awp3d")

s = Scene()
print("Loading file")
anim = AWP3D(filename=object_file)
print("Loaded file")
s.add_object("arissa", anim)

ground = Plane(10, 10)
s.add_object("ground", ground)
s.run()
