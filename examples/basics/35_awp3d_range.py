import os

from payton.scene import Scene
from payton.scene.geometry import Plane
from payton.scene.geometry.awp3d import AWP3D

"""Ideally you can create multiple animations
seperated by key frames.

For instance:
Frames -> Anim
0-64   -> Run
65-120 -> Death

For that purpose AWP3D can be defined with
ranges
"""

"""Original model from mixamo.com"""

object_file = os.path.join(os.path.dirname(__file__), "AWP3D", "fall-to-roll.awp3d")

s = Scene()
print("Loading file")
anim = AWP3D(filename=object_file)
anim.set_animation("in-the-air", 0, 14)
anim.set_animation("rolling", 15, 62)
anim.run_animation("rolling")
print("Loaded file")
s.add_object("arissa", anim)

ground = Plane(10, 10)
s.add_object("ground", ground)
s.run()
