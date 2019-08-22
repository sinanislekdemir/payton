import os
from payton.scene import Scene
from payton.math.vector import distance
from payton.scene.geometry import Plane
from payton.scene.wavefront import Wavefront

scene = Scene(width=1600, height=800)
target = [0, 0, 0]


def move_to_target(period, total):
    if distance(scene.objects["dir"].position, target) > 0.1:
        scene.objects["dir"].forward(0.1)


def set_target(hit):
    global target
    scene.objects["dir"].direct_to(hit)
    target = hit


scene.add_click_plane([0, 0, 0], [0, 0, 1], set_target)
scene.create_clock("move", 0.05, move_to_target)

obj_file = os.path.join(os.path.dirname(__file__), "director.obj")
tex_file = os.path.join(os.path.dirname(__file__), "checkers.jpg")

odir = Wavefront(filename=obj_file)
ground = Plane(width=20, height=20)
ground.material.texture = tex_file

scene.add_object("dir", odir)
scene.add_object("ground", ground)

print("Hit space to start then click on the scene")

scene.run()
