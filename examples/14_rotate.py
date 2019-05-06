import math
from payton.scene import Scene
from payton.scene.geometry import Cube


def rotate(name, scene, period, total):
    scene.objects["cube"].rotate_around_x(math.radians(period * 50))
    scene.objects["cube"].rotate_around_y(-math.radians(period * 100))
    scene.objects["cube"].rotate_around_z(math.radians(period * 150))


scene = Scene()
cube = Cube()
scene.observers[0].distance_to_target(3)
cube.material.texture = "cube.png"
scene.add_object("cube", cube)

scene.create_clock("rotate", 0.01, rotate)
scene.run()
