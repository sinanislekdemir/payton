import math
from payton.scene import Scene
from payton.scene.geometry import Cube


def rotate(name, scene, period, total):
    scene.objects["cube"].rotate(math.radians(period * 100))
    scene.objects["cube"].pitch(math.radians(period * 200))
    scene.objects["cube"].roll(math.radians(period * 300))


scene = Scene()
cube = Cube()
cube.material.texture = "cube.png"
scene.add_object("cube", cube)

cube.rotate(math.radians(45))

scene.create_clock("rotate", 0.01, rotate)
scene.run()
