import random

from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()

for i in range(100):
    x = random.randint(-10, 10)
    y = random.randint(-10, 10)
    z = random.randint(-10, 10)
    r = random.randint(0, 255) / 255.0
    g = random.randint(0, 255) / 255.0
    b = random.randint(0, 255) / 255.0
    cube = Cube()
    cube.material.color = [r, g, b]
    cube.material.opacity = random.randint(0, 255) / 255.0
    cube.position = [x, y, z]
    scene.add_object("cube_{}".format(i), cube)

scene.run()
