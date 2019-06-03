import random
from payton.scene import Scene
from payton.scene.geometry import Cube


def select(list):
    for obj in list:
        obj.material.color = [1, 1, 1]


scene = Scene(on_select=select)
for i in range(10):
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    z = random.randint(-5, 5)
    r = random.randint(0, 255) / 255.0
    g = random.randint(0, 255) / 255.0
    b = random.randint(0, 255) / 255.0
    cube = Cube()
    cube.material.color = [r, g, b]
    cube.position = [x, y, z]
    scene.add_object("cube_{}".format(i), cube)

print("Try clicking on objects")

scene.run()
