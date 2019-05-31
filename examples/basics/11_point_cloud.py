import random
from payton.scene import Scene
from payton.scene.geometry import PointCloud


def generate(name, scene, period, total):
    x = random.randint(-10, 10)
    y = random.randint(-10, 10)
    z = random.randint(-10, 10)
    r = random.randint(0, 255) / 255
    g = random.randint(0, 255) / 255
    b = random.randint(0, 255) / 255
    scene.objects["pc"].add([[x, y, z]], [[r, g, b]])


scene = Scene()
pc = PointCloud()

scene.add_object("pc", pc)
scene.create_clock("generate", 0.001, generate)

scene.run()
