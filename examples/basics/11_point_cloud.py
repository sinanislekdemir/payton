import random
from payton.scene import Scene
from payton.scene.gui import Hud, Text
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

hud = Hud()
text = Text(
    label="Hit Space to create points",
    position=(5, 5),
    size=(200, 35),
    color=(1, 1, 1),
)

hud.add_child("text", text)
scene.add_object("hud", hud)

pc = PointCloud()

scene.add_object("pc", pc)
scene.create_clock("generate", 0.001, generate)

scene.run()
