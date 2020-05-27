import random

from payton.scene import Scene
from payton.scene.geometry import ParticleSystem
from payton.scene.gui import Hud, Text

scene = Scene()
pc = ParticleSystem(particle_scale=0.1)
pc.material.color = [0.7, 1, 0]

counter = 0


def generate(period, total):
    global counter
    global scene
    counter += 1
    x = random.randint(-1000, 1000) / 100.0
    y = random.randint(-1000, 1000) / 100.0
    z = random.randint(-1000, 1000) / 100.0

    pc.add([x, y, z])
    if counter % 100 == 0:
        scene.huds["hud"].children["text"].label = f"FPS: {scene.fps}"


hud = Hud()
text = Text(label="Hit Space to create points", position=(5, 5), size=(200, 35), color=(1, 1, 1),)

hud.add_child("text", text)
scene.add_object("hud", hud)

scene.add_object("pc", pc)
scene.create_clock("generate", 0.001, generate)

scene.run()
