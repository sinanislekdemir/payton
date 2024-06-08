import random

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane
from payton.scene.gui import Hud, Text

s = Scene()
s.lights[0].position = (-10, 50, 50)
s.active_camera.position = (30, 30, 30)

for i in range(0, 20):
    c1r = (
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
    )
    c1 = Cube(width=0.5, depth=4, height=0.4)
    c1.material.color = c1r
    c2r = (
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
    )
    c2 = Cube(width=0.5, depth=4, height=0.4)
    c2.material.color = c2r
    c3r = (
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
    )
    c3 = Cube(width=4.0, depth=0.5, height=0.4)
    c3.material.color = c3r
    c4r = (
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
        random.randint(1, 255) / 255.0,
    )
    c4 = Cube(width=4.0, depth=0.5, height=0.4)
    c4.material.color = c4r
    c1.mass = 1
    c2.mass = 1
    c3.mass = 1
    c4.mass = 1
    c1.position = (-1, 0, i)
    c2.position = (1, 0, i)
    c3.position = (0, -1, i + 1)
    c4.position = (0, 1, i + 1)
    s.add_object(f"c1_{i}", c1)
    s.add_object(f"c2_{i}", c2)
    s.add_object(f"c3_{i}", c3)
    s.add_object(f"c4_{i}", c4)


ground = Plane(20, 20)
s.add_object("ground", ground)

hud = Hud()
text = Text(
    label="Hit Space to start physics",
    position=[5, 5, 1],
    size=[200, 35],
    color=[1, 1, 1],
)
hud.add_child("text", text)
s.add_object("hud", hud)

s.run()
