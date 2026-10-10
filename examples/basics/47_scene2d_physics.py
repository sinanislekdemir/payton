"""Scene2D - bullet physics, 2-D side view.

A 2-D adaptation of examples/basics/37_bullet_cubes.py. Instead of a Jenga
tower in the X-Y plane, the blocks are stacked in the X-Z plane so the locked
Scene2D camera sees a collapsing tower. Requires pybullet
(``pip install pybullet``).

Controls
--------
* Space             : start / pause physics
* Middle mouse drag : pan the camera
* Mouse wheel       : zoom
"""

import random

from payton.scene import Scene2D
from payton.scene.geometry import Cube
from payton.scene.gui import Hud, Text

s = Scene2D(width=1000, height=600, depth=20.0, zoom=8.0)
s.lights[0].position = [-10, -50, 50]

# Static ground slab (mass = 0).
ground = Cube(width=60, depth=4, height=1)
ground.position = [0, 0, -0.5]
s.add_object("ground", ground)

# A staggered brick tower in the X-Z plane. Each brick is a dynamic body.
# A small vertical gap between rows lets the stack visibly drop when physics
# starts.
columns = 4
rows = 10
brick_w = 2.0
brick_h = 0.5
brick_d = 2.0
row_gap = 0.3

for row in range(rows):
    # Offset every other row by half a brick, like real brickwork.
    offset = (row % 2) * (brick_w / 2.0)
    for col in range(columns):
        color = (
            random.randint(1, 255) / 255.0,
            random.randint(1, 255) / 255.0,
            random.randint(1, 255) / 255.0,
        )
        brick = Cube(width=brick_w, depth=brick_d, height=brick_h)
        brick.material.color = color
        brick.mass = 1
        x = (col - (columns - 1) / 2.0) * brick_w + offset
        z = row * (brick_h + row_gap) + brick_h / 2.0
        brick.position = (x, 0, z)
        s.add_object(f"brick_{row}_{col}", brick)

hud = Hud()
text = Text(
    label="Hit Space to start physics",
    position=[5, 5, 1],
    size=[260, 35],
    color=[1, 1, 1],
)
hud.add_child("text", text)
s.add_object("hud", hud)

s.run()
