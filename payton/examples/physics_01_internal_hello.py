"""Built-in physics: a cube falling onto a plane.

This demo uses Payton's dependency-free physics engine
(``Scene(use_internal_physics=True)``), so it runs even when PyBullet is not
installed.  Physics starts immediately.

Run with ``--bullet`` to use PyBullet instead (requires ``pip install pybullet``),
which is handy for a side-by-side comparison::

    python examples/physics/01_internal_hello.py --bullet

Controls
--------
* Space             : pause / resume physics
* Middle mouse drag : pan the camera
* Mouse wheel       : zoom
"""

import sys

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane
from payton.scene.gui import Hud, Text

# ``--bullet`` opts into PyBullet; without it the built-in engine is used.
use_bullet = "--bullet" in sys.argv

scene = Scene(use_internal_physics=not use_bullet)
scene.lights[0].position = [20, 20, 40]
scene.active_camera.position = [8, 8, 8]

# A static ground (mass defaults to 0, which means "never moves").
ground = Plane(width=20, height=20)
scene.add_object("ground", ground)

# Anything with a positive mass reacts to gravity and collisions.
cube = Cube(width=1, depth=1, height=1)
cube.mass = 1
cube.position = [0, 0, 6]
scene.add_object("cube", cube)

hud = Hud()
hud.add_child(
    "text",
    Text(
        label="Built-in physics - press Space to pause",
        position=[5, 5, 1],
        size=[340, 35],
        color=[1, 1, 1],
    ),
)
scene.add_object("hud", hud)

scene.run(start_clocks=True)
