"""Built-in physics: boxes, spheres and capsules falling onto a plane.

Shows that the built-in engine understands the natural collision shape of each
primitive: a ``Cube`` is a box, a ``Sphere`` is a sphere and a ``Capsule`` is a
capsule.  All of them share the same simple parameters (``mass``, position).

Controls
--------
* Space             : pause / resume physics
* Middle mouse drag : pan the camera
* Mouse wheel       : zoom

Run with ``--bullet`` to use PyBullet instead of the built-in engine (requires
``pip install pybullet``)::

    python examples/physics/03_internal_shapes.py --bullet
"""

import random
import sys

from payton.scene import Scene
from payton.scene.geometry import Capsule, Cube, Plane, Sphere
from payton.scene.gui import Hud, Text

# ``--bullet`` opts into PyBullet; without it the built-in engine is used.
use_bullet = "--bullet" in sys.argv

scene = Scene(use_internal_physics=not use_bullet)
scene.lights[0].position = [20, 20, 40]
scene.active_camera.position = [10, 10, 10]

scene.add_object("ground", Plane(width=30, height=30))

for i in range(8):
    color = [random.random() for _ in range(3)]

    cube = Cube(width=0.8, depth=0.8, height=0.8)
    cube.material.color = color
    cube.mass = 1
    cube.position = [-3, (i % 4) - 2, 3 + i]
    scene.add_object(f"cube_{i}", cube)

    ball = Sphere(radius=0.4)
    ball.material.color = color
    ball.mass = 1
    # A tiny lateral jitter so the balls do not balance exactly on top of one
    # another (a perfect vertical stack is an unstable equilibrium that an
    # ideal, symmetric simulation would otherwise hold forever).
    ball.position = [
        random.uniform(-0.03, 0.03),
        (i % 4) - 2 + random.uniform(-0.03, 0.03),
        3 + i,
    ]
    scene.add_object(f"ball_{i}", ball)

    capsule = Capsule(radius=0.3, height=0.8)
    capsule.material.color = color
    capsule.mass = 1
    # Same small jitter: a capsule resting on its rounded end would otherwise
    # stand perfectly balanced.
    capsule.position = [
        3 + random.uniform(-0.03, 0.03),
        (i % 4) - 2 + random.uniform(-0.03, 0.03),
        3 + i,
    ]
    scene.add_object(f"capsule_{i}", capsule)

hud = Hud()
hud.add_child(
    "text",
    Text(
        label="boxes, spheres and capsules - press Space to pause",
        position=[5, 5, 1],
        size=[360, 35],
        color=[1, 1, 1],
    ),
)
scene.add_object("hud", hud)

scene.run(start_clocks=True)
