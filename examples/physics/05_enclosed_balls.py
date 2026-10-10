"""Built-in physics: balls raining into an enclosed arena.

A walled pen -- a solid floor plus four *wireframe* walls built from boxes --
that balls of random size and colour keep dropping into, one at a time, until
the app is closed or 1000 balls have spawned.  Shows off sphere-sphere and
sphere-box collisions in the dependency-free engine.

Run with ``--bullet`` to use PyBullet instead (requires ``pip install pybullet``)::

    python examples/physics/05_enclosed_balls.py --bullet

Controls
--------
* Space             : pause / resume physics
* Middle mouse drag : pan the camera
* Mouse wheel       : zoom
"""

import random
import sys

from payton.scene import Scene
from payton.scene.geometry import Cube, Sphere
from payton.scene.gui import Hud, Text
from payton.scene.material import WIREFRAME

MAX_BALLS = 1000

# ``--bullet`` opts into PyBullet; without it the built-in engine is used.
use_bullet = "--bullet" in sys.argv

scene = Scene(use_internal_physics=not use_bullet)
scene.lights[0].position = [20, 20, 40]
scene.active_camera.position = [22, 22, 18]

# --- Enclosed arena: floor + four walls (static boxes, mass stays 0) --------
HALF = 8.0  # interior half width / depth
WALL = 1.0  # wall thickness
HEIGHT = 14.0  # wall height


def add_wall(
    name: str, width: float, depth: float, height: float, position: list[float]
) -> None:
    """Add a static wall and force it to draw as a wireframe."""
    box = Cube(width=width, depth=depth, height=height)
    box.position = position
    box.material.display = WIREFRAME
    box.material.lights = False
    scene.add_object(name, box)


# Solid floor (its top surface sits at z = 0)
floor = Cube(width=2 * (HALF + WALL), depth=2 * (HALF + WALL), height=WALL)
floor.position = [0, 0, -WALL / 2]
scene.add_object("floor", floor)

# Four see-through walls
add_wall("wall_x+", WALL, 2 * (HALF + WALL), HEIGHT, [HALF + WALL / 2, 0, HEIGHT / 2])
add_wall("wall_x-", WALL, 2 * (HALF + WALL), HEIGHT, [-(HALF + WALL / 2), 0, HEIGHT / 2])
add_wall("wall_y+", 2 * (HALF + WALL), WALL, HEIGHT, [0, HALF + WALL / 2, HEIGHT / 2])
add_wall("wall_y-", 2 * (HALF + WALL), WALL, HEIGHT, [0, -(HALF + WALL / 2), HEIGHT / 2])

hud = Hud()
text = Text(
    label=f"Balls: 0 / {MAX_BALLS}   (Space to pause)",
    position=[5, 5, 1],
    size=[360, 35],
    color=[1, 1, 1],
)
hud.add_child("text", text)
scene.add_object("hud", hud)


# --- Keep dropping balls until the cap is reached ---------------------------
spawned = 0


def drop_ball(period: float, total: float) -> None:
    """Clock callback: add one random ball, up to MAX_BALLS."""
    global spawned
    if spawned >= MAX_BALLS:
        return
    radius = random.uniform(0.3, 0.6)
    ball = Sphere(radius=radius)
    ball.material.color = [random.random() for _ in range(3)]
    ball.mass = 1
    ball.position = [
        random.uniform(-HALF + radius, HALF - radius),
        random.uniform(-HALF + radius, HALF - radius),
        HEIGHT + random.uniform(0.5, 3.0),
    ]
    scene.add_object(f"ball_{spawned}", ball)
    spawned += 1
    text.label = f"Balls: {spawned} / {MAX_BALLS}   (Space to pause)"


scene.create_clock("ball_spawner", 0.1, drop_ball)

scene.run(start_clocks=True)
