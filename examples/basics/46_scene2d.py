"""Scene2D - side-view platformer demo.

The camera is locked to the X-Z plane and looks from -Y towards +Y, so X is
the screen horizontal and Z is the screen vertical. The camera follows the
player along X and Z while its depth (Y) stays fixed.

Controls
--------
* Middle mouse drag : pan the camera (when not following an object)
* Mouse wheel       : zoom in / out
"""

import math

from payton.scene import SHADOW_MID, Scene2D
from payton.scene.geometry import Cube
from payton.scene.gui import info_box

scene = Scene2D(width=1000, height=600, depth=12.0, zoom=9.0)
scene.shadow_quality = SHADOW_MID

# Ground: a long, low block whose top edge sits on Z = 0.
ground = Cube(width=80, depth=4, height=1)
ground.position = [0, 0, -0.5]
scene.add_object("ground", ground)

# A few platforms at different heights.
platforms = [
    ("platform_1", -10, 2, 6),
    ("platform_2", 0, 4, 6),
    ("platform_3", 12, 6, 6),
]
for name, x, z, width in platforms:
    platform = Cube(width=width, depth=3, height=0.6)
    platform.position = [x, 0, z]
    scene.add_object(name, platform)

# The player cube.
player = Cube(width=1, depth=1, height=1)
player.position = [0, 0, 0.5]
scene.add_object("player", player)

# Make the camera track the player along the X-Z plane.
scene.follow(player)

scene.add_object(
    "info",
    info_box(left=10, top=10, label="Scene2D - camera follows the player"),
)


def animate(period, total):
    """Move the player left / right and make it hop."""
    player.position = [
        math.sin(total) * 20,
        0,
        0.5 + abs(math.sin(total * 2.5)) * 2.0,
    ]


scene.create_clock("animate", 0.016, animate)
scene.run(start_clocks=True)
