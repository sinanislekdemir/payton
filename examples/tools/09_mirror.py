"""Mirror half a shape across the Y-axis to complete it symmetrically."""
import math

from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import mirror, rotate_line


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


# Create half a vase profile and revolve it (only half the revolve)
line = Line(
    vertices=[
        [0.0, 0.0, 0.0],
        [0.3, 0.0, 0.0],
        [0.5, 0.0, 0.3],
        [0.6, 0.0, 0.6],
        [0.4, 0.0, 1.0],
        [0.7, 0.0, 1.4],
        [0.5, 0.0, 1.8],
        [0.6, 0.0, 2.0],
        [0.0, 0.0, 2.0],
    ]
)

# Revolve 180 degrees to get half a shape
half = rotate_line(line, [0, 0, 1], math.radians(180), steps=12)

# Mirror to complete it
full = mirror(half, axis="x")

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("half", half)
scene.add_object("full", full)
scene.add_object("profile", line)
scene.run(start_clocks=True)
