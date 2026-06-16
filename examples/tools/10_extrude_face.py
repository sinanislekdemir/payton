"""Extrude selected faces of a cube with tapering."""
import math

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.tools.mesh import extrude_face


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


# Start with a cube
cube = Cube()
cube.position = [0, 0, 0]

# Extrude face index 0 (one face) outward with a slight taper
modified = extrude_face(cube, face_indices=[0], distance=0.8, taper=0.7, segments=1)

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("original", cube)
modified.position = [2.5, 0, 0]
scene.add_object("extruded", modified)
scene.run(start_clocks=True)
