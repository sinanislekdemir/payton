"""CSG boolean operations: union, difference, and intersection."""
import math

from payton.scene import Scene
from payton.scene.geometry import Cube, Sphere
from payton.tools.mesh import csg_difference, csg_intersect, csg_union


def rotate_light(period, total):
    angle = math.radians(total * 50)
    scene.lights[0].position = [
        5.0 * math.cos(angle),
        5.0 * math.sin(angle),
        3.0,
    ]


cube = Cube()
cube.position = [0, 0, 0]

sphere = Sphere(radius=0.7)
sphere.position = [0.5, 0.5, 0.5]

union = csg_union(cube, sphere)
union.position = [-2.5, 0, 0]

difference = csg_difference(cube, sphere)
difference.position = [0, 0, 0]

intersection = csg_intersect(cube, sphere)
intersection.position = [2.5, 0, 0]

scene = Scene()
scene.create_clock("rotate_light", 0.01, rotate_light)
scene.add_object("union", union)
scene.add_object("difference", difference)
scene.add_object("intersection", intersection)
scene.run(start_clocks=True)
