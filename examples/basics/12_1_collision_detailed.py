import os

from payton.scene import Scene
from payton.scene.collision import CollisionTest
from payton.scene.geometry import Wavefront
from payton.scene.gui import info_box

direction = 0


def motion(period, total):
    global scene, direction
    pos = scene.objects["scar2"].position
    apos = scene.objects["acar2"].position
    if direction == 0:
        pos[2] -= 0.01
        apos[2] -= 0.01
    else:
        pos[2] += 0.01
        apos[2] += 0.01
    if pos[2] < 0:
        direction = 1
    if pos[2] > 4:
        direction = 0
    scene.objects["scar2"].position = pos
    scene.objects["acar2"].position = apos


def hit_sphere(collision, pairs):
    for pair in pairs:
        pair[0].material.color = [1.0, 0, 0]
        pair[1].material.color = [1.0, 0, 0]
        # Once there is a hit, system will not check
        # for the same collision, if you want to have the objects
        # back in the collision detection pipeline, you have to do
        collision.resolve(pair[0], pair[1])
        return True


def hit_aabb(collision, pairs):
    for pair in pairs:
        pair[0].material.color = [0.0, 1.0, 0]
        pair[1].material.color = [0.0, 1.0, 0]
        # Once there is a hit, system will not check
        # for the same collision, if you want to have the objects
        # back in the collision detection pipeline, you have to do
        collision.resolve(pair[0], pair[1])
        return True


scene = Scene(width=600, height=600)
spherical_collision = CollisionTest(callback=hit_sphere, level=CollisionTest.SPHERICAL)
aabb_collision = CollisionTest(callback=hit_aabb, level=CollisionTest.AABB)

car_object_file = os.path.join(os.path.dirname(__file__), "lib", "Low-Poly-Racing-Car.obj")

spherical_car_1 = Wavefront(filename=car_object_file)
spherical_car_2 = Wavefront(filename=car_object_file)

aabb_car_1 = Wavefront(filename=car_object_file)
aabb_car_2 = Wavefront(filename=car_object_file)

spherical_car_1.position = [-2, 0, 0]
spherical_car_2.position = [-2, 0, 4]

aabb_car_1.position = [2, 0, 0]
aabb_car_2.position = [2, 0, 4]

scene.add_object("scar1", spherical_car_1)
scene.add_object("scar2", spherical_car_2)

scene.add_object("acar1", aabb_car_1)
scene.add_object("acar2", aabb_car_2)

spherical_collision.add_object(spherical_car_1)
spherical_collision.add_object(spherical_car_2)

aabb_collision.add_object(aabb_car_1)
aabb_collision.add_object(aabb_car_2)

scene.add_collision_test("spherical_collision", spherical_collision)
scene.add_collision_test("aabb_collision", aabb_collision)

scene.create_clock("motion", 0.01, motion)

scene.add_object(
    "info", info_box(left=10, top=10, label="Hit SPACE to start animation"),
)

scene.run()
