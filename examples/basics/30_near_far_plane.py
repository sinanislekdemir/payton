import random

from payton.scene import Scene
from payton.scene.collision import CollisionTest
from payton.scene.geometry import Cube, Sphere


scene = Scene()
direction = 0


def hit(collision, pairs):
    for pair in pairs:
        pair[0].material.color = [1.0, 0, 0]
        pair[1].material.color = [1.0, 0, 0]
        # Once there is a hit, system will not check
        # for the same collision, if you want to have the objects
        # back in the collision detection pipeline, you have to do
        # collision.resolve(pair[0], pair[1])


def change_near_plane(period, total):
    global direction
    if direction == 0:
        scene.active_observer.near += 0.1
        if scene.active_observer.near > 15:
            direction = 1
    if direction == 1:
        scene.active_observer.near -= 0.1
        if scene.active_observer.near < 0.5:
            direction = 2
    if direction == 2:
        scene.active_observer.far -= 0.1
        if scene.active_observer.far < 1:
            direction = 3
    if direction == 3:
        scene.active_observer.far += 0.1
        if scene.active_observer.far > 20:
            direction = 0
    print(direction)


scene.active_observer.far = 20
collision = CollisionTest(callback=hit)
for i in range(50):
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    z = random.randint(-5, 5)
    if i % 2 == 0:
        s = Sphere()
        s.position = [x, y, z]
        scene.add_object("s_{}".format(i), s)
        collision.add_object(s)
    else:
        c = Cube()
        c.position = [x, y, z]
        scene.add_object("c_{}".format(i), c)
        collision.add_object(c)

scene.add_collision_test("test", collision)
scene.create_clock("plane", 0.01, change_near_plane)
scene.run()
