import random
from payton.scene import Scene
from payton.scene.geometry import Sphere, Cube
from payton.scene.collision import CollisionTest


def hit(collision, pairs):
    for pair in pairs:
        pair[0].material.color = [1.0, 0, 0]
        pair[1].material.color = [1.0, 0, 0]
        # Once there is a hit, system will not check
        # for the same collision, if you want to have the objects
        # back in the collision detection pipeline, you have to do
        # collision.resolve(pair[0], pair[1])


scene = Scene()
collision = CollisionTest(callback=hit)
for i in range(50):
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    z = random.randint(-5, 5)
    if i % 2 == 0:
        s = Sphere()
        s.set_position([x, y, z])
        scene.add_object('s_{}'.format(i), s)
        collision.add_object(s)
    else:
        c = Cube()
        c.set_position([x, y, z])
        scene.add_object('c_{}'.format(i), c)
        collision.add_object(c)

scene.add_collision_test(collision)
scene.run()
