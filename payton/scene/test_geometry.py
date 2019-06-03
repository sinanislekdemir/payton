import unittest
from payton.scene.geometry import Sphere, Cube
from payton.scene.collision import CollisionTest


def dummy():
    pass


class TestGeometryMethods(unittest.TestCase):
    def test_dist(self):
        a = Sphere()
        b = Sphere()
        b.position = [3, 0, 4]
        c = CollisionTest(callback=dummy, objects=[a, b])
        dist = c._dist(a, b)
        self.assertAlmostEqual(dist, 5)

    def test_bounding_sphere_collision(self):
        a = Sphere(radius=2.0)
        b = Sphere()
        b.position = [0.5, 0.5, 0]
        c = CollisionTest(callback=dummy, objects=[a, b])
        check = c._bounding_sphere_collision(a, b)
        self.assertTrue(check)

    def test_bounding_sphere_collision_false(self):
        a = Sphere(radius=2.0)
        b = Sphere()
        b.position = [4.5, 0.5, 0]
        c = CollisionTest(callback=dummy, objects=[a, b])
        check = c._bounding_sphere_collision(a, b)
        self.assertFalse(check)

    def test_mesh_collision(self):
        a = Cube()
        b = Cube(width=2.0, height=1.23, depth=1.0)
        b.position = [1.0, 0.2, 0.1]
        c = CollisionTest(callback=dummy, objects=[a, b])
        check = c._mesh_collision(a, b)
        self.assertTrue(check)

    def test_sphere_in_sphere(self):
        a = Sphere(radius=10)
        b = Sphere(radius=2)
        b.position = [2, 2, 0]
        c = CollisionTest(callback=dummy, objects=[a, b])
        check = c._sphere_in_sphere_collision(a, b)
        self.assertTrue(check)

    def test_sphere_in_sphere_false(self):
        a = Sphere(radius=10)
        b = Sphere(radius=2)
        b.position = [10, 2, 0]
        c = CollisionTest(callback=dummy, objects=[a, b])
        check = c._sphere_in_sphere_collision(a, b)
        self.assertFalse(check)
