"""
Collision detection module

Example collision test:


    from payton.scene import Scene
    from payton.scene.geometry import Sphere
    from payton.scene.collision import CollisionTest

    def hit(collision, pairs):
        for pair in pairs:
            pair[0].material.color = [1, 0, 0]
            pair[1].material.color = [1, 0, 0]
            # mark collision as resolved if you want the pair
            # to be checked at the next cycle.
            # collision.resolve(pair[0], pair[1])

    scene = Scene()
    test = CollisionTest(callback=hit)
    s1 = Sphere()
    s1.set_position([0, 0, 0])
    s2 = Sphere()
    s2.set_position([0.7, 0, 0])
    s3 = Sphere()
    s3.set_position([0, 0, 4])
    s4 = Sphere()
    s4.set_position([2, 0, 4])

    scene.add_object('s1', s1)
    scene.add_object('s2', s2)
    scene.add_object('s3', s3)
    scene.add_object('s4', s4)

    test.add_object(s1)
    test.add_object(s2)
    test.add_object(s3)
    test.add_object(s4)

    scene.add_collision_test(test)
    scene.run()

"""
import logging
from payton.scene.geometry import Mesh, Sphere
from payton.math.geometry import distance


class Collision(object):
    """Collision pair result class"""

    def __init__(self, obj1, obj2):
        self.object_1 = obj1
        self.object_2 = obj2
        self.segment = 1


class CollisionTest(object):
    """Collision detection class

    Collision detection is made between selected objects so instead
    of trying to make the decision between all scene objects, specific
    objects must be selected for testing.

    Collision detection is a costly operation and often times, to increase
    the performance, making some assumptions can work well. As an example,
    objects' bounding sphere check is a very simple arithmetic operation.
    But collision detection using mesh triangles is quite costly.

    Collision test uses several techniques:
      - Bounding sphere collision test for Sphere2Sphere test.
      - Bounding sphere collision test for elimination
      - Sphere in sphere collision test for objects in objects
      - AABB collision test for object elimination and aproximate calculations

    Therefore, collision detection class uses some approximations. There are
    two different approximations:

      - SPHERICAL: Does a bounding sphere check only
      - AABB: Does an Axis Alinged Bounding Box check only (DEFAULT)
    """

    SPHERICAL = 0
    AABB = 1

    def __init__(self, **args):
        """Initialize collision detector

        Args:
          objects: List of object references to test
          callback: Callback function to call incase of collision
          level: Level of collision detection accuracy
        """
        self.objects = args.get("objects", [])
        self.callback = args.get("callback", None)
        self.level = args.get("level", self.AABB)
        if not callable(self.callback):
            logging.error("callback should be a callable")
        self._pairs = []

    def add_object(self, obj):
        """Add object to test group

        Args:
          obj: Instance of `payton.scene.geometry.Mesh`
        """
        if not isinstance(obj, Mesh):
            logging.error("object must be an instance of Mesh")
            return
        self.objects.append(obj)

    def _dist(self, obj1, obj2):
        """Distance between two given objects"""
        obj1.update_matrix()
        obj2.update_matrix()
        p1 = obj1._model_matrix[3]
        p2 = obj2._model_matrix[3]
        return distance(p1, p2)

    def _bounding_sphere_collision(self, obj1, obj2):
        """Bounding sphere collision test.

        Distance between the centers of two spheres should be longer than
        sum of their radiuses. Otherwise, they are colliding"""
        dist = self._dist(obj1, obj2)
        total_radius = obj1.bounding_radius + obj2.bounding_radius
        return dist <= total_radius

    def _sphere_in_sphere_collision(self, obj1, obj2):
        """Test if one object is completely in another object.

        In some cases, object 1 can be wrapping object 2 or vice versa.
        As a result of this, triangular tests will fail.

        Args:
          obj1: First object to test
          obj2: Second object to test
        """
        dist = self._dist(obj1, obj2)
        if obj1.bounding_radius > dist + obj2.bounding_radius:
            return True
        if obj2.bounding_radius > dist + obj1.bounding_radius:
            return True
        return False

    def _aabb_collision_test(self, obj1, obj2):
        """Axis Aligned Bounding Box collision test

        See: https://en.wikipedia.org/wiki/Minimum_bounding_box
        """
        bb1_min = obj1.to_absolute(obj1._bounding_box[0])
        bb1_max = obj1.to_absolute(obj1._bounding_box[1])
        bb2_min = obj2.to_absolute(obj2._bounding_box[0])
        bb2_max = obj2.to_absolute(obj2._bounding_box[1])

        if (bb1_max[0] < bb2_min[0]) or (bb2_max[0] < bb1_min[0]):
            return False
        if (bb1_max[1] < bb2_min[1]) or (bb2_max[1] < bb1_min[1]):
            return False
        if (bb1_max[2] < bb2_min[2]) or (bb2_max[2] < bb1_min[2]):
            return False
        return True

    def _test(self, obj1, obj2):
        """Test if obj1 and obj2 are colliding"""
        bs_test = self._bounding_sphere_collision(obj1, obj2)
        if not bs_test:
            return False
        if isinstance(obj1, Sphere) and isinstance(obj2, Sphere):
            # If both objects are Spheres, this check is enough
            return True
        # No faces are colliding but an object wraps other one
        if self._sphere_in_sphere_collision(obj1, obj2):
            return True

        if isinstance(obj1, Mesh) and isinstance(obj2, Mesh):
            if self.level == self.SPHERICAL:
                return True
            return self._aabb_collision_test(obj1, obj2)
        return False

    def resolve(self, obj1, obj2):
        """Report that you have solved the collision between objects

        For a better performance, once two objects collide, system will
        not check for a collision between two objects in the next iteration.
        If you want these two objects to be tested again, you have to
        resolve the conflict and report this to collision test class.

        Args:
          obj1: First object in test
          obj2: Second object in test
        """
        pair = [obj1, obj2]
        pair2 = [obj2, obj1]
        if pair in self._pairs:
            self._pairs.remove(pair)
        if pair2 in self._pairs:
            self._pairs.remove(pair2)

    def check(self):
        """Check if any objects within the test group are colliding"""
        for i in range(len(self.objects) - 1):
            for j in range(len(self.objects) - i - 1):
                obj1 = self.objects[i]
                obj2 = self.objects[i + j + 1]
                pair = {}
                pair = [obj1, obj2]
                if pair not in self._pairs:
                    res = self._test(obj1, obj2)
                    if res:
                        self._pairs.append(pair)

        self.callback(self, self._pairs)
