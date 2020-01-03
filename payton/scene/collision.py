import logging
from itertools import combinations
from typing import Any, Callable, List, Optional, Type

from payton.math.geometry import distance
from payton.scene.geometry.base import Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.geometry.sphere import Sphere


class Collision(object):
    def __init__(self, obj1: Object, obj2: Object) -> None:
        self.object_1 = obj1
        self.object_2 = obj2
        self.segment = 1


class CollisionTest(object):
    """
    TODO: Improve collision detection algorithms.
    """

    SPHERICAL = 0
    AABB = 1

    def __init__(
        self, callback: Callable, level: int = AABB, objects: Optional[List[Type[Mesh]]] = None, **kwargs: Any,
    ) -> None:
        self.objects: List[Type[Mesh]] = [] if objects is None else objects
        self.callback: Callable = callback
        self.level: int = level
        self._pairs: List[List[Mesh]] = []

    def add_object(self, obj: Type[Mesh]) -> None:
        if not isinstance(obj, Mesh):
            logging.error("object must be an instance of Mesh")
            return
        self.objects.append(obj)

    def _dist(self, obj1: Object, obj2: Object) -> float:
        p1 = obj1._model_matrix[3]
        p2 = obj2._model_matrix[3]
        return distance(p1, p2)

    def _bounding_sphere_collision(self, obj1: Object, obj2: Object) -> bool:
        dist = self._dist(obj1, obj2)
        total_radius = obj1.bounding_radius + obj2.bounding_radius
        return dist <= total_radius

    def _sphere_in_sphere_collision(self, obj1: Mesh, obj2: Mesh) -> bool:
        dist = self._dist(obj1, obj2)
        if obj1.bounding_radius > dist + obj2.bounding_radius:  # type: ignore
            return True
        if obj2.bounding_radius > dist + obj1.bounding_radius:  # type: ignore
            return True
        return False

    def _aabb_collision_test(self, obj1: Mesh, obj2: Mesh) -> bool:
        bb1_min = obj1.to_absolute(obj1.bounding_box[0])
        bb1_max = obj1.to_absolute(obj1.bounding_box[1])
        bb2_min = obj2.to_absolute(obj2.bounding_box[0])
        bb2_max = obj2.to_absolute(obj2.bounding_box[1])

        if (bb1_max[0] < bb2_min[0]) or (bb2_max[0] < bb1_min[0]):
            return False
        if (bb1_max[1] < bb2_min[1]) or (bb2_max[1] < bb1_min[1]):
            return False
        if (bb1_max[2] < bb2_min[2]) or (bb2_max[2] < bb1_min[2]):
            return False
        return True

    def _test(self, obj1: Mesh, obj2: Mesh) -> bool:
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

    def resolve(self, obj1: Mesh, obj2: Mesh) -> None:
        pair = [obj1, obj2]
        pair2 = [obj2, obj1]
        if pair in self._pairs:
            self._pairs.remove(pair)
        if pair2 in self._pairs:
            self._pairs.remove(pair2)

    def check(self):
        for obj1, obj2 in combinations(self.objects, 2):
            pair = {}
            pair = [obj1, obj2]
            if pair not in self._pairs:
                res = self._test(obj1, obj2)
                if res:
                    self._pairs.append(pair)

        if len(self._pairs) > 0:
            self.callback(self, self._pairs)
