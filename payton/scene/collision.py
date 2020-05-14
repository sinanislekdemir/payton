import logging
from itertools import combinations
from typing import Any, Callable, List, Optional, Set, Type

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
        self._pairs: List[Set[Mesh]] = []

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
        if obj1.bounding_radius > dist + obj2.bounding_radius:
            return True
        if obj2.bounding_radius > dist + obj1.bounding_radius:
            return True
        return False

    def _aabb_collision_test(self, obj1: Mesh, obj2: Mesh) -> bool:
        a_min = obj1.bounding_box[0]
        a_max = obj1.bounding_box[1]
        b_min = obj2.bounding_box[0]
        b_max = obj2.bounding_box[1]
        if a_min[0] >= b_max[0] or b_min[0] >= a_max[0]:
            return False
        if a_min[1] >= b_max[1] or b_min[1] >= a_max[1]:
            return False
        if a_min[2] >= b_max[2] or b_min[2] >= a_max[2]:
            return False
        return True

    def _test(self, obj1: Mesh, obj2: Mesh) -> bool:
        bs_test = self._bounding_sphere_collision(obj1, obj2)
        if not bs_test:
            return False
        if self.level == self.SPHERICAL:
            return True
        if isinstance(obj1, Sphere) and isinstance(obj2, Sphere):
            # If both objects are Spheres, this check is enough
            return True
        if isinstance(obj1, Mesh) and isinstance(obj2, Mesh):
            return self._aabb_collision_test(obj1, obj2)
        return False

    def resolve(self, obj1: Mesh, obj2: Mesh) -> None:
        pair = set([obj1, obj2])
        if pair in self._pairs:
            self._pairs.remove(pair)

    def check(self):
        for pair_tuple in combinations(self.objects, 2):
            pair = set(pair_tuple)
            if pair not in self._pairs:
                res = self._test(pair_tuple[0], pair_tuple[1])
                if res:
                    self._pairs.append(pair)

        if len(self._pairs) > 0:
            self.callback(self, [list(pair) for pair in self._pairs])
