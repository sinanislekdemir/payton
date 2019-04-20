"""
Collision detection module
"""
import math
import logging
import numpy as np
from payton.scene.geometry import Sphere
from payton.math.geometry import (distance, line_triangle_intersect)


class Collision(object):
    """Collision detection class

    Collision detection is made between selected objects so instead
    of trying to make the decision between all scene objects, specific
    objects must be selected for testing.

    Collision detection is a costly operation and often times, to increase
    the performance, making some assumptions can work well. As an instance,
    objects' bounding sphere check is a very simple arithmetic operation.
    But collision detection using mesh triangles is quite costly.

    Therefore, collision detection class uses some approximations.
    """
    def __init__(self, **args):
        """Initialize collision detector

        Args:
          objects: List of object references to test
          callback: Callback function to call incase of collision
        """
        self.objects = args.get('objects', [])
        self.callback = args.get('callback', None)
        if not callable(self.callback):
            logging.error('callback should be a callable')

    def add_object(self, obj):
        self.objects.append(obj)

    def _dist(self, obj1, obj2):
        obj1.update_matrix()
        obj2.update_matrix()
        p1 = obj1._model_matrix[3]
        p2 = obj2._model_matrix[3]
        return distance(p1, p2)

    def _bounding_sphere_collision(self, obj1, obj2):
        dist = self._dist(obj1, obj2)
        total_radius = obj1.bounding_radius + obj2.bounding_radius
        return dist <= total_radius

    def _mesh_collision(self, obj1, obj2):
        for i in range(math.floor(len(obj1._indices) / 3)):
            ix = i * 3
            i1, i2, i3 = (obj1._indices[ix],
                          obj1._indices[ix + 1],
                          obj1._indices[ix + 2])
            p1 = [obj1._vertices[i1 * 3],
                  obj1._vertices[i1 * 3 + 1],
                  obj1._vertices[i1 * 3 + 2]]
            p2 = [obj1._vertices[i2 * 3],
                  obj1._vertices[i2 * 3 + 1],
                  obj1._vertices[i2 * 3 + 2]]
            p3 = [obj1._vertices[i3 * 3],
                  obj1._vertices[i3 * 3 + 1],
                  obj1._vertices[i3 * 3 + 2]]
            p1 = np.array(p1, dtype=np.float32)
            p2 = np.array(p2, dtype=np.float32)
            p3 = np.array(p3, dtype=np.float32)

            for j in range(math.floor(len(obj2._indices) / 3)):
                jx = j * 3
                i1, i2, i3 = (obj2._indices[jx],
                              obj2._indices[jx + 1],
                              obj1._indices[jx + 2])
                q1 = [obj2._vertices[i1 * 3],
                      obj2._vertices[i1 * 3 + 1],
                      obj2._vertices[i1 * 3 + 2]]
                q2 = [obj2._vertices[i2 * 3],
                      obj2._vertices[i2 * 3 + 1],
                      obj2._vertices[i2 * 3 + 2]]
                q3 = [obj2._vertices[i3 * 3],
                      obj2._vertices[i3 * 3 + 1],
                      obj2._vertices[i3 * 3 + 2]]
                q1 = np.array(q1, dtype=np.float32)
                q2 = np.array(q2, dtype=np.float32)
                q3 = np.array(q3, dtype=np.float32)
                c = line_triangle_intersect(p1, p2,
                                            q1, q2, q3)
                if c:
                    return True
                c = line_triangle_intersect(p1, p3,
                                            q1, q2, q3)
                if c:
                    return True
                c = line_triangle_intersect(p2, p3,
                                            q1, q2, q3)
                if c:
                    return True
                c = line_triangle_intersect(q1, q2,
                                            p1, p2, p3)
                if c:
                    return True
                c = line_triangle_intersect(q1, q3,
                                            p1, p2, p3)
                if c:
                    return True
                c = line_triangle_intersect(q2, q3,
                                            p1, p2, p3)
                if c:
                    return True
        return False

    def _test(self, obj1, obj2):
        if isinstance(obj1, Sphere) and isinstance(obj2, Sphere):
            return self._bounding_sphere_collision(obj1, obj2)
        if not self._bounding_sphere_collision(obj1, obj2):
            return False
        return self._mesh_collision(obj1, obj2)

    def check(self):
        pairs = []
        for i in range(len(self.objects) - 1):
            for j in range(len(self.objects) - i - 1):
                obj1 = self.objects[i]
                obj2 = self.objects[i+j+1]
                res = self._test(obj1, obj2)
                if res:
                    pairs.append([obj1, obj2])

        self.callback(pairs)
