# pylama:ignore=W
"""
Payton main geometry module

Geometry module holds the basic geometry shapes. They are all inherited
from `payton.scene.geometry.base.Object` class. They are as simple as possible.

Their face informations are generated at the initialization but vertex array
object is not generated until it arrives in render pipeline.

* Object
  * Material (`payton.scene.material`)
    * Shader (`payton.scene.shader`)
"""
from payton.scene.geometry.base import Object, Line
from payton.scene.geometry.mesh import Mesh
from payton.scene.geometry.cylinder import Cylinder
from payton.scene.geometry.cube import Cube
from payton.scene.geometry.plane import Plane, MatrixPlane
from payton.scene.geometry.sphere import Sphere
from payton.scene.geometry.point import PointCloud


__all__ = [
    "Object",
    "Line",
    "Mesh",
    "Cylinder",
    "Cube",
    "Plane",
    "MatrixPlane",
    "Sphere",
    "PointCloud",
]
