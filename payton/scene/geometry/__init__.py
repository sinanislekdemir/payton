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
from payton.scene.geometry.base import Line, Object
from payton.scene.geometry.cube import Cube
from payton.scene.geometry.cylinder import Cylinder
from payton.scene.geometry.md2 import MD2
from payton.scene.geometry.mesh import Mesh
from payton.scene.geometry.plane import MatrixPlane, Plane
from payton.scene.geometry.point import PointCloud
from payton.scene.geometry.ragdoll import Joint, RagDoll
from payton.scene.geometry.sphere import Sphere
from payton.scene.geometry.wavefront import Wavefront

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
    "MD2",
    "Wavefront",
    "RagDoll",
    "Joint",
]
