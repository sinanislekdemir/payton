# pylama:ignore=W
"""Payton geometry module.

Public 3-D primitives and loaders, all extending :class:`Object`:

* `Cube`, `Sphere`, `Cylinder`, `Capsule`, `Plane`, `MatrixPlane`
* `Line`, `Mesh` (base for custom geometry), `ParticleSystem`
* `Wavefront` (OBJ), `MD2` (Quake II), `NavMesh`, `RagDoll` / `Joint`
* `COLLISION_AUTO`, `COLLISION_BOX`, `COLLISION_SPHERE`, `COLLISION_CAPSULE`
  for the built-in physics collision approximation.

Add instances to a scene with ``scene.add_object(name, obj)``.
"""

from payton.scene.geometry.base import Line, Object
from payton.scene.geometry.capsule import Capsule
from payton.scene.geometry.cube import Cube
from payton.scene.geometry.cylinder import Cylinder
from payton.scene.geometry.md2 import MD2
from payton.scene.geometry.mesh import Mesh
from payton.scene.geometry.navmesh import NavMesh
from payton.scene.geometry.particle import ParticleSystem
from payton.scene.geometry.plane import MatrixPlane, Plane
from payton.scene.geometry.ragdoll import Joint, RagDoll
from payton.scene.geometry.sphere import Sphere
from payton.scene.geometry.wavefront import Wavefront
from payton.scene.internal_physics import (
    COLLISION_AUTO,
    COLLISION_BOX,
    COLLISION_CAPSULE,
    COLLISION_SHAPES,
    COLLISION_SPHERE,
)

__all__ = [
    "COLLISION_AUTO",
    "COLLISION_BOX",
    "COLLISION_CAPSULE",
    "COLLISION_SHAPES",
    "COLLISION_SPHERE",
    "MD2",
    "Capsule",
    "Cube",
    "Cylinder",
    "Joint",
    "Line",
    "MatrixPlane",
    "Mesh",
    "NavMesh",
    "Object",
    "ParticleSystem",
    "Plane",
    "RagDoll",
    "Sphere",
    "Wavefront",
]
