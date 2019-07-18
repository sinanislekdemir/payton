# pylama:ignore=W
"""
Payton main geometry module

Geometry module holds the basic geometry shapes. They are all inherited
from `payton.scene.geometry.Object` class. They are as simple as possible.

Their face informations are generated at the initialization but vertex array
object is not generated until it arrives in render pipeline.

* Object
  * Material (`payton.scene.material`)
    * Shader (`payton.scene.shader`)
"""
from payton.scene.geometries.base import *
from payton.scene.geometries.mesh import *
from payton.scene.geometries.cylinder import *
from payton.scene.geometries.cube import *
from payton.scene.geometries.plane import *
from payton.scene.geometries.sphere import *
