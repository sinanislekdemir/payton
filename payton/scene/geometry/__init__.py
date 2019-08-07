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
from payton.scene.geometry.base import *
from payton.scene.geometry.mesh import *
from payton.scene.geometry.cylinder import *
from payton.scene.geometry.cube import *
from payton.scene.geometry.plane import *
from payton.scene.geometry.sphere import *
from payton.scene.geometry.point import *
