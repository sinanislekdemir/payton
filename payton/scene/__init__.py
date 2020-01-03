# pylama:ignore=W
"""#Payton Scene Module

##Main Scene Module:

* SDL2 Window
  * Scene (`payton.scene.scene.Scene`)
    * Geometry (`payton.scene.geometry`)
    * Grid (`payton.scene.grid`)
    * Background (`payton.scene.scene.Background`)
    * Clock (`payton.scene.clock`)
    * Lighting (`payton.scene.light`)
    * Collision Test (`payton.scene.collision`)
    * Observer (`payton.scene.observer`)
    * GUI (`payton.scene.gui`)
    * Material (`payton.scene.material`)
      * Shader (`payton.scene.shader`)
    * Type Support (`payton.scene.types`)

Also, `payton.scene.scene` contents are accessible through `payton.scene` main
module
"""
from payton.scene.scene import SHADOW_HIGH, SHADOW_LOW, SHADOW_MID, SHADOW_NONE, Background, Scene

__all__ = [
    "Scene",
    "Background",
    "SHADOW_HIGH",
    "SHADOW_LOW",
    "SHADOW_MID",
    "SHADOW_NONE",
]
