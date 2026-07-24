"""Payton Scene Module.

##Main Scene Module:

* SDL2 Window
  * Scene (`payton.scene.scene.Scene`)
    * Geometry (`payton.scene.geometry`)
    * Grid (`payton.scene.grid`)
    * Background (`payton.scene.scene.Background`)
    * Clock (`payton.scene.clock`)
    * Lighting (`payton.scene.light`)
    * Collision Test (`payton.scene.collision`)
    * Camera (`payton.scene.camera`)
    * GUI (`payton.scene.gui`)
    * Material (`payton.scene.material`)
      * Shader (`payton.scene.shader`)
    * Type Support (`payton.scene.types`)

Also, `payton.scene.scene` contents are accessible through `payton.scene` main
module
"""

# pylama:ignore=W
from payton.scene.physics import physics_client
from payton.scene.scene import (
    SHADOW_HIGH,
    SHADOW_LOW,
    SHADOW_MID,
    SHADOW_NONE,
    Background,
    Scene,
)
from payton.scene.theme import THEME_BLENDER, THEME_GAMEENGINE, THEME_STUDIO, SceneTheme

__all__ = [
    "SHADOW_HIGH",
    "SHADOW_LOW",
    "SHADOW_MID",
    "SHADOW_NONE",
    "THEME_BLENDER",
    "THEME_GAMEENGINE",
    "THEME_STUDIO",
    "Background",
    "Scene",
    "SceneTheme",
    "physics_client",
]
