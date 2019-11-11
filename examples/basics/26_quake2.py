"""Quake 2 Model Test"""
import os

import sdl2

from payton.scene import Scene
from payton.scene.controller import Controller
from payton.scene.geometry import MD2

scene = Scene()
object_file = os.path.join(os.path.dirname(__file__), "infantry", "tris.md2")
model = MD2(filename=object_file)


class CustomKeyboardControls(Controller):
    def keyboard(self, event, scene):
        super().keyboard(event, scene)
        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_UP:
                model.animate("walk", 2, 13)
            if key == sdl2.SDLK_DOWN:
                model.animate("death", 0, 19, False)


print(
    """
Hit Keyboard UP to walk
Hit Keyboard Down to death animation
"""
)


scene.add_object("infantry", model)
scene.controller = CustomKeyboardControls()
model.animate("walk", 2, 13)
scene.run()
