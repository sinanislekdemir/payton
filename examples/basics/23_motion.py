import math

import sdl2

from payton.scene import Scene
from payton.scene.controller import Controller
from payton.scene.geometry import Cube
from payton.scene.gui import info_box

scene = Scene()
forward = 0
rotate = 0


def motion(period, total):
    scene.objects["cube"].forward(forward)
    scene.objects["cube"].rotate_around_z(math.radians(rotate))


class CustomKeyboardControls(Controller):
    def keyboard(self, event, scene):
        global forward
        global rotate
        super().keyboard(event, scene)
        if event.type == sdl2.SDL_KEYDOWN:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_UP:
                forward = 0.02
            if key == sdl2.SDLK_DOWN:
                forward = -0.02
            if key == sdl2.SDLK_LEFT:
                rotate = -1
            if key == sdl2.SDLK_RIGHT:
                rotate = 1
        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_UP or key == sdl2.SDLK_DOWN:
                forward = 0
            if key == sdl2.SDLK_LEFT or key == sdl2.SDLK_RIGHT:
                rotate = 0


cube = Cube()
cube.track_motion = True
scene.controller = CustomKeyboardControls()
scene.add_object("cube", cube)
scene.create_clock("motion", 0.01, motion)

label = """This demo uses clock\n hit SPACE to start

Key Up = forward
Key Down = back
Key Left = turn left
Key Right = turn right
"""

scene.add_object("info", info_box(left=10, top=10, width=220, height=200, label=label))

scene.run()
