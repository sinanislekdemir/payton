# pylama:ignore=C901
import logging
from typing import Any, Optional

import sdl2

from payton.scene.gui import EditBox
from payton.scene.observer import BUTTON_LEFT, BUTTON_MIDDLE, BUTTON_RIGHT


class Controller(object):
    def __init__(self):
        super().__init__()
        self._active_object: Optional[EditBox] = None

    def keyboard(self, event: sdl2.SDL_Event, scene: Any) -> None:
        if event.type == sdl2.SDL_QUIT:
            logging.debug("Quit SDL Scene")
            scene.running = False
            for clock in scene.clocks:
                c = scene.clocks[clock]
                logging.debug(f"Kill clock [{clock}]")
                c.kill()
                c.join()

        if event.type == sdl2.SDL_KEYDOWN:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_LSHIFT:
                scene._shift_down = True
            if key == sdl2.SDLK_LCTRL:
                scene._ctrl_down = True

        if event.type == sdl2.SDL_TEXTINPUT:
            if self._active_object is not None:
                self._active_object._on_keypress(event.text.text.decode('utf-8'))

        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym

            # GUI Related Controls, move to another place when you have time
            if self._active_object is not None:
                if key == sdl2.SDLK_ESCAPE or (key == sdl2.SDLK_RETURN and not self._active_object.multiline):
                    self._active_object._exit()
                    self._active_object = None
                    sdl2.SDL_ShowCursor(True)
                    sdl2.SDL_StopTextInput()
                if key == sdl2.SDLK_BACKSPACE and self._active_object is not None:
                    self._active_object.backspace()
                if key == sdl2.SDLK_RETURN and self._active_object is not None and self._active_object.multiline:
                    self._active_object._on_keypress("\n")
                if self._active_object and key == sdl2.SDLK_LEFT:
                    self._active_object.cursor_left()
                if self._active_object and key == sdl2.SDLK_RIGHT:
                    self._active_object.cursor_right()
                return

            if key == sdl2.SDLK_LSHIFT:
                scene._shift_down = False
            if key == sdl2.SDLK_LCTRL:
                scene._ctrl_down = False

            if key == sdl2.SDLK_c:
                # below variable assignment is only for code style
                p = scene.active_observer.perspective
                scene.active_observer.perspective = not p
                logging.debug(f"Observer Perspective={scene.active_observer.perspective}")

            if key == sdl2.SDLK_g:
                scene.grid.visible = not scene.grid.visible

            if key == sdl2.SDLK_SPACE:
                for clock in scene.clocks:
                    c = scene.clocks[clock]
                    logging.debug(f"Pause clock [{clock}]")
                    c.pause()

            if key == sdl2.SDLK_h:
                scene.huds["_help"]._visible = not scene.huds["_help"]._visible

            if key == sdl2.SDLK_w:
                for obj in scene.objects:
                    scene.objects[obj].toggle_wireframe()

            if key == sdl2.SDLK_ESCAPE:
                scene.running = False

            if key in [sdl2.SDLK_F2, sdl2.SDLK_F3]:
                active = 0
                for i in range(len(scene.observers)):
                    if scene.observers[i].active:
                        active = i
                if key == sdl2.SDLK_F2:
                    active -= 1
                if key == sdl2.SDLK_F3:
                    active += 1
                if active < 0:
                    active = len(scene.observers) - 1
                if active > len(scene.observers) - 1:
                    active = 0
                scene.active_observer = scene.observers[active]
                for i in range(len(scene.observers)):
                    scene.observers[i].active = i == active

    def mouse(self, event: sdl2.SDL_Event, scene: Any) -> None:
        observer = scene.active_observer
        if event.type == sdl2.SDL_MOUSEBUTTONUP:
            observer._prev_intersection = None

        if event.type == sdl2.SDL_MOUSEBUTTONDOWN:
            mx, my = event.button.x, event.button.y
            for hud in scene.huds:
                h = scene.huds[hud]
                for shape in h.children:
                    focus_element = h.children[shape].click(mx, my)
                    if focus_element:
                        if self._active_object is not None and self._active_object != focus_element:
                            self._active_object._exit()
                            self._active_object = None
                            sdl2.SDL_ShowCursor(True)
                            sdl2.SDL_StopTextInput()
                        if hasattr(focus_element, "_on_keypress"):
                            self._active_object = focus_element
                            sdl2.SDL_ShowCursor(False)
                            sdl2.SDL_StartTextInput()
                        return

            eye, ray_dir = observer.screen_to_world(mx, my, scene.window_width, scene.window_height)

            list = []
            if callable(scene.on_select):
                for obj in scene.objects:
                    hit = scene.objects[obj].select(eye, ray_dir)
                    if hit:
                        list.append(scene.objects[obj])
                if len(list) > 0:
                    scene.on_select(list)

            if not (scene._shift_down or scene._ctrl_down):
                scene._check_click_plane(eye, ray_dir)

        if event.type == sdl2.SDL_MOUSEWHEEL:
            yrel = event.wheel.y
            observer.mouse_wheel(yrel)

        if event.type == sdl2.SDL_MOUSEMOTION:
            button = -1
            if event.motion.state == sdl2.SDL_BUTTON_LMASK:
                button = BUTTON_LEFT
            if event.motion.state == sdl2.SDL_BUTTON_RMASK:
                button = BUTTON_RIGHT
            if event.motion.state == sdl2.SDL_BUTTON_MIDDLE:
                button = BUTTON_MIDDLE
            observer.mouse_move(
                button,
                scene._shift_down,
                scene._ctrl_down,
                event.motion.x,
                event.motion.y,
                event.motion.xrel,
                event.motion.yrel,
                scene.window_width,
                scene.window_height,
            )
