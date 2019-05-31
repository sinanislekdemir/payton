"""Scene controller module

This module is mainly for private use.

But if you want to create your own keyboard shortcuts or extended controls,
you can extend this controller for your own needs.
"""
import sdl2
import logging
from payton.scene.observer import BUTTON_LEFT, BUTTON_RIGHT


class Controller(object):
    """SDL2 OpenGL controller."""

    def keyboard(self, event, scene):
        """
        Keyboard event handler.

        Args:
          event: SDL2 Event (by PullEvent)
          scene: Main scene reference
        """
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

        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_LSHIFT:
                scene._shift_down = False
            if key == sdl2.SDLK_LCTRL:
                scene._ctrl_down = False

            if key == sdl2.SDLK_c:
                # below variable assignment is only for code style
                p = scene.active_observer.perspective
                scene.active_observer.perspective = not p
                logging.debug(
                    f"Observer Perspective={scene.active_observer.perspective}"
                )

            if key == sdl2.SDLK_g:
                scene.grid.visible = not scene.grid.visible

            if key == sdl2.SDLK_SPACE:
                for clock in scene.clocks:
                    c = scene.clocks[clock]
                    logging.debug(f"Pause clock [{clock}]")
                    c.pause()

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
                scene._active_observer = active
                for i in range(len(scene.observers)):
                    scene.observers[i].active = i == active

    def mouse(self, event, scene):
        observer = scene.active_observer
        if event.type == sdl2.SDL_MOUSEBUTTONUP:
            observer._prev_intersection = None

        if event.type == sdl2.SDL_MOUSEBUTTONDOWN:
            mx, my = event.button.x, event.button.y

            eye, ray_dir = observer.screen_to_world(
                mx, my, scene.window_width, scene.window_height
            )

            list = []
            for obj in scene.objects:
                hit = scene.objects[obj].select(eye, ray_dir)
                if hit:
                    list.append(scene.objects[obj])

            if callable(scene.on_select):
                scene.on_select(list)

        if event.type == sdl2.SDL_MOUSEMOTION:
            button = -1
            if event.motion.state == sdl2.SDL_BUTTON_LMASK:
                button = BUTTON_LEFT
            if event.motion.state == sdl2.SDL_BUTTON_RMASK:
                button = BUTTON_RIGHT
            observer.mouse(
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
