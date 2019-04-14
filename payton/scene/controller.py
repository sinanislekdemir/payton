"""Scene controller module

This module is mainly for private use.

But if you want to create your own keyboard shortcuts or extended controls,
you can extend this controller for your own needs.
"""
import pyrr
import numpy as np
import sdl2
import logging
from payton.math.geometry import raycast_sphere_intersect
from payton.scene.observer import BUTTON_LEFT, BUTTON_RIGHT
from payton.scene.geometry import Line


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
            logging.debug('Quit SDL Scene')
            scene.running = False
            for clock in scene.clocks:
                c = scene.clocks[clock]
                logging.debug('Kill clock [{}]'.format(clock))
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
                scene.observers[0].perspective = (not scene
                                                  .observers[0].perspective)
                logging.debug('Observer[0] Perspective = {}'.format(
                    'True' if scene.observers[0].perspective else 'False'))

            if key == sdl2.SDLK_g:
                scene.grid.visible = not scene.grid.visible

            if key == sdl2.SDLK_SPACE:
                for clock in scene.clocks:
                    c = scene.clocks[clock]
                    logging.debug('Pause clock [{}]'.format(clock))
                    c.pause()

            if key == sdl2.SDLK_w:
                for obj in scene.objects:
                    if not isinstance(scene.objects[obj], Line):
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
                    scene.observers[i].active = (i == active)

    def mouse(self, event, scene):
        if event.type == sdl2.SDL_MOUSEBUTTONDOWN:
            observer = scene.observers[scene._active_observer]
            mx, my = event.button.x, event.button.y
            x = (2.0 * mx) / scene.window_width - 1.0
            y = 1.0 - (2.0 * my) / scene.window_height
            z = 1.0

            ray_start = np.array([x, y, -1.0, 1.0], dtype=np.float32)
            proj = observer._projection
            inv_proj = pyrr.matrix44.inverse(proj)
            eye_coords = pyrr.matrix44.apply_to_vector(inv_proj, ray_start)

            eye_coords = np.array([eye_coords[0], eye_coords[1],
                                   -1.0, 0.0], dtype=np.float32)

            view = observer._view
            inv_view = pyrr.matrix44.inverse(view)

            ray_end = pyrr.matrix44.apply_to_vector(inv_view, eye_coords)
            ray_dir = pyrr.vector.normalize(ray_end[0:4])

            # Now shoot the ray to the scene.
            eye = np.array([observer.position[0], observer.position[1],
                            observer.position[2], 1.0], dtype=np.float32)

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
            for o in scene.observers:
                o.mouse(button, scene._shift_down, scene._ctrl_down,
                        event.motion.x,
                        event.motion.y,
                        event.motion.xrel,
                        event.motion.yrel)
