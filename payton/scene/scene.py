# pylama:ignore=C901
import ctypes
import logging
import time
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar

import numpy as np  # type: ignore
import sdl2
from OpenGL.GL import (
    GL_CLAMP_TO_EDGE,
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_ATTACHMENT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_COMPONENT,
    GL_DEPTH_TEST,
    GL_FLOAT,
    GL_FRAMEBUFFER,
    GL_LESS,
    GL_NEAREST,
    GL_NONE,
    GL_TEXTURE1,
    GL_TEXTURE_CUBE_MAP,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z,
    GL_TEXTURE_CUBE_MAP_POSITIVE_X,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TEXTURE_WRAP_R,
    GL_TEXTURE_WRAP_S,
    GL_TEXTURE_WRAP_T,
    GL_TRIANGLES,
    glActiveTexture,
    glBindFramebuffer,
    glBindTexture,
    glBindVertexArray,
    glClear,
    glClearColor,
    glDepthFunc,
    glDisable,
    glDrawArrays,
    glDrawBuffer,
    glEnable,
    glFramebufferTexture,
    glGenFramebuffers,
    glGenTextures,
    glGenVertexArrays,
    glReadBuffer,
    glTexImage2D,
    glTexParameteri,
    glViewport,
)

from payton.math.geometry import (
    distance_native,
    raycast_box_intersect,
    raycast_plane_intersect,
    raycast_triangle_intersect,
)
from payton.scene.clock import Clock
from payton.scene.collision import CollisionTest
from payton.scene.controller import Controller
from payton.scene.geometry.base import Object
from payton.scene.grid import Grid
from payton.scene.gui import Hud, Shape2D
from payton.scene.gui.help import help_win
from payton.scene.light import Light
from payton.scene.observer import Observer
from payton.scene.receiver import Receiver
from payton.scene.shader import (
    DEFAULT_SHADER,
    PARTICLE_SHADER,
    SHADOW_SHADER,
    Shader,
    background_fragment_shader,
    background_vertex_shader,
    default_fragment_shader,
    default_vertex_shader,
    depth_fragment_shader,
    depth_geometry_shader,
    depth_vertex_shader,
    particle_fragment_shader,
    particle_geometry_shader,
    particle_vertex_shader,
)
from payton.scene.types import CPlane

S = TypeVar("S", bound="Scene")
# Shadow Qualities
SHADOW_NONE = 0
SHADOW_LOW = 512
SHADOW_MID = 1024
SHADOW_HIGH = 2048


class Scene(Receiver):
    def __init__(
        self, width: int = 800, height: int = 600, on_select: Optional[Callable] = None, **kwargs: Any,
    ) -> None:
        self.__fps_counter = 0
        self.fps = 0
        # All objects list
        self.objects: Dict[str, Object] = {}
        # All Huds (Heads Up Display)
        self.huds: Dict[str, Hud] = {"_help": Hud(width=width, height=height)}
        self.huds["_help"].add_child("help", help_win())
        self.huds["_help"].hide()

        self.__timer = -1.0
        self.observers: List[Observer] = []
        self.observers.append(Observer(active=True))

        self.hudcam = Observer(active=True, position=[0, 0, 1.0], up=[0, 1.0, 0], perspective=False,)

        self.active_observer = self.observers[0]

        self.lights: List[Light] = []
        self.lights.append(Light())

        self.clocks: Dict[str, Clock] = {}
        self.grid = Grid()
        self.controller = Controller()
        self.background = Background()
        self.shaders: Dict[str, Shader] = {
            DEFAULT_SHADER: Shader(fragment=default_fragment_shader, vertex=default_vertex_shader),
            PARTICLE_SHADER: Shader(
                fragment=particle_fragment_shader, vertex=particle_vertex_shader, geometry=particle_geometry_shader
            ),
            SHADOW_SHADER: Shader(
                fragment=depth_fragment_shader, vertex=depth_vertex_shader, geometry=depth_geometry_shader,
            ),
        }
        self.shaders["depth"]._depth_shader = True

        # SDL Related Stuff
        self.window = None
        self.window_width = width
        self.window_height = height
        self._context = None
        self._mouse = [0, 0]
        self._shift_down = False
        self._ctrl_down = False
        self._rotate = False
        self.collisions: Dict[str, CollisionTest] = {}
        self._click_planes: List[CPlane] = []

        self.on_select = on_select
        self.depth_map = 0
        self.depth_map_fbo = 0
        # Main running state
        self.running = False
        self._render_lock = False
        self._shadow_quality = SHADOW_MID
        self._shadow_samples = 20

    @property
    def shadow_samples(self):
        return self._shadow_samples

    @shadow_samples.setter
    def shadow_samples(self, samples: int):
        self._shadow_samples = samples % 21
        if self._shadow_samples == 0:
            self._shadow_samples = 1

    @property
    def shadow_quality(self):
        return self._shadow_quality

    @shadow_quality.setter
    def shadow_quality(self, quality: int):
        self._shadow_quality = quality

    def add_click_plane(
        self, plane_point: List[float], plane_normal: List[float], callback: Callable[[List[float]], Any],
    ):
        pn = plane_normal.copy() + [0.0]
        pp = plane_point.copy() + [1.0]
        self._click_planes.append((pp, pn, callback))

    def _check_click_plane(self, eye: List[float], vector: List[float]) -> None:
        for click_plane in self._click_planes:
            hit = raycast_plane_intersect(eye, vector, click_plane[0], click_plane[1])
            if hit is None:
                continue
            click_plane[2](hit[:3])

    def _render_3d_scene(self, shadow_round=False, shader=DEFAULT_SHADER) -> None:
        light_count = len(self.lights)
        lit = light_count > 0
        if not lit:
            return

        proj, view = self.active_observer.render()
        _shader = self.shaders[shader]
        _shader.set_vector3("camera_pos", self.active_observer.position)
        if view is None:
            if not shadow_round:
                _shader.set_int("view_mode", 1)
        else:
            _shader.set_matrix4x4_np("view", view)
            if not shadow_round:
                _shader.set_int("view_mode", 0)

        _shader.set_float("far_plane", self.lights[0].shadow_far_plane)
        _shader.set_matrix4x4_np("projection", proj)
        light_array = [light.position for light in self.lights]
        lcolor_array = [light.color for light in self.lights]
        light_array = np.array(light_array, dtype=np.float32)
        lcolor_array = np.array(lcolor_array, dtype=np.float32)
        _shader.set_vector3_array_np("light_pos", light_array, light_count)

        if not shadow_round:
            _shader.set_int("LIGHT_COUNT", light_count)
            _shader.set_int("samples", self._shadow_samples)
            _shader.set_vector3_array_np("light_color", lcolor_array, light_count)
        if self.shadow_quality > 0 and not shadow_round:
            _shader.set_int("shadow_enabled", 1)
        if shadow_round:
            shadow_matrices = self.lights[0].shadow_matrices
            for i, mat in enumerate(shadow_matrices):
                _shader.set_matrix4x4_np("shadowMatrices[{}]".format(i), mat)
        else:
            if self.depth_map > -1:
                glActiveTexture(GL_TEXTURE1)
                glBindTexture(GL_TEXTURE_CUBE_MAP, self.depth_map)
                _shader.set_int("depthMap", 1)

        if not shadow_round and shader == DEFAULT_SHADER:
            self.grid.render(lit, self.shaders[DEFAULT_SHADER])

        for object in self.objects.values():
            if object.shader == shader or (shadow_round and object.shader != PARTICLE_SHADER):
                object.render(lit, _shader)

    def _render(self) -> None:
        self.shaders["default"].use()
        self._render_lock = True
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # type: ignore

        if self.shadow_quality > 0:
            glViewport(0, 0, self._shadow_quality, self._shadow_quality)
            glBindFramebuffer(GL_FRAMEBUFFER, self.depth_map_fbo)
            glClear(GL_DEPTH_BUFFER_BIT)
            self.shaders[SHADOW_SHADER].use()
            self._render_3d_scene(True, SHADOW_SHADER)
            glBindFramebuffer(GL_FRAMEBUFFER, 0)
            self.shaders[SHADOW_SHADER].end()

        # Render background
        glViewport(0, 0, self.window_width, self.window_height)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)  # type: ignore
        self.background.render()
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        # Render scene
        lit = len(self.lights) > 0
        self.shaders[DEFAULT_SHADER].use()
        self._render_3d_scene(False, DEFAULT_SHADER)

        # Render HUD
        for object in self.huds:
            self.huds[object].render(lit, self.shaders[DEFAULT_SHADER])
        self.shaders[DEFAULT_SHADER].end()

        self.shaders[PARTICLE_SHADER].use()
        self._render_3d_scene(False, PARTICLE_SHADER)
        self.shaders[PARTICLE_SHADER].end()

        self._render_lock = False

        for test in self.collisions.values():
            test.check()

        self.__fps_counter += 1
        if self.__timer == -1:
            self.__timer = time.time()
        diff = time.time() - self.__timer
        if diff > 1:
            self.__timer = time.time()
            self.fps = self.__fps_counter
            self.__fps_counter = 0

    def add_collision_test(self, name: str, tester: CollisionTest) -> None:
        if not isinstance(tester, CollisionTest):
            logging.error("tester must be an instance of CollisionTest")
            return

        self.collisions[name] = tester

    def add_object(self, name: str, obj: Object) -> bool:
        if not isinstance(obj, Object):
            logging.error("Given object is not an instance of `scene.Object`")
            return False

        if name in self.objects:
            logging.error(f"Given object name [{name}] already exists")
            return False

        if isinstance(obj, Shape2D):
            logging.error("2D Shapes can't be added directly to the scene")
            return False

        if isinstance(obj, Hud):
            """Huds must be rendered in a different loop after rendering
            all objects"""
            if self._render_lock:
                while self._render_lock:
                    # Wait for render loop to release the lock
                    # TODO: This can cause a possible deadlock.
                    continue
            self.huds[name] = obj
            obj.name = name
            obj.set_size(self.window_width, self.window_height)
            return True

        if self._render_lock:
            while self._render_lock:
                # Wait for render loop to release the lock
                # TODO: This can cause a possible deadlock.
                continue
        self.objects[name] = obj
        obj.name = name
        return True

    def add_observer(self, obj: Observer) -> bool:
        if not isinstance(obj, Observer):
            logging.error("Observer is not an instance of `scene.Observer`")
            return False

        self.observers.append(obj)
        return True

    def create_observer(self) -> None:
        self.observers.append(Observer())

    def create_clock(self, name: str, period: float, callback: Callable[[float, float], None],) -> None:
        if name in self.clocks:
            logging.error(f"A clock named {name} already exists")
            return

        c = Clock(period, callback)
        self.clocks[name] = c

    def _clear_context(self):
        for shader in self.shaders.values():
            shader.program = -1
        self.background._shader.program = -1
        self.background._vao = None
        self.grid.destroy()
        for hud in self.huds.values():
            hud.destroy()
        # As Payton clocks are basically threads,
        # best practice is to create new threads as Python does not let
        # a thread to be started twice by design.
        new_clocks = {}
        for clock_name in self.clocks:
            clock = self.clocks[clock_name]
            new_clock = Clock(clock.period, clock.callback)
            new_clock._total_time = clock._total_time
            new_clocks[clock_name] = new_clock

        self.clocks = new_clocks

    def to_dict(self):
        result = {
            "objects": {name: self.objects[name].to_dict() for name in self.objects},
            "lights": [light.to_dict() for light in self.lights],
            "observers": [observer.to_dict() for observer in self.observers],
        }
        return result

    def raycast_intersect(
        self,
        start: List[float],
        vector: List[float],
        box_only: bool = True,
        exempt_objects: Optional[List[Object]] = None,
    ) -> Optional[Tuple[Object, List[float]]]:
        shortest = [0.0, 0.0, 0.0]
        dist_best = -1.0
        hit_obj = None
        for obj in self.objects.values():
            if exempt_objects is not None:
                if obj in exempt_objects:
                    continue
            box_hit = raycast_box_intersect(start, vector, obj.bounding_box[0], obj.bounding_box[1])
            if box_hit is None:
                continue
            if box_only:
                dist = distance_native(box_hit, start)
                if dist_best == -1.0 or dist < dist_best:
                    dist_best = dist
                    shortest = box_hit
                    hit_obj = obj
                    continue
            for p1, p2, p3 in zip(*[iter(obj.absolute_vertices())] * 3):
                ip, _ = raycast_triangle_intersect(start, vector, p1, p2, p3)
                if ip is None:
                    continue
                dist = distance_native(ip, start)
                if dist_best == -1.0 or dist < dist_best:
                    dist_best = dist
                    shortest = ip
                    hit_obj = obj
        if hit_obj is None:
            return None
        return hit_obj, shortest

    def run(self) -> int:
        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO) != 0:
            return -1
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MAJOR_VERSION, 3)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MINOR_VERSION, 3)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_PROFILE_MASK, sdl2.SDL_GL_CONTEXT_PROFILE_CORE)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLEBUFFERS, 1)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLESAMPLES, 16)

        self.window = sdl2.SDL_CreateWindow(
            b"Payton Scene",
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            int(self.window_width),
            int(self.window_height),
            sdl2.SDL_WINDOW_OPENGL | sdl2.SDL_WINDOW_RESIZABLE,
        )  # type: ignore

        if not self.window:
            return -1

        self._context = sdl2.SDL_GL_CreateContext(self.window)
        sdl2.SDL_GL_SetSwapInterval(0)
        self.event = sdl2.SDL_Event()
        self.running = True

        # Fix aspect ratios of observers
        for observer in self.observers:
            observer.aspect_ratio = self.window_width / self.window_height * 1.0
        for hud in self.huds.values():
            hud.set_size(self.window_width, self.window_height)

        for clock in self.clocks:
            self.clocks[clock].start()

        # if shadows
        # @TODO Creating the shadow cubemap here is not a good idea
        self.depth_map_fbo = glGenFramebuffers(1)
        self.depth_map = glGenTextures(1)
        side_map = [
            GL_TEXTURE_CUBE_MAP_POSITIVE_X,
            GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
            GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
            GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
            GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
            GL_TEXTURE_CUBE_MAP_NEGATIVE_Z,
        ]
        glBindTexture(GL_TEXTURE_CUBE_MAP, self.depth_map)
        for i in side_map:
            glTexImage2D(
                i,
                0,
                GL_DEPTH_COMPONENT,
                self.shadow_quality,
                self.shadow_quality,
                0,
                GL_DEPTH_COMPONENT,
                GL_FLOAT,
                None,
            )

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)

        glBindFramebuffer(GL_FRAMEBUFFER, self.depth_map_fbo)
        glFramebufferTexture(
            GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, self.depth_map, 0,
        )
        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        for shader in self.shaders.values():
            shader.build()

        while self.running:
            while sdl2.SDL_PollEvent(ctypes.byref(self.event)) != 0:
                if self.event.type == sdl2.SDL_WINDOWEVENT and self.event.window.event == sdl2.SDL_WINDOWEVENT_RESIZED:
                    self.window_width = self.event.window.data1
                    self.window_height = self.event.window.data2
                    glViewport(0, 0, self.window_width, self.window_height)

                    for ob in self.observers:
                        ob.aspect_ratio = self.window_width / self.window_height
                    for hud in self.huds:
                        self.huds[hud].set_size(self.window_width, self.window_height)
                self.controller.keyboard(self.event, self)
                self.controller.mouse(self.event, self)

            self._render()

            sdl2.SDL_GL_SwapWindow(self.window)
            sdl2.SDL_Delay(1)

        for obj in self.objects:
            self.objects[obj].destroy()

        for clock in self.clocks:
            self.clocks[clock].kill()
            self.clocks[clock]._hold = True

        sdl2.SDL_GL_DeleteContext(self._context)
        sdl2.SDL_DestroyWindow(self.window)
        self.window = None
        sdl2.SDL_Quit()
        self._clear_context()
        return 0


class Background(object):
    """
    (Shader code and idea derived from the original work of:
    https://www.cs.princeton.edu/~mhalber/blog/ogl_gradient/)
    """

    def __init__(
        self,
        top_color: Optional[List[float]] = None,
        bottom_color: Optional[List[float]] = None,
        **kwargs: Dict[str, Any],
    ):
        self.top_color = [0.0, 0.0, 0.0, 1.0] if top_color is None else top_color
        self.bottom_color = [0.0, 0.1, 0.2, 1.0] if bottom_color is None else bottom_color
        variables = ["top_color", "bot_color"]
        self._shader = Shader(
            fragment=background_fragment_shader, vertex=background_vertex_shader, variables=variables,
        )
        self._vao = None
        self.visible = True

    def set_time(self, hour: int, minute: int):
        hour = hour % 24
        minute = minute % 60
        color_scheme = {
            (0, 3): [[24 / 255, 24 / 255, 48 / 255, 1.0], [24 / 255, 48 / 255, 72 / 255, 1.0]],
            (3, 9): [[24 / 255, 48 / 255, 72 / 255, 1.0], [96 / 255, 168 / 255, 192 / 255, 1.0]],
            (9, 15): [[96 / 255, 168 / 255, 192 / 255, 1.0], [144 / 255, 192 / 255, 240 / 255, 1.0]],
            (15, 21): [[144 / 255, 192 / 255, 240 / 255, 1.0], [48 / 255, 72 / 255, 120 / 255, 1.0]],
            (21, 24): [[48 / 255, 72 / 255, 120 / 255, 1.0], [24 / 255, 24 / 255, 48 / 255, 1.0]],
        }
        for color in color_scheme:
            if hour >= color[0] and hour < color[1]:
                hours = color[1] - color[0]
                minutes = hours * 60
                minute = ((hour - color[0]) * 60) + minute
                xdist = ((color_scheme[color][1][0] - color_scheme[color][0][0]) / minutes) * minute
                ydist = ((color_scheme[color][1][1] - color_scheme[color][0][1]) / minutes) * minute
                zdist = ((color_scheme[color][1][2] - color_scheme[color][0][2]) / minutes) * minute

                self.bottom_color = color_scheme[color][0]
                self.top_color = [
                    color_scheme[color][0][0] + xdist,
                    color_scheme[color][0][1] + ydist,
                    color_scheme[color][0][2] + zdist,
                    1.0,
                ]
                return

    def render(self) -> None:
        if not self.visible:
            return

        if not self._vao:
            self._vao = glGenVertexArrays(1)
            glBindVertexArray(self._vao)
            self._shader.build()
            glBindVertexArray(0)

        glDisable(GL_DEPTH_TEST)

        self._shader.use()
        self._shader.set_vector4_np("top_color", np.array(self.top_color, dtype=np.float32))
        self._shader.set_vector4_np("bot_color", np.array(self.bottom_color, dtype=np.float32))
        glBindVertexArray(self._vao)
        glDrawArrays(GL_TRIANGLES, 0, 3)
        glBindVertexArray(0)
        self._shader.end()
        glEnable(GL_DEPTH_TEST)
