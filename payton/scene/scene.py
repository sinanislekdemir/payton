"""
Main Scene Handler.

Scene is the universe. Everything about Payton happens inside a Scene.
"""
# pylama:ignore=C901
import ctypes
import logging
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar

import numpy as np
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
    GL_MAJOR_VERSION,
    GL_MINOR_VERSION,
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
    GL_VERSION,
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
    glGetIntegerv,
    glGetString,
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
from payton.math.vector import Vector3D
from payton.scene.camera import Camera
from payton.scene.clock import Clock
from payton.scene.collision import CollisionTest
from payton.scene.controller import Controller, GUIController, SceneController
from payton.scene.geometry.base import Object
from payton.scene.grid import Grid
from payton.scene.gui import Hud, Shape2D
from payton.scene.gui.help import help_win
from payton.scene.light import Light
from payton.scene.physics import physics_client
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

try:
    import pybullet
except ModuleNotFoundError:
    pass

S = TypeVar("S", bound="Scene")
# Shadow Qualities
SHADOW_NONE = 0
SHADOW_LOW = 512
SHADOW_MID = 1024
SHADOW_HIGH = 2048


@dataclass
class PhysicsParams:
    """Global physics parameters for the scene."""

    gravity_x: float = 0
    gravity_y: float = 0
    gravity_z: float = -9.8


class Scene(Receiver):
    """Main Scene where everything happens."""

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        on_select: Optional[Callable] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize Scene.

        Scene is the playground of the whole show!

        Keyword arguments:
        width -- Window width for the Scene (default 800)
        height -- Window height for the Scene (default 600)
        on_select -- Callback function to handle object selections in the scene

        On Select method definition:

            def select(selected_object_list: List[Object]):
                pass
        """
        self.__fps_counter = 0
        self.fps = 0
        _env_width = os.getenv('SDL_WINDOW_WIDTH', None)
        _env_height = os.getenv('SDL_WINDOW_HEIGHT', None)
        if _env_width:
            width = int(_env_width)
        if _env_height:
            height = int(_env_height)

        # All objects list
        self.objects: Dict[str, Object] = {}
        # All Huds (Heads Up Display)
        self.huds: Dict[str, Hud] = {"_help": Hud(width=width, height=height)}
        self.huds["_help"].add_child("help", help_win())
        self.huds["_help"].hide()

        self.__timer = -1.0
        self.cameras: List[Camera] = []
        self.cameras.append(Camera(active=True))
        self._physics_params = PhysicsParams(0, 0, -10)
        self.clocks: Dict[str, Clock] = {}

        if physics_client is not None:
            pybullet.setGravity(
                self._physics_params.gravity_x, self._physics_params.gravity_y, self._physics_params.gravity_z
            )
            pybullet.setPhysicsEngineParameter(numSolverIterations=10, minimumSolverIslandSize=1024)
            pybullet.setTimeStep(1.0 / 120.0)
            self.create_clock("_bullet_physics", 1.0 / 120.0, self._step_physics)

        self.hudcam = Camera(
            active=True,
            position=[0, 0, 1.0],
            up=[0, 1.0, 0],
            perspective=False,
        )

        self.active_camera = self.cameras[0]

        self.lights: List[Light] = []
        self.lights.append(Light())

        self.grid = Grid()
        self.controller = Controller()
        self.controller.add_controller(GUIController())
        self.controller.add_controller(SceneController())
        self.background = Background()
        self.shaders: Dict[str, Shader] = {
            DEFAULT_SHADER: Shader(fragment=default_fragment_shader, vertex=default_vertex_shader),
            PARTICLE_SHADER: Shader(
                fragment=particle_fragment_shader, vertex=particle_vertex_shader, geometry=particle_geometry_shader
            ),
            SHADOW_SHADER: Shader(
                fragment=depth_fragment_shader,
                vertex=depth_vertex_shader,
                geometry=depth_geometry_shader,
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

    def _step_physics(self, period: float, total: float) -> None:
        pybullet.stepSimulation()
        for child in self.objects:
            self.objects[child]._bullet_physics()

    @property
    def shadow_samples(self) -> int:
        """Return number of shadow samples."""
        return self._shadow_samples

    @shadow_samples.setter
    def shadow_samples(self, samples: int) -> None:
        """Set shadow samples.

        Keyword arguments:
        samples -- Number of shadow samples (default 20)
        """
        self._shadow_samples = samples % 21
        if self._shadow_samples == 0:
            self._shadow_samples = 1

    @property
    def shadow_quality(self) -> int:
        """Return the shadow quality integer."""
        return self._shadow_quality

    @shadow_quality.setter
    def shadow_quality(self, quality: int) -> None:
        """Set shadow quality.

        Shadow quality is simply the resolution of shadow texture in pixels.
        The higher the resolution gets, the better the shadow is. But that also
        effects the performance in every cycle.

        Keyword arguments:
        quality -- SHADOW_NONE, SHADOW_LOW, SHADOW_MID, SHADOW_HIGH (default SHADOW_MID)
        """
        self._shadow_quality = quality

    def add_click_plane(
        self,
        plane_point: Vector3D,
        plane_normal: Vector3D,
        callback: Callable[[Vector3D], Any],
    ) -> None:
        """Add a click plane to the scene.

        Click planes are infinite 2D planes that can grep and react to clicks.
        You can create a CAD playground or any other stuff that involves selecting
        points in space instead of objects.

        Keyword arguments:
        plane_point -- A point on the surface of the plane.
        plane_normal -- Normal of the plane
        callback -- Callback method to call when a click occurs on the plane

        Callback method definition:

            def hit(hit_point: Vector):
                pass
        """
        self._click_planes.append((plane_point, plane_normal, callback))

    def _check_click_plane(self, eye: Vector3D, vector: Vector3D) -> None:
        """Check if any click planes registered has received a click event."""
        for click_plane in self._click_planes:
            hit = raycast_plane_intersect(eye, vector, click_plane[0], click_plane[1])
            if hit is None:
                continue
            _hit = hit.tolist()
            click_plane[2](_hit[:3])

    def _render_3d_scene(self, shadow_round: bool = False, shader: str = DEFAULT_SHADER) -> None:
        """
        Render the 3D Scene.

        Keyword arguments:
        shadow_round -- Is this render for shadow map creation? (default False)
        shader -- Shader to use for the render pass (default DEFAULT_SHADER)
        """
        light_count = len(self.lights)
        lit = light_count > 0
        if not lit:
            return

        proj, view = self.active_camera.render()
        _shader = self.shaders[shader]
        _shader.set_vector3("camera_pos", self.active_camera.position)
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
        light_array_np = np.array(light_array, dtype=np.float32)
        lcolor_array_np = np.array(lcolor_array, dtype=np.float32)
        _shader.set_vector3_array_np("light_pos", light_array_np, light_count)

        if not shadow_round:
            _shader.set_int("LIGHT_COUNT", light_count)
            _shader.set_int("samples", self._shadow_samples)
            _shader.set_vector3_array_np("light_color", lcolor_array_np, light_count)
        if self.shadow_quality > 0 and not shadow_round:
            _shader.set_int("shadow_enabled", 1)
        if shadow_round:
            shadow_matrices = self.lights[0].shadow_matrices
            for i, mat in enumerate(shadow_matrices):
                _shader.set_matrix4x4_np("shadowMatrices[{}]".format(i), mat)
        elif self.depth_map > -1:
            glActiveTexture(GL_TEXTURE1)
            glBindTexture(GL_TEXTURE_CUBE_MAP, self.depth_map)
            _shader.set_int("depthMap", 1)

        if not shadow_round and shader == DEFAULT_SHADER:
            self.grid.render(lit, self.shaders[DEFAULT_SHADER])

        for object in self.objects.values():
            if object.shader == shader or (shadow_round and object.shader != PARTICLE_SHADER):
                object.render(lit, _shader)

    def _render(self) -> None:
        """
        Render the scene.

        This is the core render method that renders everything to the SDL buffer
        """
        self.shaders["default"].use()
        self._render_lock = True
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

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
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
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
        """
        Add a collision tester to the scene.

        Keyword arguments:
        name -- Name of the collision tester to be added
        tester -- CollisionTest object to be added
        """
        if not isinstance(tester, CollisionTest):
            logging.error("tester must be an instance of CollisionTest")
            return

        self.collisions[name] = tester

    def add_object(self, name: str, obj: Object) -> bool:
        """
        Add an object to the scene.

        Object can be a HUD object, 2D Shape or 3D Mesh.

        Keyword arguments:
        name -- Name of the object to add to the scene
        obj -- 3D Object to be added.
        """
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

    def add_camera(self, camera: Camera) -> bool:
        """Add a camera to the scene and return True if successful.

        If an invalid type of object is given to the scene as an camera,
        it does not break the scene as the scene already has a working camera
        to operate. Instead, failure returns False

        Keyword arguments:
        camera -- Camera object to be added
        """
        if not isinstance(camera, Camera):
            logging.error("Camera is not an instance of `scene.camera`")
            return False

        self.cameras.append(camera)
        return True

    def create_camera(self) -> None:
        """Create a dummy camera with defaults."""
        self.cameras.append(Camera())

    def create_clock(
        self,
        name: str,
        period: float,
        callback: Callable[[float, float], None],
    ) -> None:
        """Create a Clock in the scene.

        Clocks are asyn calls that are triggered in given periods.
        Clocks should be written effectively to take as less time as possible.
        If a clock can not be completed within the given period, there can be two parallel
        processing interfering with eachother.

        Keyword arguments:
        name -- Name of the clock to create
        period -- Period of the callback function in seconds
        callback -- Callback method to call for each "tick" of the clock.

        Callback method definition:

            def logger(period_of_the_clock: float, total_seconds_passed: float)
                pass
        """
        if name in self.clocks:
            logging.error(f"A clock named {name} already exists")
            return

        c = Clock(period, callback)
        self.clocks[name] = c

    def _clear_context(self) -> None:
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

    def to_dict(self) -> Dict[str, Any]:
        """Convert everything inside the Scene to Dictionary.

        This method is better to be called outside the render cycle as it can be
        very costly.
        """
        return {
            "objects": {name: self.objects[name].to_dict() for name in self.objects},
            "lights": [light.to_dict() for light in self.lights],
            "cameras": [camera.to_dict() for camera in self.cameras],
        }

    def raycast_intersect(
        self,
        start: Vector3D,
        direction: Vector3D,
        box_only: bool = True,
        exempt_objects: Optional[List[Object]] = None,
    ) -> Optional[Tuple[Object, Vector3D]]:
        """Raycast a vector to the scene.

        Assume that there is a vector in space starting from "start"
        and going in "vector" direction. This method checks through the
        scene objects and returns the list of objects the vector hits
        with their distances.

        Keyword arguments:
        start -- Start of the vector
        direction -- Direction of the vector
        box_only -- Only make an axis-aligned bounding box collusion check (default True)
        exempt_objects -- Exempt these objects from being tested
        """
        shortest = [0.0, 0.0, 0.0]
        dist_best = -1.0
        hit_obj = None
        for obj in self.objects.values():
            if exempt_objects is not None and obj in exempt_objects:
                continue
            box_hit = raycast_box_intersect(start, direction, obj.bounding_box[0], obj.bounding_box[1])
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
                ip_np, _ = raycast_triangle_intersect(start, direction, p1, p2, p3)
                if ip_np is None:
                    continue
                ip = ip_np.tolist()
                dist = distance_native(ip, start)
                if dist_best == -1.0 or dist < dist_best:
                    dist_best = dist
                    shortest = ip
                    hit_obj = obj
        if hit_obj is None:
            return None
        return hit_obj, shortest

    def run(self, start_clocks: bool = False) -> int:
        """Run the show."""
        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO) != 0:
            return -1

        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_PROFILE_MASK, sdl2.SDL_GL_CONTEXT_PROFILE_CORE)
        multisample_buffers = os.getenv('GL_MULTISAMPLEBUFFERS', None)
        multisample_samples = os.getenv('GL_MULTISAMPLESAMPLES', None)
        if multisample_buffers:
            sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLEBUFFERS, int(multisample_buffers))
        if multisample_samples:
            sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLESAMPLES, int(multisample_samples))

        self.window = sdl2.SDL_CreateWindow(
            b"Payton Scene",
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            int(self.window_width),
            int(self.window_height),
            sdl2.SDL_WINDOW_OPENGL | sdl2.SDL_WINDOW_RESIZABLE,
        )

        if not self.window:
            return -1

        self._context = sdl2.SDL_GL_CreateContext(self.window)
        sdl2.SDL_GL_SetSwapInterval(0)
        self.event = sdl2.SDL_Event()
        self.running = True
        version = glGetString(GL_VERSION).decode('utf-8')
        ogl_major = glGetIntegerv(GL_MAJOR_VERSION)
        ogl_minor = glGetIntegerv(GL_MINOR_VERSION)
        print(f"gl-int: {ogl_major}.{ogl_minor} - {version}")

        v = (ogl_major * 10) + ogl_minor
        if v < 33:  # OpenGL 3.3
            print(
                """
Sorry, Payton couldn't create an OpenGL 3.3 Context
This can be related to an old graphics card / an old driver issue.

Payton requires at least OpenGL 3.3 support and above."""
            )
            print(f"OpenGL Version installed: {ogl_major}.{ogl_minor}")
            exit(16)

        # Fix aspect ratios of cameras
        for camera in self.cameras:
            camera.aspect_ratio = self.window_width / self.window_height * 1.0
        for hud in self.huds.values():
            hud.set_size(self.window_width, self.window_height)

        for clock in self.clocks:
            self.clocks[clock].start()
            if start_clocks:
                self.clocks[clock].pause()

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
            GL_FRAMEBUFFER,
            GL_DEPTH_ATTACHMENT,
            self.depth_map,
            0,
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

                    for ob in self.cameras:
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

    def terminate(self) -> None:
        """Terminate scene."""
        self.running = False
        for clock in self.clocks:
            logging.debug(f"Kill clock [{clock}]")
            self.clocks[clock].kill()
            self.clocks[clock].join()


class Background:
    """(Shader code and idea derived from the original work of.

    https://www.cs.princeton.edu/~mhalber/blog/ogl_gradient/)
    """

    def __init__(
        self,
        top_color: Optional[Vector3D] = None,
        bottom_color: Optional[Vector3D] = None,
        **kwargs: Dict[str, Any],
    ):
        """Initialize the background.

        Keyword arguments:
        top_color -- Gradient color at the top of the viewport
        bottom_color -- Gradient color at the bottom of the viewport
        """
        self.top_color = [0.0, 0.0, 0.0, 1.0] if top_color is None else top_color
        self.bottom_color = [0.0, 0.1, 0.2, 1.0] if bottom_color is None else bottom_color
        variables = ["top_color", "bot_color"]
        self._shader = Shader(
            fragment=background_fragment_shader,
            vertex=background_vertex_shader,
            variables=variables,
        )
        self._vao = None
        self.visible = True

    def set_time(self, hour: int, minute: int) -> None:
        """Background can mimic a background color based on the time of the date.

        It is not accurate but at least gives a small impression

        Keyword arguments:
        hour -- Hour as integer (24 hour format)
        minute -- Minute as integer
        """
        hour %= 24
        minute %= 60
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
        """Render the background."""
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
