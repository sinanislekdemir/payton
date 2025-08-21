"""payton.scene.scene

High-level scene manager for Payton.

The :class:`Scene` contains objects, cameras, lights, HUD elements, physics
and the render loop. This module provides the Scene class and the
Background helper used by the renderer. Only documentation strings were
improved here; runtime behaviour is unchanged.
"""

# pylama:ignore=C901
import ctypes
import logging
import os
import sys
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar, cast

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
    GL_FRAMEBUFFER_BINDING,
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
    """Main scene container.

    The Scene manages 3D objects, HUDs, cameras, lights, collision tests,
    physics clocks and the render loop. It is responsible for initializing
    OpenGL runtime resources, dispatching input to controllers, and
    coordinating rendering passes (including shadow-map passes).
    """

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        on_select: Optional[Callable] = None,
        physics_force_continuous: bool = False,
        **kwargs: Any,
    ) -> None:
        """Create a new Scene.

        Parameters
        ----------
        width : int, optional
            Initial window width in pixels. Default is 800.
        height : int, optional
            Initial window height in pixels. Default is 600.
        on_select : callable or None, optional
            Optional callback invoked when objects are selected. The
            callback receives a list of selected :class:`Object` instances.
        physics_force_continuous : bool, optional
            If True, forces the physics clock to run continuously.
        **kwargs
            Ignored; kept for backward compatibility.
        """
        self.__fps_counter = 0
        self.fps = 0
        _env_width = os.getenv("SDL_WINDOW_WIDTH", None)
        _env_height = os.getenv("SDL_WINDOW_HEIGHT", None)
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
        self.cameras.append(Camera(active=True, viewport_size=[width, height, 0]))
        self._physics_params = PhysicsParams(0, 0, -10)
        self.clocks: Dict[str, Clock] = {}

        if physics_client is not None:
            pybullet.setGravity(
                self._physics_params.gravity_x,
                self._physics_params.gravity_y,
                self._physics_params.gravity_z,
            )
            pybullet.setPhysicsEngineParameter(
                numSolverIterations=10, minimumSolverIslandSize=1024
            )
            pybullet.setTimeStep(1.0 / 120.0)
            pybullet.connect(pybullet.DIRECT)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_KEYBOARD_SHORTCUTS, 0)
            pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
            self.create_clock(
                "_bullet_physics",
                1.0 / 120.0,
                self._step_physics,
                physics_force_continuous,
            )

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
            DEFAULT_SHADER: Shader(
                fragment=default_fragment_shader, vertex=default_vertex_shader
            ),
            PARTICLE_SHADER: Shader(
                fragment=particle_fragment_shader,
                vertex=particle_vertex_shader,
                geometry=particle_geometry_shader,
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
        """Advance the physics simulation one step.

        This method is intended to be called from a Clock. It advances the
        PyBullet simulation and forwards a physics tick to every object that
        implements the ``_bullet_physics`` hook.

        Parameters
        ----------
        period : float
            The period passed by the clock (seconds between ticks).
        total : float
            Accumulated time since clock start (seconds).
        """
        pybullet.stepSimulation()
        for child in self.objects:
            self.objects[child]._bullet_physics()

    @property
    def shadow_samples(self) -> int:
        """Number of shadow samples used for soft shadow sampling.

        Higher values improve shadow smoothness at the cost of performance.
        """
        return self._shadow_samples

    @shadow_samples.setter
    def shadow_samples(self, samples: int) -> None:
        """Set the number of shadow samples.

        Parameters
        ----------
        samples : int
            Desired number of samples. The value will be normalized into the
            range [1, 20].
        """
        self._shadow_samples = samples % 21
        if self._shadow_samples == 0:
            self._shadow_samples = 1

    @property
    def shadow_quality(self) -> int:
        """Shadow texture resolution (0 disables shadows).

        The value corresponds to a square cubemap face size in pixels
        (e.g. 512, 1024, 2048). 0 disables shadow mapping.
        """
        return self._shadow_quality

    @shadow_quality.setter
    def shadow_quality(self, quality: int) -> None:
        """Set shadow texture resolution.

        Parameters
        ----------
        quality : int
            One of SHADOW_NONE, SHADOW_LOW, SHADOW_MID or SHADOW_HIGH.
        """
        self._shadow_quality = quality

    def add_click_plane(
        self,
        plane_point: Vector3D,
        plane_normal: Vector3D,
        callback: Callable[[Vector3D], Any],
    ) -> None:
        """Register an infinite plane that receives click events.

        The plane is defined by a point and a normal. When the user clicks in
        the viewport and the ray intersects the plane, the provided callback
        is invoked with the 3D hit point as a list/Vector-compatible object.

        Parameters
        ----------
        plane_point : Vector3D
            Any point that lies on the plane.
        plane_normal : Vector3D
            Plane normal vector.
        callback : Callable[[Vector3D], Any]
            Function called with the world-space hit coordinates.
        """
        self._click_planes.append((plane_point, plane_normal, callback))

    def _check_click_plane(self, eye: Vector3D, vector: Vector3D) -> None:
        """Test registered click-planes against a ray and call callbacks.

        Parameters
        ----------
        eye : Vector3D
            Ray origin in world coordinates.
        vector : Vector3D
            Ray direction in world coordinates.
        """
        for click_plane in self._click_planes:
            hit = raycast_plane_intersect(eye, vector, click_plane[0], click_plane[1])
            if hit is None:
                continue
            _hit = hit.tolist()
            click_plane[2](cast(list[float], _hit[:3]))

    def _render_3d_scene(
        self, shadow_round: bool = False, shader: str = DEFAULT_SHADER
    ) -> None:
        """Render all 3D objects with a given shader/pass.

        The method sets up shader uniforms, binds shadow maps when available
        and iterates scene objects calling their :py:meth:`Object.render`.

        Parameters
        ----------
        shadow_round : bool, optional
            If True the render is for creating the shadow map and should
            skip non-shadow-casting objects. Default is False.
        shader : str, optional
            Key of the shader to use from :pyattr:`self.shaders`.
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
            # Always set shadow_enabled to ensure shader knows the state
            _shader.set_int("shadow_enabled", 1 if self.shadow_quality > 0 else 0)
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
            if object.shader == shader or (
                shadow_round and object.shader != PARTICLE_SHADER
            ):
                object.render(lit, _shader)

    def _render(self) -> None:
        """Main render routine invoked every frame.

        Performs shadow-map pass (if enabled), renders background, 3D scene,
        HUD elements and particle passes. Also updates per-frame FPS counters
        and runs collision checks.
        """
        self.shaders["default"].use()
        self._render_lock = True
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        if self.shadow_quality > 0:
            glViewport(0, 0, self._shadow_quality, self._shadow_quality)
            default_id = glGetIntegerv(GL_FRAMEBUFFER_BINDING)
            glBindFramebuffer(GL_FRAMEBUFFER, self.depth_map_fbo)
            glClear(GL_DEPTH_BUFFER_BIT)
            self.shaders[SHADOW_SHADER].use()
            self._render_3d_scene(True, SHADOW_SHADER)
            glBindFramebuffer(GL_FRAMEBUFFER, default_id)
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
        """Register a named :class:`CollisionTest`.

        Parameters
        ----------
        name : str
            Identifier for the collision tester.
        tester : CollisionTest
            Instance implementing collision checks.
        """
        if not isinstance(tester, CollisionTest):
            logging.error("tester must be an instance of CollisionTest")
            return

        self.collisions[name] = tester

    def add_object(self, name: str, obj: Object) -> bool:
        """Add a named object or HUD to the scene.

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
        """Append a Camera to the scene.

        Returns True on success, False if the provided object is not a
        :class:`Camera` instance.
        """
        if not isinstance(camera, Camera):
            logging.error("Camera is not an instance of `scene.camera`")
            return False

        self.cameras.append(camera)
        return True

    def create_camera(self) -> None:
        """Create and append a default camera sized to the current window."""
        self.cameras.append(
            Camera(viewport_size=[self.window_width, self.window_height, 0])
        )

    def create_clock(
        self,
        name: str,
        period: float,
        callback: Callable[[float, float], None],
        non_stop: bool = False,
    ) -> None:
        """Create a named periodic Clock.

        The clock will invoke ``callback(period, total)`` every ``period``
        seconds. Clocks run in their own thread so callbacks should be fast
        and thread-safe.

        Parameters
        ----------
        name : str
            Unique name for the clock.
        period : float
            Tick interval in seconds.
        callback : Callable[[float, float], None]
            Function called with (period, total_elapsed_seconds).
        non_stop : bool, optional
            If True the clock is never paused automatically.
        """
        if name in self.clocks:
            logging.error(f"A clock named {name} already exists")
            return

        c = Clock(period, callback, non_stop)
        self.clocks[name] = c

    def _clear_context(self) -> None:
        """Reset GL-related runtime state without destroying the Scene.

        This method resets shader program IDs and recreates clock objects so
        that new threads may be started after a context loss. It also
        destroys GL-owned resources such as the grid and HUD VAOs.
        """
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
        """Serialize scene content to a nested dictionary.

        The returned structure contains serializable representations of
        objects, lights and cameras. Avoid calling this from the render
        thread since it can be expensive.
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
        """Cast a ray and return the nearest intersected object and point.

        The function first performs a fast axis-aligned bounding box test and
        optionally falls back to per-triangle intersection tests. It returns
        a tuple ``(Object, hit_point)`` where ``hit_point`` is a 3-element
        sequence in world coordinates, or ``None`` if nothing was hit.

        Parameters
        ----------
        start : Vector3D
            Ray origin in world coordinates.
        direction : Vector3D
            Ray direction (does not need to be normalized).
        box_only : bool, optional
            If True only test bounding boxes and return the box hit point.
            If False, perform triangle-level testing for mesh objects.
        exempt_objects : list[Object] or None, optional
            Objects to skip during intersection testing.
        """
        shortest = [0.0, 0.0, 0.0]
        dist_best = -1.0
        hit_obj = None
        for obj in self.objects.values():
            if exempt_objects is not None and obj in exempt_objects:
                continue
            box_hit = raycast_box_intersect(
                start, direction, obj.bounding_box[0], obj.bounding_box[1]
            )
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
                ip = cast(list[float], ip_np.tolist())
                dist = distance_native(ip, start)
                if dist_best == -1.0 or dist < dist_best:
                    dist_best = dist
                    shortest = ip
                    hit_obj = obj
        if hit_obj is None:
            return None
        return hit_obj, shortest

    def _init_runtime(self, start_clocks: bool = False) -> bool:
        """Initialize OpenGL resources and compile shaders.

        This performs a minimal runtime check (OpenGL version), adjusts
        camera/hud aspect ratios to the current window size, starts clocks
        and builds shader programs and the depth cubemap used for shadows.

        Returns True on success.
        """
        version = glGetString(GL_VERSION).decode("utf-8")
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
            sys.exit(1)

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

        default_id = glGetIntegerv(GL_FRAMEBUFFER_BINDING)
        glBindFramebuffer(GL_FRAMEBUFFER, self.depth_map_fbo)
        glFramebufferTexture(
            GL_FRAMEBUFFER,
            GL_DEPTH_ATTACHMENT,
            self.depth_map,
            0,
        )
        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)
        glBindFramebuffer(GL_FRAMEBUFFER, default_id)
        for shader in self.shaders.values():
            shader.build()

        return True

    def run(self, start_clocks: bool = False) -> int:
        """Create an SDL window, initialize the GL context and run the main loop.

        Parameters
        ----------
        start_clocks : bool, optional
            If True, clocks are unpaused at startup.

        Returns
        -------
        int
            Process return code: 0 on normal termination, -1 on SDL/window
            initialization failure.
        """
        if sdl2.SDL_Init(sdl2.SDL_INIT_VIDEO) != 0:
            return -1

        sdl2.SDL_GL_SetAttribute(
            sdl2.SDL_GL_CONTEXT_PROFILE_MASK, sdl2.SDL_GL_CONTEXT_PROFILE_CORE
        )
        multisample_buffers = os.getenv("GL_MULTISAMPLEBUFFERS", None)
        multisample_samples = os.getenv("GL_MULTISAMPLESAMPLES", None)
        if multisample_buffers:
            sdl2.SDL_GL_SetAttribute(
                sdl2.SDL_GL_MULTISAMPLEBUFFERS, int(multisample_buffers)
            )
        if multisample_samples:
            sdl2.SDL_GL_SetAttribute(
                sdl2.SDL_GL_MULTISAMPLESAMPLES, int(multisample_samples)
            )

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

        self._init_runtime()
        if start_clocks:
            for clock in self.clocks:
                self.clocks[clock]._pause = False

        while self.running:
            while sdl2.SDL_PollEvent(ctypes.byref(self.event)) != 0:
                if (
                    self.event.type == sdl2.SDL_WINDOWEVENT
                    and self.event.window.event == sdl2.SDL_WINDOWEVENT_RESIZED
                ):
                    self.window_width = self.event.window.data1
                    self.window_height = self.event.window.data2
                    glViewport(0, 0, self.window_width, self.window_height)

                    for ob in self.cameras:
                        ob.aspect_ratio = self.window_width / self.window_height
                        ob._viewport_size = [self.window_width, self.window_height, 0]
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
        """Stop the render loop and terminate running clocks.

        This will signal the scene to stop and attempt to gracefully join
        existing clock threads.
        """
        self.running = False
        for clock in self.clocks:
            logging.debug(f"Kill clock [{clock}]")
            self.clocks[clock].kill()
            self.clocks[clock].join()


class Background:
    """Simple fullscreen gradient background used by the renderer.

    The shader and approach were inspired by the gradient demo from
    http://www.cs.princeton.edu/~mhalber/blog/ogl_gradient/.
    """

    def __init__(
        self,
        top_color: Optional[Vector3D] = None,
        bottom_color: Optional[Vector3D] = None,
        **kwargs: Dict[str, Any],
    ):
        """Create a Background with optional top and bottom colors.

        Parameters
        ----------
        top_color : Vector3D or None
            RGBA color for the top of the gradient. If None a default is used.
        bottom_color : Vector3D or None
            RGBA color for the bottom of the gradient. If None a default is used.
        """
        self.top_color = [0.0, 0.0, 0.0, 1.0] if top_color is None else top_color
        self.bottom_color = (
            [0.0, 0.1, 0.2, 1.0] if bottom_color is None else bottom_color
        )
        variables = ["top_color", "bot_color"]
        self._shader = Shader(
            fragment=background_fragment_shader,
            vertex=background_vertex_shader,
            variables=variables,
        )
        self._vao = None
        self.visible = True

    def set_time(self, hour: int, minute: int) -> None:
        """Adjust the background gradient to approximate a time-of-day color.

        The method maps the provided (hour, minute) to a precomputed
        color band and interpolates the top color accordingly.

        Parameters
        ----------
        hour : int
            Hour in 24-hour format. Values are wrapped into [0, 23].
        minute : int
            Minute in [0, 59]. Values are wrapped into [0, 59].
        """
        hour %= 24
        minute %= 60
        color_scheme = {
            (0, 3): [
                [24 / 255, 24 / 255, 48 / 255, 1.0],
                [24 / 255, 48 / 255, 72 / 255, 1.0],
            ],
            (3, 9): [
                [24 / 255, 48 / 255, 72 / 255, 1.0],
                [96 / 255, 168 / 255, 192 / 255, 1.0],
            ],
            (9, 15): [
                [96 / 255, 168 / 255, 192 / 255, 1.0],
                [144 / 255, 192 / 255, 240 / 255, 1.0],
            ],
            (15, 21): [
                [144 / 255, 192 / 255, 240 / 255, 1.0],
                [48 / 255, 72 / 255, 120 / 255, 1.0],
            ],
            (21, 24): [
                [48 / 255, 72 / 255, 120 / 255, 1.0],
                [24 / 255, 24 / 255, 48 / 255, 1.0],
            ],
        }
        for color in color_scheme:
            if hour >= color[0] and hour < color[1]:
                hours = color[1] - color[0]
                minutes = hours * 60
                minute = ((hour - color[0]) * 60) + minute
                xdist = (
                    (color_scheme[color][1][0] - color_scheme[color][0][0]) / minutes
                ) * minute
                ydist = (
                    (color_scheme[color][1][1] - color_scheme[color][0][1]) / minutes
                ) * minute
                zdist = (
                    (color_scheme[color][1][2] - color_scheme[color][0][2]) / minutes
                ) * minute

                self.bottom_color = color_scheme[color][0]
                self.top_color = [
                    color_scheme[color][0][0] + xdist,
                    color_scheme[color][0][1] + ydist,
                    color_scheme[color][0][2] + zdist,
                    1.0,
                ]
                return

    def render(self) -> None:
        """Render a full-screen gradient quad using the background shader.

        The method lazily builds the VAO and shader on first invocation. If
        the background is hidden the call is a no-op.
        """
        if not self.visible:
            return

        if not self._vao:
            self._vao = glGenVertexArrays(1)
            glBindVertexArray(self._vao)
            self._shader.build()
            glBindVertexArray(0)

        glDisable(GL_DEPTH_TEST)

        self._shader.use()
        self._shader.set_vector4_np(
            "top_color", np.array(self.top_color, dtype=np.float32)
        )
        self._shader.set_vector4_np(
            "bot_color", np.array(self.bottom_color, dtype=np.float32)
        )
        glBindVertexArray(self._vao)
        glDrawArrays(GL_TRIANGLES, 0, 3)
        glBindVertexArray(0)
        self._shader.end()
        glEnable(GL_DEPTH_TEST)
