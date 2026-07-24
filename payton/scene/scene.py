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
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Optional, TypeVar, cast

import numpy as np
import pyrr
import sdl2
from OpenGL.GL import (
    GL_COLOR_BUFFER_BIT,
    GL_DEPTH_ATTACHMENT,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_FRAMEBUFFER,
    GL_FRAMEBUFFER_BINDING,
    GL_LESS,
    GL_MAJOR_VERSION,
    GL_MINOR_VERSION,
    GL_MULTISAMPLE,
    GL_PACK_ALIGNMENT,
    GL_RGBA,
    GL_TEXTURE0,
    GL_TEXTURE_CUBE_MAP,
    GL_TRIANGLES,
    GL_UNSIGNED_BYTE,
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
    glEnable,
    glFramebufferTexture2D,
    glGenVertexArrays,
    glGetIntegerv,
    glGetString,
    glPixelStorei,
    glReadPixels,
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
from payton.scene.gui.window import Theme
from payton.scene.light import Light
from payton.scene.physics import physics_client
from payton.scene.receiver import Receiver
from payton.scene.shader import (
    DEFAULT_SHADER,
    PARTICLE_SHADER,
    SHADOW_CUBE,
    Shader,
    background_fragment_shader,
    background_vertex_shader,
    default_fragment_shader,
    default_vertex_shader,
    particle_fragment_shader,
    particle_geometry_shader,
    particle_vertex_shader,
    shadow_cube_fragment_shader,
    shadow_cube_vertex_shader,
)
from payton.scene.theme import THEME_BLENDER, THEME_GAMEENGINE, THEME_STUDIO, SceneTheme
from payton.scene.types import CPlane

try:
    import pybullet
except ModuleNotFoundError:
    pass

logger = logging.getLogger(__name__)

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
        on_select: Callable | None = None,
        physics_force_continuous: bool = False,
        theme: SceneTheme | None = None,
        antialiasing: int | None = None,
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
        theme : SceneTheme or None, optional
            Visual theme for the scene.  Pass one of the built-in presets
            (``THEME_BLENDER``, ``THEME_STUDIO``, ``THEME_GAMEENGINE``) or a
            custom :class:`~payton.scene.theme.SceneTheme` instance.
            Defaults to :data:`~payton.scene.theme.THEME_STUDIO`.
        antialiasing : int or None, optional
            Multisampling antialiasing (MSAA) level.  ``None`` (default) lets
            Payton request the best level the driver supports automatically.
            ``0`` disables MSAA entirely.  Explicit values (``2``, ``4``,
            ``8``, ``16``) request a specific sample count.  Environment
            variables ``GL_MULTISAMPLEBUFFERS`` / ``GL_MULTISAMPLESAMPLES``
            take precedence over this parameter when both are set.
        **kwargs
            Ignored; kept for backward compatibility.
        """
        self.__fps_counter = 0
        self.fps = 0
        self._antialiasing = antialiasing
        self._active_msaa_samples: int = 0
        _env_width = os.getenv("SDL_WINDOW_WIDTH", None)
        _env_height = os.getenv("SDL_WINDOW_HEIGHT", None)
        if _env_width:
            width = int(_env_width)
        if _env_height:
            height = int(_env_height)

        # All objects list
        self.objects: dict[str, Object] = {}
        # All Huds (Heads Up Display)
        self.huds: dict[str, Hud] = {"_help": Hud(width=width, height=height)}
        self.huds["_help"].add_child("help", help_win())
        self.huds["_help"].hide()

        self.__timer = -1.0
        self.cameras: list[Camera] = []
        self.cameras.append(Camera(active=True, viewport_size=[width, height, 0]))
        self._physics_params = PhysicsParams(0, 0, -10)
        self.clocks: dict[str, Clock] = {}

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

        _theme: SceneTheme = theme if theme is not None else THEME_STUDIO

        self.ui_theme: Theme = Theme.from_scene_theme(_theme)
        """UI Theme derived from the scene's SceneTheme.

        Pass this to :class:`~payton.scene.gui.window.Window` or other UI
        elements to get a palette that is coherent with the 3-D scene::

            scene = Scene(theme=THEME_STUDIO)
            win   = Window("My Window", theme=scene.ui_theme)
        """

        self.lights: list[Light] = []
        self.lights.append(
            Light(
                position=list(_theme.light_position),
                color=list(_theme.light_color),
            )
        )

        self.grid = Grid(
            color=list(_theme.grid_color),
            major_color=(
                list(_theme.grid_major_color)
                if _theme.grid_major_color is not None
                else None
            ),
            major_interval=_theme.grid_major_interval,
        )
        self.controller = Controller()
        self.controller.add_controller(GUIController())
        self.controller.add_controller(SceneController())
        self.background = Background(theme=_theme)
        self.shaders: dict[str, Shader] = {
            DEFAULT_SHADER: Shader(
                fragment=default_fragment_shader, vertex=default_vertex_shader
            ),
            PARTICLE_SHADER: Shader(
                fragment=particle_fragment_shader,
                vertex=particle_vertex_shader,
                geometry=particle_geometry_shader,
            ),
            SHADOW_CUBE: Shader(
                fragment=shadow_cube_fragment_shader,
                vertex=shadow_cube_vertex_shader,
                variables=["model", "shadowView", "shadowProj", "lightPos", "shadowFar"],
            ),
        }
        self.shaders[SHADOW_CUBE]._depth_pass = True

        # SDL Related Stuff

        self.window = None
        self.window_width = width
        self.window_height = height
        self._context = None
        self._mouse = [0, 0]
        self._shift_down = False
        self._ctrl_down = False
        self._rotate = False
        self.collisions: dict[str, CollisionTest] = {}
        self._click_planes: list[CPlane] = []

        self.on_select = on_select
        self._screenshot_requested: str | None = None
        # Main running state
        self.running = False
        self._objects_lock = threading.Lock()
        self._pending_destroy: list[Object] = []
        self._shadow_quality = SHADOW_MID
        self._shadow_samples = 20

        # Fog settings (1 unit == 1 metre)
        self.fog_enabled: bool = False
        """Whether fog is applied to the 3-D scene.  Use :meth:`enable_fog`
        and :meth:`disable_fog` as a convenient shorthand."""
        self.fog_mode: int = 0
        """Fog attenuation mode.
        ``0`` – linear (uses :attr:`fog_near` / :attr:`fog_far`),
        ``1`` – exponential (uses :attr:`fog_density`),
        ``2`` – exponential-squared (smoother, uses :attr:`fog_density`)."""
        self.fog_color: list[float] = [0.7, 0.7, 0.75]
        """RGB fog colour (neutral grey-blue).  Should match or complement
        the background colour for a natural look."""
        self.fog_near: float = 20.0
        """Distance in metres at which linear fog begins (``fog_mode == 0``)."""
        self.fog_far: float = 100.0
        """Distance in metres at which linear fog reaches full opacity
        (``fog_mode == 0``)."""
        self.fog_density: float = 0.02
        """Density coefficient for exponential fog modes
        (``fog_mode`` 1 or 2).  Smaller values = thinner haze."""

    # ------------------------------------------------------------------
    # Theme helpers
    # ------------------------------------------------------------------

    def apply_theme(self, theme: SceneTheme) -> None:
        """Apply a :class:`~payton.scene.theme.SceneTheme` at any time.

        Updates the background colours and mode, the grid colours, and the
        first light's position and colour instantly — no need to recreate the
        scene.

        Parameters
        ----------
        theme : SceneTheme
            Any :class:`SceneTheme` instance, including the built-in presets.

        Example
        -------
        >>> from payton.scene import Scene, THEME_STUDIO
        >>> scene = Scene()
        >>> scene.apply_theme(THEME_STUDIO)
        """
        self.background.top_color = list(theme.background_top_color)
        self.background.bottom_color = list(theme.background_bottom_color)
        self.background._background_mode = theme.background_mode

        self.grid._color = list(theme.grid_color)
        self.grid._material.color = list(theme.grid_color)
        self.grid._major_color = (
            list(theme.grid_major_color) if theme.grid_major_color is not None else None
        )
        self.grid._major_interval = theme.grid_major_interval
        # Rebuild grid geometry to reflect new colours
        self.grid.resize(self.grid._xres, self.grid._yres)

        if self.lights:
            self.lights[0].position = list(theme.light_position)
            self.lights[0].color = list(theme.light_color)

    def theme_blender(self) -> None:
        """Switch to the Blender-style neutral mid-gray viewport theme.

        Example
        -------
        >>> scene = Scene()
        >>> scene.theme_blender()
        """
        self.apply_theme(THEME_BLENDER)

    def theme_studio(self) -> None:
        """Switch to the Studio theme: warm charcoal background with vignette.

        Example
        -------
        >>> scene = Scene()
        >>> scene.theme_studio()
        """
        self.apply_theme(THEME_STUDIO)

    def theme_gameengine(self) -> None:
        """Switch to the Game-engine theme: sky-blue horizon with glow.

        Example
        -------
        >>> scene = Scene()
        >>> scene.theme_gameengine()
        """
        self.apply_theme(THEME_GAMEENGINE)

    # ------------------------------------------------------------------
    # Fog helpers
    # ------------------------------------------------------------------

    def enable_fog(self) -> None:
        """Enable atmospheric fog rendering.

        Example
        -------
        >>> scene = Scene()
        >>> scene.enable_fog()
        >>> scene.fog_near = 10.0
        >>> scene.fog_far = 60.0
        """
        self.fog_enabled = True

    def disable_fog(self) -> None:
        """Disable atmospheric fog rendering.

        Example
        -------
        >>> scene = Scene()
        >>> scene.disable_fog()
        """
        self.fog_enabled = False

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
        with self._objects_lock:
            objects_snapshot = list(self.objects.values())
        for child in objects_snapshot:
            child._bullet_physics()

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
        self,
        shadow_round: bool = False,
        shader: str = DEFAULT_SHADER,
        objects_snapshot: list[Object] | None = None,
    ) -> None:
        """Render all 3D objects with a given shader/pass.

        The method sets up shader uniforms, binds the depth pre-pass texture
        for screen-space shadows (when not a shadow round), and iterates
        scene objects calling their :py:meth:`Object.render`.

        Parameters
        ----------
        shadow_round : bool, optional
            If True the render is for the depth pre-pass and should skip
            particles. Default is False.
        shader : str, optional
            Key of the shader to use from :pyattr:`self.shaders`.
        objects_snapshot : list, optional
            Pre-snapshot object list from _render() to avoid repeated copies.
            If None, the live dict view is used (backward-compatible fallback).
        """
        if not shadow_round:
            light_count = len(self.lights)
            lit = light_count > 0
            if not lit:
                return
        else:
            light_count = 0
            lit = False

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

        if not shadow_round:
            _shader.set_matrix4x4_np("projection", proj)
            light_array_np = np.array(
                [light.position for light in self.lights], dtype=np.float32
            )
            lcolor_array_np = np.array(
                [light.color for light in self.lights], dtype=np.float32
            )
            _shader.set_vector3_array_np("light_pos", light_array_np, light_count)
            _shader.set_int("LIGHT_COUNT", light_count)
            _shader.set_vector3_array_np("light_color", lcolor_array_np, light_count)
            _shader.set_int("shadow_enabled", 1 if self.shadow_quality > 0 else 0)
            _shader.set_int("pcfSamples", self._shadow_samples)

            has_shadows = self.shadow_quality > 0
            max_shadows = min(light_count, 16)
            for i in range(max_shadows):
                light = self.lights[i]
                _shader.set_int(f"shadow_casts[{i}]", 1 if light.cast_shadows else 0)
                _shader.set_float(f"shadowBias[{i}]", light.shadow_bias)
                _shader.set_float(f"shadowFar[{i}]", light.shadow_far)
                tex_unit = 2 + i
                if has_shadows and light.cast_shadows and light._shadow_cubemap_tex > 0:
                    glActiveTexture(GL_TEXTURE0 + tex_unit)
                    glBindTexture(GL_TEXTURE_CUBE_MAP, light._shadow_cubemap_tex)
                    _shader.set_int(f"shadowCube[{i}]", tex_unit)
                else:
                    _shader.set_int(f"shadowCube[{i}]", 0)

            # Fog uniforms (only meaningful for the default 3-D shader)
            if shader == DEFAULT_SHADER:
                _shader.set_int("fog_enabled", 1 if self.fog_enabled else 0)
                _shader.set_int("fog_mode", self.fog_mode)
                _shader.set_vector3("fog_color", self.fog_color)
                _shader.set_float("fog_near", self.fog_near)
                _shader.set_float("fog_far", self.fog_far)
                _shader.set_float("fog_density", self.fog_density)
        else:
            _shader.set_matrix4x4_np("projection", proj)

        if not shadow_round and shader == DEFAULT_SHADER:
            self.grid.render(lit, self.shaders[DEFAULT_SHADER])

        objects = (
            objects_snapshot if objects_snapshot is not None else self.objects.values()
        )
        for object in objects:
            if object.shader == shader or (
                shadow_round and object.shader != PARTICLE_SHADER
            ):
                object.render(lit, _shader)

    def _render_shadow_maps(self, objects_snapshot: list[Object]) -> None:
        """Render cubemap shadow maps for each shadow-casting light.

        For each light that has ``cast_shadows=True``, renders the scene
        geometry from the light's perspective to the six faces of the
        shadow cubemap.  Shadow maps that are not marked dirty are reused
        across frames to avoid redundant rendering.
        """
        face_size = self.shadow_quality if self.shadow_quality > 0 else 1024
        default_id = glGetIntegerv(GL_FRAMEBUFFER_BINDING)

        for light in self.lights:
            if not light.active or not light.cast_shadows:
                continue

            if light._shadow_cubemap_tex <= 0 or light._shadow_face_size != face_size:
                light.init_shadow_cubemap(face_size)

            if not light._shadow_dirty:
                continue

            proj = light.shadow_projection()
            shader = self.shaders[SHADOW_CUBE]
            shader.use()
            shader.set_matrix4x4_np("shadowProj", proj)
            shader.set_vector3_np("lightPos", light._position_np)
            shader.set_float("shadowFar", light.shadow_far)

            glViewport(0, 0, face_size, face_size)
            glBindFramebuffer(GL_FRAMEBUFFER, light._shadow_cubemap_fbo)

            glEnable(GL_DEPTH_TEST)
            glDepthFunc(GL_LESS)

            for face_target, target_vec, up_vec in light.shadow_face_matrices():
                glFramebufferTexture2D(
                    GL_FRAMEBUFFER,
                    GL_DEPTH_ATTACHMENT,
                    face_target,
                    light._shadow_cubemap_tex,
                    0,
                )
                glClear(GL_DEPTH_BUFFER_BIT)

                view_mat = pyrr.matrix44.create_look_at(
                    eye=np.array(light.position, dtype=np.float32),
                    target=np.array(
                        [
                            light.position[0] + target_vec[0],
                            light.position[1] + target_vec[1],
                            light.position[2] + target_vec[2],
                        ],
                        dtype=np.float32,
                    ),
                    up=np.array(up_vec, dtype=np.float32),
                )
                shader.set_matrix4x4_np("shadowView", view_mat)

                for obj in objects_snapshot:
                    if obj.shader != PARTICLE_SHADER and obj._visible:
                        obj.render(False, shader)

            shader.end()
            glBindFramebuffer(GL_FRAMEBUFFER, default_id)
            glViewport(0, 0, self.window_width, self.window_height)

            light._shadow_dirty = False

    def _render(self) -> None:
        """Main render routine invoked every frame.

        Performs depth pre-pass for screen-space shadows (if enabled),
        renders background, 3D scene, HUD elements and particle passes.
        Also updates per-frame FPS counters and runs collision checks.
        """
        self.shaders["default"].use()

        # Free GL resources for objects removed from clock threads
        with self._objects_lock:
            if self._pending_destroy:
                pending = list(self._pending_destroy)
                self._pending_destroy.clear()
            else:
                pending = []
        for obj in pending:
            obj.destroy()

        # Snapshot objects once under lock for all render passes
        with self._objects_lock:
            objects_snapshot = list(self.objects.values())

        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        glClearColor(0.1, 0.1, 0.1, 1.0)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Cubemap shadow map rendering
        if self.shadow_quality > 0:
            self._render_shadow_maps(objects_snapshot)

        # Render background
        glViewport(0, 0, self.window_width, self.window_height)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        self.background.render(
            self.window_width, self.window_height, self.active_camera
        )
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LESS)

        # Render 3D scene
        lit = len(self.lights) > 0
        self.shaders[DEFAULT_SHADER].use()
        self._render_3d_scene(False, DEFAULT_SHADER, objects_snapshot)

        # Render HUD
        with self._objects_lock:
            huds_snapshot = list(self.huds.items())
        for name, hud_obj in huds_snapshot:
            hud_obj.render(lit, self.shaders[DEFAULT_SHADER])
        self.shaders[DEFAULT_SHADER].end()

        self.shaders[PARTICLE_SHADER].use()
        self._render_3d_scene(False, PARTICLE_SHADER, objects_snapshot)
        self.shaders[PARTICLE_SHADER].end()

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
            logger.error("tester must be an instance of CollisionTest")
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
            logger.error("Given object is not an instance of `scene.Object`")
            return False

        if isinstance(obj, Shape2D):
            logger.error("2D Shapes can't be added directly to the scene")
            return False

        if isinstance(obj, Hud):
            """Huds must be rendered in a different loop after rendering
            all objects"""
            with self._objects_lock:
                if name in self.huds:
                    logger.error(f"Given HUD name [{name}] already exists")
                    return False
                self.huds[name] = obj
            obj.name = name
            obj.set_size(self.window_width, self.window_height)
            return True

        with self._objects_lock:
            if name in self.objects:
                logger.error(f"Given object name [{name}] already exists")
                return False
            self.objects[name] = obj
        obj.name = name
        return True

    def remove_object(self, name: str) -> Object | None:
        """Remove a named object from the scene.

        The object's OpenGL resources are freed on the next render cycle
        in the main thread, making this call safe from clock callbacks.

        Parameters
        ----------
        name : str
            Name of the object to remove.

        Returns
        -------
        Object or None
            The removed object, or None if not found.
        """
        with self._objects_lock:
            if name not in self.objects:
                return None
            obj = self.objects.pop(name)
            self._pending_destroy.append(obj)
        return obj

    def add_camera(self, camera: Camera) -> bool:
        """Append a Camera to the scene.

        Returns True on success, False if the provided object is not a
        :class:`Camera` instance.
        """
        if not isinstance(camera, Camera):
            logger.error("Camera is not an instance of `scene.camera`")
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
            logger.error(f"A clock named {name} already exists")
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

    def to_dict(self) -> dict[str, Any]:
        """Serialize scene content to a nested dictionary.

        The returned structure contains serializable representations of
        objects, lights and cameras. Avoid calling this from the render
        thread since it can be expensive.
        """
        return {
            "objects": {
                name: self.objects[name].to_dict() for name in list(self.objects)
            },
            "lights": [light.to_dict() for light in self.lights],
            "cameras": [camera.to_dict() for camera in self.cameras],
        }

    def raycast_intersect(
        self,
        start: Vector3D,
        direction: Vector3D,
        box_only: bool = True,
        exempt_objects: list[Object] | None = None,
    ) -> tuple[Object, Vector3D] | None:
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
        with self._objects_lock:
            objects_snapshot = list(self.objects.values())
        for obj in objects_snapshot:
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

        if self._active_msaa_samples > 1:
            glEnable(GL_MULTISAMPLE)

        for hud in self.huds.values():
            hud.set_size(self.window_width, self.window_height)

        for clock in self.clocks:
            self.clocks[clock].start()
            if start_clocks:
                self.clocks[clock].pause()

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
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MAJOR_VERSION, 3)
        sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_CONTEXT_MINOR_VERSION, 3)
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
        elif self._antialiasing is not None and self._antialiasing > 0:
            sdl2.SDL_GL_SetAttribute(sdl2.SDL_GL_MULTISAMPLEBUFFERS, 1)
            sdl2.SDL_GL_SetAttribute(
                sdl2.SDL_GL_MULTISAMPLESAMPLES, self._antialiasing
            )

        _window_args = (
            b"Payton Scene",
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            sdl2.SDL_WINDOWPOS_UNDEFINED,
            int(self.window_width),
            int(self.window_height),
            sdl2.SDL_WINDOW_OPENGL | sdl2.SDL_WINDOW_RESIZABLE,
        )

        if self._antialiasing is None and not multisample_samples:
            # Auto-detect: probe descending MSAA levels until one sticks.
            for samples in (16, 8, 4, 2):
                sdl2.SDL_GL_SetAttribute(
                    sdl2.SDL_GL_MULTISAMPLEBUFFERS, 1
                )
                sdl2.SDL_GL_SetAttribute(
                    sdl2.SDL_GL_MULTISAMPLESAMPLES, samples
                )
                self.window = sdl2.SDL_CreateWindow(*_window_args)
                if self.window:
                    break
                logger.info(
                    "MSAA %dx not available, trying next level", samples
                )
            else:
                sdl2.SDL_GL_SetAttribute(
                    sdl2.SDL_GL_MULTISAMPLEBUFFERS, 0
                )
                sdl2.SDL_GL_SetAttribute(
                    sdl2.SDL_GL_MULTISAMPLESAMPLES, 0
                )
                logger.info("MSAA not available, disabling antialiasing")
                self.window = sdl2.SDL_CreateWindow(*_window_args)
        else:
            self.window = sdl2.SDL_CreateWindow(*_window_args)

        if not self.window:
            return -2

        self._context = sdl2.SDL_GL_CreateContext(self.window)
        if not self._context:
            logger.error(
                f"Failed to create OpenGL context: {sdl2.SDL_GetError().decode()}"
            )
            sdl2.SDL_DestroyWindow(self.window)
            sdl2.SDL_Quit()
            return -3
        sdl2.SDL_GL_MakeCurrent(self.window, self._context)
        sdl2.SDL_GL_SetSwapInterval(0)

        # Query the actual MSAA sample count granted by the driver.
        _actual_samples = ctypes.c_int(0)
        sdl2.SDL_GL_GetAttribute(
            sdl2.SDL_GL_MULTISAMPLESAMPLES, ctypes.byref(_actual_samples)
        )
        self._active_msaa_samples = _actual_samples.value
        if self._active_msaa_samples > 1:
            logger.info(f"Antialiasing enabled: {self._active_msaa_samples}x MSAA")
        else:
            logger.info("Antialiasing disabled (0 samples granted by driver)")

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
                    with self._objects_lock:
                        huds_snapshot = list(self.huds.items())
                    for hud_name, hud_obj in huds_snapshot:
                        hud_obj.set_size(self.window_width, self.window_height)
                self.controller.keyboard(self.event, self)
                self.controller.mouse(self.event, self)

            self._render()

            if self._screenshot_requested is not None:
                self._do_screenshot(self._screenshot_requested)
                self._screenshot_requested = None

            sdl2.SDL_GL_SwapWindow(self.window)
            sdl2.SDL_Delay(1)

        for obj in list(self.objects.values()):
            obj.destroy()

        for clock in self.clocks:
            self.clocks[clock].kill()
            self.clocks[clock]._hold = True

        sdl2.SDL_GL_DeleteContext(self._context)
        sdl2.SDL_DestroyWindow(self.window)
        self.window = None
        sdl2.SDL_Quit()
        self._clear_context()
        return 0

    def screenshot(self, filename: str | None = None) -> str:
        """Request a screenshot of the current frame as a PNG file.

        The actual capture happens on the main render thread during the next
        frame. Safe to call from clock callbacks (any thread).

        Parameters
        ----------
        filename : str or None, optional
            Output filename. If None, the current Unix timestamp as an
            integer is used (e.g. ``1718123456.png``).

        Returns
        -------
        str
            The filename the screenshot will be saved to.
        """
        if filename is None:
            filename = f"{int(time.time())}.png"
        self._screenshot_requested = filename
        return filename

    def _do_screenshot(self, filename: str) -> None:
        """Perform the actual framebuffer read and PNG save on the main thread."""
        from PIL import Image

        glPixelStorei(GL_PACK_ALIGNMENT, 1)
        data = glReadPixels(
            0,
            0,
            self.window_width,
            self.window_height,
            GL_RGBA,
            GL_UNSIGNED_BYTE,
        )

        image = Image.frombytes("RGBA", (self.window_width, self.window_height), data)
        image = image.transpose(Image.Transpose.FLIP_TOP_BOTTOM)
        image.save(filename, "PNG")

    def terminate(self) -> None:
        """Stop the render loop and terminate running clocks.

        This will signal the scene to stop and attempt to gracefully join
        existing clock threads.
        """
        self.running = False
        for clock in self.clocks:
            logger.debug(f"Kill clock [{clock}]")
            self.clocks[clock].kill()
            self.clocks[clock].join()


class Background:
    """Simple fullscreen gradient background used by the renderer.

    The shader and approach were inspired by the gradient demo from
    http://www.cs.princeton.edu/~mhalber/blog/ogl_gradient/.
    """

    def __init__(
        self,
        top_color: Vector3D | None = None,
        bottom_color: Vector3D | None = None,
        theme: SceneTheme | None = None,
        **kwargs: dict[str, Any],
    ):
        """Create a Background with optional top and bottom colors.

        Parameters
        ----------
        top_color : Vector3D or None
            RGBA color for the top of the gradient.  When provided this
            overrides any value from *theme*.
        bottom_color : Vector3D or None
            RGBA color for the bottom of the gradient.  When provided this
            overrides any value from *theme*.
        theme : SceneTheme or None
            Visual theme to initialise colors and background mode from.
            Explicit *top_color* / *bottom_color* arguments take precedence.
        """
        if theme is not None:
            self.top_color = list(theme.background_top_color)
            self.bottom_color = list(theme.background_bottom_color)
            self._background_mode: int = theme.background_mode
        else:
            self.top_color = [0.0, 0.0, 0.0, 1.0]
            self.bottom_color = [0.0, 0.1, 0.2, 1.0]
            self._background_mode = 0

        # Explicit arguments override the theme
        if top_color is not None:
            self.top_color = list(top_color)
        if bottom_color is not None:
            self.bottom_color = list(bottom_color)

        variables = [
            "top_color",
            "bot_color",
            "time",
            "resolution",
            "background_mode",
            "inv_proj",
            "inv_view",
        ]
        self._shader = Shader(
            fragment=background_fragment_shader,
            vertex=background_vertex_shader,
            variables=variables,
        )
        self._vao = None
        self.visible = True
        self._start_time: float = time.time()

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

    def render(
        self, width: int = 0, height: int = 0, camera: Optional["Camera"] = None
    ) -> None:
        """Render a full-screen gradient quad using the background shader.

        The method lazily builds the VAO and shader on first invocation. If
        the background is hidden the call is a no-op.

        Parameters
        ----------
        width : int
            Current viewport width in pixels (used for the *resolution*
            shader uniform).
        height : int
            Current viewport height in pixels.
        camera : Camera or None
            Active camera. When provided and background_mode == 2 (game-engine),
            the inverse projection and view matrices are passed to the shader so
            the horizon glow follows the camera's actual look direction.
        """
        if not self.visible:
            return

        if not self._vao:
            self._vao = glGenVertexArrays(1)
            glBindVertexArray(self._vao)
            self._shader.build()
            glBindVertexArray(0)

        glDisable(GL_DEPTH_TEST)

        elapsed = time.time() - self._start_time

        self._shader.use()
        self._shader.set_vector4_np(
            "top_color", np.array(self.top_color, dtype=np.float32)
        )
        self._shader.set_vector4_np(
            "bot_color", np.array(self.bottom_color, dtype=np.float32)
        )
        self._shader.set_float("time", elapsed)
        self._shader.set_int("background_mode", self._background_mode)
        if width > 0 and height > 0:
            self._shader.set_resolution(float(width), float(height))

        if self._background_mode == 2 and camera is not None:
            proj, view = camera.render()
            if proj is not None and view is not None:
                inv_proj = np.asfortranarray(np.linalg.inv(proj), dtype=np.float32)
                inv_view = np.asfortranarray(np.linalg.inv(view), dtype=np.float32)
                self._shader.set_matrix4x4_np("inv_proj", inv_proj)
                self._shader.set_matrix4x4_np("inv_view", inv_view)

        glBindVertexArray(self._vao)
        glDrawArrays(GL_TRIANGLES, 0, 3)
        glBindVertexArray(0)
        self._shader.end()
        glEnable(GL_DEPTH_TEST)
