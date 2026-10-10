"""payton.scene.scene2d

A side-view, X-Z plane scene for 2-D games and graph drawing.

:class:`Scene2D` is a drop-in subclass of :class:`~payton.scene.scene.Scene`
that swaps the default perspective camera for a locked, orthographic
:class:`~payton.scene.camera.Camera2D`. Everything else — objects, lights,
shadows, HUD/GUI, clocks, physics and audio — is inherited unchanged.
"""

import logging
import math
from typing import Any, cast

from payton.math.functions import create_rotation_matrix
from payton.scene.camera import Camera2D
from payton.scene.geometry.base import Object
from payton.scene.scene import Scene

logger = logging.getLogger(__name__)


class Scene2D(Scene):
    """A scene rendered on the X-Z plane with a locked orthographic camera.

    The camera looks from negative Y towards +Y with +Z pointing up, making
    X the screen horizontal and Z the screen vertical. This is a good fit for
    platformer games, scene design, graph drawing and 2-D animation.

    The default camera is an orthographic :class:`Camera2D`. Panning (middle
    mouse drag) and zooming (mouse wheel) work as usual. Use :meth:`follow`
    to make the camera track an object along the X-Z plane.

    Parameters
    ----------
    width : int, optional
        Initial window width in pixels. Default is 800.
    height : int, optional
        Initial window height in pixels. Default is 600.
    depth : float, optional
        Distance of the camera from the X-Z plane (on the Y axis).
        Default is 10.0.
    zoom : float, optional
        Initial orthographic zoom ratio. Default is 10.0.
    **kwargs
        Any other keyword arguments accepted by :class:`~payton.scene.scene.Scene`
        (``on_select``, ``physics_force_continuous``, ``theme``,
        ``antialiasing``, ...).

    Example
    -------
    >>> from payton.scene import Scene2D
    >>> from payton.scene.geometry import Cube
    >>> scene = Scene2D()
    >>> player = Cube(width=1, depth=1, height=1)
    >>> scene.add_object("player", player)
    >>> scene.follow(player)
    >>> scene.run()
    """

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        depth: float = 10.0,
        zoom: float = 10.0,
        **kwargs: Any,
    ) -> None:
        super().__init__(width=width, height=height, **kwargs)

        self.cameras = [
            Camera2D(
                depth=depth,
                zoom=zoom,
                active=True,
                viewport_size=[width, height, 0],
            )
        ]
        self.active_camera = self.cameras[0]

        # Lay the ground grid on the X-Z plane so it faces the camera, and
        # push it behind the scene (larger Y is further from the camera) so it
        # acts as a backdrop instead of z-fighting with in-scene objects.
        # Rotating around X maps the grid's local X-Y plane onto the world
        # X-Z plane. Grid lines are drawn by child Line objects, so the grid
        # model matrix is passed down to them at render time.
        grid_matrix = create_rotation_matrix([1, 0, 0], math.radians(-90)).tolist()
        grid_matrix[3] = [0.0, 20.0, 0.0, 1.0]
        self.grid.matrix = grid_matrix
        self.grid.resize(self.grid._xres, self.grid._yres)

    def create_camera(self) -> None:
        """Create and append a default 2-D camera sized to the current window."""
        self.cameras.append(
            Camera2D(viewport_size=[self.window_width, self.window_height, 0])
        )

    def follow(self, obj: Object) -> None:
        """Make the active camera follow *obj* along the X-Z plane.

        The camera's X and Z are snapped to the object's X and Z each frame
        while its depth (Y) stays fixed, so the view direction remains +Y.

        Parameters
        ----------
        obj : Object
            The object the camera should track.

        Example
        -------
        >>> scene = Scene2D()
        >>> scene.follow(scene.objects["player"])
        """
        self.active_camera.target_object = obj

    def unfollow(self) -> None:
        """Stop the active camera from following its target object."""
        self.active_camera.target_object = None
        self.active_camera._previous_target_location = None

    @property
    def depth(self) -> float:
        """Signed depth (Y distance) of the active camera."""
        return cast(Camera2D, self.active_camera).depth

    @depth.setter
    def depth(self, value: float) -> None:
        """Set the depth of the active camera on the Y axis."""
        cast(Camera2D, self.active_camera).depth = value

    @property
    def zoom(self) -> float:
        """Orthographic zoom ratio of the active camera."""
        return self.active_camera.zoom

    @zoom.setter
    def zoom(self, value: float) -> None:
        """Set the orthographic zoom ratio of the active camera."""
        self.active_camera.zoom = value
