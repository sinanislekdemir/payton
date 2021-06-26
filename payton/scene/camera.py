"""Camera module
"""

import logging
import math
from typing import Any, Dict, Optional, Tuple

import numpy as np
import pyrr

from payton.math.functions import sub_vector
from payton.math.geometry import raycast_plane_intersect
from payton.math.vector import Vector3D
from payton.scene.geometry.base import Object

BUTTON_LEFT = 1
BUTTON_RIGHT = 2
BUTTON_MIDDLE = 3


class Camera:
    def __init__(
        self,
        position: Optional[Vector3D] = None,
        target: Optional[Vector3D] = None,
        up: Optional[Vector3D] = None,
        target_object: Optional[Object] = None,
        fov: float = 45.0,
        aspect_ratio: float = 1.33333,
        near: float = 0.1,
        far: float = 100.0,
        zoom: float = 10.0,
        active: bool = False,
        perspective: bool = True,
        **kwargs: Any,
    ) -> None:
        """Initialize the camera

        Keyword arguments:
        position -- Position of the camera in space
        target -- Where does the camera point?
        up -- Up direction of the camera (roll)
        target_object -- Camera will follow the target object if defined
        fov -- Field of view in degrees
        aspect_ratio -- Aspect ratio of the camera
        near -- Nearest distance that the camera can see
        far -- Furthest distance that the camera can see (effects performance)
        zoom -- Zoom ratio
        active -- Is this the active camera in the scene?
        perspective -- Perspective camera if True, otherwise Orthographic camera (default True)
        """
        self.position = [10.0, 10.0, 5.0] if position is None else position
        self.target = [0.0, 0.0, 0.0] if target is None else target
        self.up = [0.0, 0.0, 1.0] if up is None else up

        self.target_object: Optional[Object] = target_object
        self.fov: float = fov
        self.aspect_ratio: float = aspect_ratio
        self._near: float = near
        self._far: float = far

        # zoom factor for Orthographic projection
        self._zoom: float = zoom
        self.active: bool = active
        self._perspective: bool = perspective
        self._use_cache = False

        # Store matrices for future reference.
        self._projection: Optional[np.ndarray] = None
        self._view: Optional[np.ndarray] = None
        self._prev_intersection: Optional[np.ndarray] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert the camera to a dictionary"""
        return {
            "position": self.position,
            "target": self.target,
            "up": self.up,
            "fov": self.fov,
            "aspect_ratio": self.aspect_ratio,
            "zoom": self._zoom,
            "near": self._near,
            "far": self._far,
            "perspective": self.perspective,
        }

    def distance(self) -> float:
        """Calculate distance to target and return as a float"""
        xdiff = self.position[0] - self.target[0]
        ydiff = self.position[1] - self.target[1]
        zdiff = self.position[2] - self.target[2]
        res = math.sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff)
        if res == 0:
            res = 0.001
        return res

    @property
    def perspective(self) -> bool:
        """Is this a perspective projected camera"""
        return self._perspective

    @perspective.setter
    def perspective(self, perspective: bool) -> None:
        """Set perspective

        Keyword arguments:
        perspective -- Boolean (True for perspective, False for orthographic)
        """
        self._perspective = perspective
        self._use_cache = False

    @property
    def zoom(self) -> float:
        """Return the zoom factor as a float"""
        return self._zoom

    @zoom.setter
    def zoom(self, zoom_ratio: float) -> None:
        """Set the zoom factor

        Keyword arguments:
        zoom_ratio -- Zoom ratio to be set
        """
        self._zoom = zoom_ratio
        self._use_cache = False

    @property
    def near(self) -> float:
        """Return the near plane distance"""
        return self._near

    @near.setter
    def near(self, distance: float) -> None:
        """Set the near plane distance

        Keyword arguments:
        distance -- Distance in units
        """
        self._near = distance
        self._use_cache = False

    @property
    def far(self) -> float:
        """Return the far plane distance"""
        return self._far

    @far.setter
    def far(self, distance: float) -> None:
        """Set the far plane distance

        Keyword arguments:
        distance -- Distance in units
        """
        self._far = distance
        self._use_cache = False

    def rotate_around_target(self, phi: float, theta: float) -> None:
        """Rotate the camera around the target

        NOTE: This method works with Spherical Coordinates!
        https://en.wikipedia.org/wiki/Spherical_coordinate_system

        Keyword arguments:
        phi -- Phi angle in radians
        theta -- Theta angle in radians
        """

        diff = sub_vector(self.position, self.target)

        r = self.distance()
        _theta = math.acos(diff[2] / r)
        _phi = math.atan2(diff[1], diff[0])

        _ct = _theta + math.radians(theta)
        _cp = _phi + math.radians(phi)
        _ct = max(_ct, 0.001)
        _ct = min(_ct, 3.13)

        x = r * math.sin(_ct) * math.cos(_cp)
        y = r * math.sin(_ct) * math.sin(_cp)
        z = r * math.cos(_ct)

        self.position = [
            x + self.target[0],
            y + self.target[1],
            z + self.target[2],
        ]
        self._use_cache = False

    def pan(self, x: int, y: int, w: int, h: int) -> None:
        """Pan the camera to sideways

        Keyword arguments:
        x -- X displacement
        y -- Y displacement
        w -- Width of the viewport
        h -- Height of the viewport
        """
        if self.target_object is not None:
            logging.error("Camera has an active target, can not pan")
            return
        px = math.ceil(w / 2)
        py = math.ceil(h / 2)
        _ceye, center_vector = self.screen_to_world(px, py, w, h)
        eye, vector = self.screen_to_world(x, y, w, h)
        starget = self.target
        hit = raycast_plane_intersect(eye, vector, starget, center_vector)
        if hit is None:
            return
        if self._prev_intersection is None:
            self._prev_intersection = hit
        else:
            diff = np.subtract(self._prev_intersection, hit)
            self.position = [
                self.position[0] + diff[0],
                self.position[1] + diff[1],
                self.position[2] + diff[2],
            ]
            self.target = [
                self.target[0] + diff[0],
                self.target[1] + diff[1],
                self.target[2] + diff[2],
            ]
        self._use_cache = False

    def mouse_move(
        self,
        button: int,
        shift: bool,
        ctrl: bool,
        x: int,
        y: int,
        xrel: int,
        yrel: int,
        w: int,
        h: int,
    ) -> None:
        """Mouse move event handler for rotation and panning

        Keyword arguments:
        button -- Pressed button
        shift -- If the shift key is down
        ctrl -- If the ctrl key is down
        x -- x coordinate of the cursor
        y -- y coordinate of the cursor
        xrel -- x displacement of the cursor
        yrel -- y displacement of the cursor
        w -- Viewport width
        h -- Viewport height
        """
        if button == BUTTON_RIGHT:
            self.rotate_around_target(-xrel, -yrel)

        if button == BUTTON_MIDDLE:
            self.pan(x, y, w, h)

    def mouse_wheel(self, yrel: int) -> None:
        """Mouse wheel event handler for zoom

        Keyword arguments:
        yrel -- y displacement of the cursor
        """
        if self.perspective:
            self.distance_to_target(self.distance() + yrel)
        else:
            self.distance_to_target(self.zoom + yrel)

    def distance_to_target(self, distance: float) -> None:
        """Set the distance to the target

        NOTE: This refers to the radius in spherical coordinates

        Keyword arguments:
        distance -- Distance to target
        """
        if not self.perspective:
            self.zoom = distance
            return
        diff = sub_vector(self.position, self.target)
        _theta = math.acos(diff[2] / self.distance())
        _phi = math.atan2(diff[1], diff[0])

        self.position = [
            self.target[0] + distance * math.sin(_theta) * math.cos(_phi),
            self.target[1] + distance * math.sin(_theta) * math.sin(_phi),
            self.target[2] + distance * math.cos(_theta),
        ]
        self._use_cache = False

    def render(self) -> Tuple[np.ndarray, np.ndarray]:
        """Render the camera"""
        if self._use_cache and self.target_object is None and self._projection is not None and self._view is not None:
            return self._projection, self._view

        if self.perspective:
            proj_matrix = pyrr.matrix44.create_perspective_projection(
                self.fov,
                self.aspect_ratio,
                self.near,
                self.far,
                dtype=np.float32,
            )
        else:
            x = 70 * (self.aspect_ratio)
            y = 70
            if self.zoom == 0:
                self.zoom = 0.01
            self.near = max(self.near, 1)
            proj_matrix = pyrr.matrix44.create_orthogonal_projection_matrix(
                left=(-x / self.zoom),
                right=(x / self.zoom),
                top=(y / self.zoom),
                bottom=(-y / self.zoom),
                near=self.near,
                far=self.far,
                dtype=np.float32,
            )

        self._projection = proj_matrix
        eye = np.array(list(self.position), dtype=np.float32)

        if self.target_object:
            # I believe there is a bug at mypy about @property methods
            self.target = self.target_object.position

        target = np.array(list(self.target), dtype=np.float32)
        up = np.array(list(self.up), dtype=np.float32)
        view_matrix = pyrr.matrix44.create_look_at(eye[:3], target[:3], up[:3])
        self._view = view_matrix
        self._use_cache = True
        return proj_matrix, view_matrix

    def screen_to_world(self, x: int, y: int, width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
        """Turn the screen coordinates into world coordinates

        Imagine a point on the surface of the camera (your cursor),
        this method turns X-Y coordinates into world coordinates

        Keyword arguments:
        x -- x coordinate of the cursor
        y -- y coordinate of the cursor
        width -- Width of the viewport
        height -- Height of the viewport
        """
        x_f = (2.0 * x) / width - 1.0
        y_f = 1.0 - (2.0 * y) / height

        eye = np.array(
            [self.position[0], self.position[1], self.position[2], 1.0],
            dtype=np.float32,
        )

        ray_start = np.array([x_f, y_f, 1.0, 1.0], dtype=np.float32)
        proj = self._projection
        inv_proj = pyrr.matrix44.inverse(proj)
        eye_coords_ray = pyrr.matrix44.apply_to_vector(inv_proj, ray_start)
        eye_coords = np.array([eye_coords_ray[0], eye_coords_ray[1], -1.0, 0.0], dtype=np.float32)
        view = self._view
        inv_view = pyrr.matrix44.inverse(view)
        ray_end = pyrr.matrix44.apply_to_vector(inv_view, eye_coords)

        if not self.perspective:
            ray_start = np.array([x_f, y_f, 1.0, 1.0], dtype=np.float32)
            eye_coords_ray = pyrr.matrix44.apply_to_vector(inv_proj, ray_start)
            eye_coords = np.array(
                [eye_coords_ray[0], eye_coords_ray[1], 0.0, 0.0],
                dtype=np.float32,
            )
            eye = pyrr.matrix44.apply_to_vector(inv_view, eye_coords)
            ray = ray_end - eye
            ray_dir = pyrr.vector.normalize(ray[0:4])
            eye[0] -= ray_dir[0] * 500
            eye[1] -= ray_dir[1] * 500
            eye[2] -= ray_dir[2] * 500
            eye[3] = 1

            return eye, ray_dir

        ray_dir = pyrr.vector.normalize(ray_end[0:4])
        return (eye, ray_dir)
