"""Observer module
"""

import logging
import math
from typing import Any, Dict, List, Optional, Tuple, Type

import numpy as np  # type: ignore
import pyrr

from payton.math.geometry import raycast_plane_intersect
from payton.math.vector import invert_vector, sub_vector
from payton.scene.geometry.base import Object

BUTTON_LEFT = 1
BUTTON_RIGHT = 2
BUTTON_MIDDLE = 3


class Observer(object):
    def __init__(
        self,
        position: Optional[List[float]] = None,
        target: Optional[List[float]] = None,
        up: Optional[List[float]] = None,
        target_object: Optional[Type[Object]] = None,
        fov: float = 45.0,
        aspect_ratio: float = 1.33333,
        near: float = 0.1,
        far: float = 100.0,
        zoom: float = 10.0,
        active: bool = False,
        perspective: bool = True,
        **kwargs: Any,
    ) -> None:
        self.position: List[float] = [10.0, 10.0, 5.0] if position is None else position
        self.target: List[float] = [0.0, 0.0, 0.0] if target is None else target
        self.up: List[float] = [0.0, 0.0, 1.0] if up is None else up

        self.target_object: Optional[Type[Object]] = target_object
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
        """
        Calculate distance to target
        """
        xdiff = self.position[0] - self.target[0]
        ydiff = self.position[1] - self.target[1]
        zdiff = self.position[2] - self.target[2]
        res = math.sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff)
        if res == 0:
            res = 0.001
        return res

    @property
    def perspective(self) -> bool:
        return self._perspective

    @perspective.setter
    def perspective(self, b: bool) -> None:
        self._perspective = b
        self._use_cache = False

    @property
    def zoom(self) -> float:
        return self._zoom

    @zoom.setter
    def zoom(self, f: float) -> None:
        self._zoom = f
        self._use_cache = False

    @property
    def near(self) -> float:
        return self._near

    @near.setter
    def near(self, val: float) -> None:
        self._near = val
        self._use_cache = False

    @property
    def far(self) -> float:
        return self._far

    @far.setter
    def far(self, val: float) -> None:
        self._far = val
        self._use_cache = False

    def rotate_around_target(self, phi: float, theta: float) -> None:
        diff = sub_vector(self.position, self.target)

        r = self.distance()
        _theta = math.acos(diff[2] / r)
        _phi = math.atan2(diff[1], diff[0])

        _theta = (math.degrees(_theta) + theta) % 360
        _phi = (math.degrees(_phi) + phi) % 360

        _theta = math.radians(_theta)
        _phi = math.radians(_phi)
        x = r * math.sin(_theta) * math.cos(_phi)
        y = r * math.sin(_theta) * math.sin(_phi)
        z = r * math.cos(_theta)

        self.position[0] = x + self.target[0]
        self.position[1] = y + self.target[1]
        self.position[2] = z + self.target[2]
        self._use_cache = False

    def pan(self, x: int, y: int, w: int, h: int) -> None:
        if self.target_object is not None:
            logging.error("Camera has an active target, can not pan")
            return
        px = math.ceil(w / 2)
        py = math.ceil(h / 2)
        _ceye, center_vector = self.screen_to_world(px, py, w, h)
        eye, vector = self.screen_to_world(x, y, w, h)
        center_vector = invert_vector(center_vector).copy() + [0.0]
        starget = self.target.copy() + [1.0]
        hit = raycast_plane_intersect(eye, vector, starget, center_vector)
        if hit is None:
            return
        if self._prev_intersection is None:
            self._prev_intersection = hit
        else:
            diff = np.subtract(self._prev_intersection, hit)
            self.position[0] += diff[0]
            self.position[1] += diff[1]
            self.position[2] += diff[2]

            self.target[0] += diff[0]
            self.target[1] += diff[1]
            self.target[2] += diff[2]
        self._use_cache = False

    def mouse_move(
        self, button: int, shift: bool, ctrl: bool, x: int, y: int, xrel: int, yrel: int, w: int, h: int,
    ) -> None:
        if button == BUTTON_RIGHT:
            self.rotate_around_target(-xrel, -yrel)

        if button == BUTTON_MIDDLE:
            self.pan(x, y, w, h)

    def mouse_wheel(self, yrel: int):
        if self.perspective:
            self.distance_to_target(self.distance() + yrel)
        else:
            self.distance_to_target(self.zoom + yrel)

    def distance_to_target(self, distance: float) -> None:
        if not self.perspective:
            self.zoom = distance
            return
        diff = sub_vector(self.position, self.target)
        _theta = math.acos(diff[2] / self.distance())
        _phi = math.atan2(diff[1], diff[0])

        self.position[0] = self.target[0] + distance * math.sin(_theta) * math.cos(_phi)
        self.position[1] = self.target[1] + distance * math.sin(_theta) * math.sin(_phi)
        self.position[2] = self.target[2] + distance * math.cos(_theta)
        self._use_cache = False

    def render(self) -> Tuple[np.ndarray, np.ndarray]:
        if self._use_cache and self.target_object is None:
            return self._projection, self._view

        if self.perspective:
            proj_matrix = pyrr.matrix44.create_perspective_projection(
                self.fov, self.aspect_ratio, self.near, self.far, dtype=np.float32,
            )
        else:
            x = 70 * (self.aspect_ratio)
            y = 70
            if self.zoom == 0:
                self.zoom = 0.01
            if self.near < 1:
                self.near = 1
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
        eye = np.array(self.position, dtype=np.float32)

        if self.target_object:
            # I believe there is a bug at mypy about @property methods
            self.target = self.target_object.position  # type: ignore

        target = np.array(self.target, dtype=np.float32)
        up = np.array(self.up, dtype=np.float32)
        view_matrix = pyrr.matrix44.create_look_at(eye, target, up)
        self._view = view_matrix
        self._use_cache = True
        return proj_matrix, view_matrix

    def screen_to_world(self, x: int, y: int, width: int, height: int) -> Tuple[np.ndarray, np.ndarray]:
        x_f = (2.0 * x) / width - 1.0
        y_f = 1.0 - (2.0 * y) / height

        eye = np.array([self.position[0], self.position[1], self.position[2], 1.0], dtype=np.float32,)

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
            eye_coords = np.array([eye_coords_ray[0], eye_coords_ray[1], 0.0, 0.0], dtype=np.float32,)
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
