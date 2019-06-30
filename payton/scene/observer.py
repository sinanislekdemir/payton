"""Observer module
"""

import math
import pyrr
import numpy as np  # type: ignore

from typing import Any, List, Optional, Type, Tuple

from payton.scene.geometry import Object
from payton.math.vector import sub_vector
from payton.math.geometry import raycast_plane_intersect

BUTTON_LEFT = 1
BUTTON_RIGHT = 2


class Observer(object):
    """
    Observers are basically cameras in the scene.
    """

    def __init__(self, **args: Any) -> None:
        """
        Initialize the defaults of an Observer.
        Default observer is a perspective mode camera
        standing at x:10 y:10 z:10 looking at the origin
        and up direction is x:0 y:1 z:0

        Args:
          position: Position of the camera in absolute coordinates.
                    [10., 10., 10.]
          target: Target of the camera in absolute coordinates.
                  [0., 0., 0.]
          up: Up vector of the camera. [0., 0., 1.] Z-Up
          target_object: Instead of a fixed coordinate, track an object. (None)
          fov: Field of view of the camera [45.]
          aspect_ratio: Aspect ratio of the camera. [4/3]
          near: Near clipping plane, can't be negative. [0.1]
          far: Far plane. Further objects will be invisible. [100.]
          active: Is this the active camera in the scene? [False]
        """
        self.position: List[float] = args.get("position", [10.0, 10.0, 5.0])
        self.target: List[float] = args.get("target", [0.0, 0.0, 0.0])
        self.up: List[float] = args.get("up", [0.0, 0.0, 1.0])

        self.target_object: Optional[Type[Object]] = args.get(
            "target_object", None
        )
        self.fov: float = args.get("fov", 45.0)
        self.aspect_ratio: float = args.get("aspect_ratio", 800.0 / 600.0)
        self.near: float = args.get("near", 0.1)
        self.far: float = args.get("far", 100.0)

        # self.perspective = args.get('perspective', True)
        # zoom factor for Orthographic projection
        self.zoom: float = args.get("zoom", 10.0)
        self.active: bool = args.get("active", False)
        self.perspective: bool = args.get("perspective", True)

        # Store matrices for future reference.
        self._projection: Optional[np.ndarray] = None
        self._view: Optional[np.ndarray] = None
        self._prev_intersection: Optional[np.ndarray] = None

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

    def rotate_around_target(self, phi: float, theta: float) -> None:
        """
        Theta: horizontal
        Phi: vertical
        """
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

    def mouse(
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
        if shift and ctrl and button == BUTTON_LEFT:  # Panning
            eye, vector = self.screen_to_world(x, y, w, h)
            hit = raycast_plane_intersect(
                eye, vector, [0, 0, 0, 1], [0, 0, 1, 0]
            )

            if hit is None:
                return

            if self._prev_intersection is None:
                self._prev_intersection = hit
            else:
                diff = np.subtract(self._prev_intersection, hit)
                self.position[0] += diff[0]
                self.position[1] += diff[1]

                self.target[0] += diff[0]
                self.target[1] += diff[1]
            return

        if shift:
            if button == BUTTON_LEFT:
                self.rotate_around_target(xrel, -yrel)

        if ctrl:
            if button == BUTTON_LEFT:
                if self.perspective:
                    self.distance_to_target(self.distance() + yrel)
                else:
                    self.distance_to_target(self.zoom + yrel)

    def distance_to_target(self, distance: float) -> None:
        """Change distance to target

        This function does not change the spherical angles but just
        adjusts the distance to target coordinates
        """
        if not self.perspective:
            self.zoom = distance
            return
        diff = sub_vector(self.position, self.target)
        _theta = math.acos(diff[2] / self.distance())
        _phi = math.atan2(diff[1], diff[0])

        self.position[0] = self.target[0] + distance * math.sin(
            _theta
        ) * math.cos(_phi)
        self.position[1] = self.target[1] + distance * math.sin(
            _theta
        ) * math.sin(_phi)
        self.position[2] = self.target[2] + distance * math.cos(_theta)

    def render(self) -> Tuple[np.ndarray, np.ndarray]:
        """Render camera into two matrices.

        Return:
          (projection_matrix, view_matrix)
        """
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
        return proj_matrix, view_matrix

    def screen_to_world(
        self, x: int, y: int, width: int, height: int
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Convert screen coordinates to world coordinates

        Unlike gluUnproject, this method returns a tuple of ray start
        (eye position) and ray vector (look at direction)

        Args:
          x: X Coordinate
          y: Y Coordinate
          width: Screen width
          height: Screen height

        Returns:
          (ray_start, ray_vector): Numpy arrays
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
        eye_coords = np.array(
            [eye_coords_ray[0], eye_coords_ray[1], -1.0, 0.0], dtype=np.float32
        )
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
