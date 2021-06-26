from typing import Any, Dict, List, Optional

import numpy as np

from payton.math.functions import cubemap_projection_matrices
from payton.math.vector import Vector3D


class Light:
    def __init__(
        self,
        position: Optional[Vector3D] = None,
        color: Optional[Vector3D] = None,
        **kwargs: Any,
    ):
        """Initialize light

        Light in Payton is a point light to all directions

        Keyword arguments:
        position -- Position of the light in space
        color -- Color of the light source
        """
        self._position = [10.0, 7.0, 6.0] if position is None else position
        self._color = [1.0, 1.0, 1.0] if color is None else color
        self._position_np: np.ndarray = np.array(list(self._position), dtype=np.float32)
        self._color_np: np.ndarray = np.array(list(self._color), dtype=np.float32)
        self._shadow_matrices: List[np.ndarray] = []
        self._shadow_far_plane = 100.0

        self.active: bool = True

    @property
    def position(self) -> Vector3D:
        """Return the position of the light"""
        return self._position

    @position.setter
    def position(self, position: Vector3D) -> None:
        """Set the position of the light

        Keyword arguments:
        position -- Position in space
        """
        self._position = position
        self._position_np = np.array(self.position, dtype=np.float32)
        self._shadow_matrices = []

    def to_dict(self) -> Dict[str, Any]:
        """Convert the light into dictionary"""
        return {
            "position": self.position,
            "color": self.color,
            "active": self.active,
        }

    @property
    def shadow_matrices(self) -> List[np.ndarray]:
        """Return the shadow casting cubemap projection matrices for the light"""
        if len(self._shadow_matrices) > 0:
            return self._shadow_matrices
        self._shadow_matrices = cubemap_projection_matrices(self.position, self._shadow_far_plane)
        return self._shadow_matrices

    @property
    def shadow_far_plane(self) -> float:
        """Return the furthest distance that the light can cast shadows"""
        return self._shadow_far_plane

    @shadow_far_plane.setter
    def shadow_far_plane(self, distance: float) -> None:
        """Set the furthest distance that the light can cast shadows

        Keyword arguments:
        val -- Distance
        """
        self._shadow_far_plane = distance
        self._shadow_matrices = []

    @property
    def color(self) -> Vector3D:
        """Return the light color"""
        return self._color

    @color.setter
    def color(self, color: Vector3D) -> None:
        """Set the light color

        Keyword arguments:
        color -- Color of the light
        """
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)
