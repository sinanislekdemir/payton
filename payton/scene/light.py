from typing import Any, Dict, List, Optional

import numpy as np  # type: ignore

from payton.math.matrix import cubemap_projection_matrices


class Light(object):
    def __init__(
        self, position: Optional[List[float]] = None, color: Optional[List[float]] = None, **kwargs: Any,
    ):
        self._position: List[float] = [10.0, 7.0, 6.0] if position is None else position
        self._color: List[float] = [1.0, 1.0, 1.0] if color is None else color
        self._position_np: np.ndarray = np.array(self._position, dtype=np.float32)
        self._color_np: np.ndarray = np.array(self._color, dtype=np.float32)
        self._shadow_matrices: List[np.ndarray] = []
        self._shadow_far_plane = 100.0

        self.active: bool = True

    @property
    def position(self) -> List[float]:
        return self._position

    @position.setter
    def position(self, position: List[float]) -> None:
        self._position = position
        self._position_np = np.array(self.position, dtype=np.float32)
        self._shadow_matrices = []

    def to_dict(self) -> Dict[str, Any]:
        return {
            "position": self.position,
            "color": self.color,
            "active": self.active,
        }

    @property
    def shadow_matrices(self):
        if len(self._shadow_matrices) > 0:
            return self._shadow_matrices
        self._shadow_matrices = cubemap_projection_matrices(self.position, self._shadow_far_plane)
        return self._shadow_matrices

    @property
    def shadow_far_plane(self):
        return self._shadow_far_plane

    @shadow_far_plane.setter
    def shadow_far_plane(self, val: float):
        self._shadow_far_plane = val
        self._shadow_matrices = []

    @property
    def color(self) -> List[float]:
        return self._color

    @color.setter
    def color(self, color: List[float]) -> None:
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)
