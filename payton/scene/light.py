"""Payton Lights.

Support for scene lights.
"""

import numpy as np  # type: ignore
from typing import Any, List


class Light(object):
    """Main Light object
    """

    def __init__(self, **args: Any):
        """Initialize Payton Light

        Args:
          position: Position of the light in global coordinates
          default position is [10, 7, 6]
          color: Color of the light source.
        """
        self._position: List[float] = args.get("position", [10.0, 7.0, 6.0])
        self._color: List[float] = args.get("color", [1.0, 1.0, 1.0])
        self._position_np: np.ndarray = np.array(
            self._position, dtype=np.float32
        )
        self._color_np: np.ndarray = np.array(self._color, dtype=np.float32)

        self.active: bool = True

    @property
    def position(self) -> List[float]:
        return self._position

    @position.setter
    def position(self, position: List[float]) -> None:
        self._position = position
        self._position_np = np.array(self.position, dtype=np.float32)

    @property
    def color(self) -> List[float]:
        return self._color

    @color.setter
    def color(self, color: List[float]) -> None:
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)
