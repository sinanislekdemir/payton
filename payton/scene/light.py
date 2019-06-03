"""Payton Lights.

Support for scene lights.
"""

import numpy as np


class Light(object):
    """Main Light object
    """

    def __init__(self, **args):
        """Initialize Payton Light

        Args:
          position: Position of the light in global coordinates
          default position is [10, 7, 6]
          color: Color of the light source.
        """
        self._position = args.get("position", [10.0, 7.0, 6.0])
        self._color = args.get("color", [1.0, 1.0, 1.0])
        self._position_np = np.array(self._position, dtype=np.float32)
        self._color_np = np.array(self._color, dtype=np.float32)

        self.active = True

    @property
    def position(self):
        return self._position

    @position.setter
    def position(self, position):
        self._position = position
        self._position_np = np.array(self.position, dtype=np.float32)

    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, color):
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)
