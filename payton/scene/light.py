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
        self.position = args.get("position", [10.0, 7.0, 6.0])
        self.color = args.get("color", [1.0, 1.0, 1.0])
        self._position = np.array(self.position, dtype=np.float32)
        self._color = np.array(self.color, dtype=np.float32)

        self.active = True

    def set_position(self, position):
        """Render light
        """
        self.position = position
        self._position = np.array(self.position, dtype=np.float32)

    def set_color(self, color):
        self.color = color
        self._color = np.array(self.color, dtype=np.float32)
