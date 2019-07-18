from typing import Any
from payton.scene.geometries.mesh import Mesh


class Plane(Mesh):
    """Plane object

    This is a 2D Plane in 3D World. Has a width in X and height in Y.
    If you need to place it in another axis, try modifying its matrix.

    Example use case:

        .. include:: ../../../examples/basics/13_plane.py
    """

    def __init__(self, **args: Any) -> None:
        """Initialize plane

        Args:
          width: Width of the plane
          height: Height of the plane
        """
        super().__init__(**args)
        width = args.get("width", 1.0) * 0.5
        height = args.get("height", 1.0) * 0.5
        self._vertices = [
            [-width, -height, 0],
            [width, -height, 0],
            [width, height, 0],
            [-width, height, 0],
        ]
        self._normals = [[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
        self._texcoords = [[-1, -1], [1, -1], [1, 1], [-1, 1]]
        self._indices = [[0, 1, 2], [0, 2, 3]]
