from itertools import product
from typing import Any, List
from payton.scene.geometry import Mesh
from payton.scene.material import SOLID


class Plane(Mesh):
    """Plane object

    This is a 2D Plane in 3D World. Has a width in X and height in Y.
    If you need to place it in another axis, try modifying its matrix.

    Example use case:

        .. include:: ../../../examples/basics/13_plane.py
    """

    def __init__(
        self, width: float = 1.0, height: float = 1.0, **args: Any
    ) -> None:
        """Initialize plane

        Args:
          width: Width of the plane
          height: Height of the plane
        """
        super().__init__(**args)
        width *= 0.5
        height *= 0.5
        self._vertices = [
            [-width, -height, 0],
            [width, -height, 0],
            [width, height, 0],
            [-width, height, 0],
        ]
        self._normals = [[0, 0, 1], [0, 0, 1], [0, 0, 1], [0, 0, 1]]
        self._texcoords = [[-1, -1], [1, -1], [1, 1], [-1, 1]]
        self._indices = [[0, 1, 2], [2, 3, 0]]
        self.material._indices = self._indices


class MatrixPlane(Mesh):
    """Grid plane is a special plane which is generated by a NxM matrix and you
    can modify the z values by `.grid` list

    This is quite handy if you want to create a planar animation/simulation

    Example use case:

        .. include:: ../../../examples/mid-level/ripple.py
    """

    def __init__(
        self,
        width: float = 1.0,
        height: float = 1.0,
        x: int = 2,
        y: int = 2,
        **args: Any,
    ) -> None:
        """Initialize MatrixPlane

        Args:
          x: Number of control points along X axis
          y: Number of control points along Y axis
          width: Total width
          height: Total height
        """
        super().__init__(**args)
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        if self.x < 2:
            self.x = 2
        if self.y < 2:
            self.y = 2
        self.grid: List[List[float]] = []
        self.populate_grid()

    def update_grid(self) -> None:
        for i, j in product(range(self.x), range(self.y)):
            self._vertices[(self.x * i) + j][2] = self.grid[i][j]
        if self.material.display == SOLID:
            self.fix_normals()
        self.refresh()

    def populate_grid(self) -> None:
        """Turn grid data into plane"""
        self.clear_triangles()

        self.grid = [[0.0] * self.y for _i in range(self.x)]

        step_x = self.width / (self.x - 1)
        step_y = self.height / (self.y - 1)
        step_u = 1.0 / (self.x - 1)
        step_v = 1.0 / (self.y - 1)

        c_x = self.width / 2.0
        c_y = self.height / 2.0
        for i, j in product(range(self.x), range(self.y)):
            self._vertices.append(
                [(i * step_x) - c_x, (j * step_y) - c_y, self.grid[i][j]]
            )
            self._texcoords.append([i * step_u, j * step_v])
            if i < self.x - 1 and j < self.y - 1:
                left = (self.x * i) + j
                right = (self.x * i) + (j + 1)
                top_left = (self.x * (i + 1)) + j
                top_right = (self.x * (i + 1)) + (j + 1)
                self._indices.append([left, right, top_right])
                self._indices.append([top_right, top_left, left])

        self.material._indices = self._indices
        self.fix_normals()
        self.refresh()