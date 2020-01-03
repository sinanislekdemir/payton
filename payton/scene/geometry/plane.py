from itertools import product
from typing import Any, List

from payton.scene.geometry.mesh import Mesh
from payton.scene.material import SOLID, WHITE


class Plane(Mesh):
    def __init__(self, width: float = 1.0, height: float = 1.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
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
    def __init__(self, width: float = 1.0, height: float = 1.0, x: int = 2, y: int = 2, **kwargs: Any,) -> None:
        super().__init__(**kwargs)
        self.width = width
        self.height = height
        self.x = x
        self.y = y
        if self.x < 2:
            self.x = 2
        if self.y < 2:
            self.y = 2
        self.grid: List[List[float]] = []
        # List of List for X, Y and List[float] for color.
        self.color_grid: List[List[List[float]]] = []
        self.populate_grid()

    def update_grid(self) -> None:
        # TODO This method needs some optimization
        for i, j in product(range(self.x), range(self.y)):
            self._vertices[(self.x * i) + j][2] = self.grid[i][j]
            self._vertex_colors[(self.x * i) + j] = self.color_grid[i][j]
        if self.material.display == SOLID:
            self.fix_normals()
        self.refresh()

    def populate_grid(self) -> None:
        """Turn grid data into plane"""
        self.clear_triangles()

        self.grid = [[0.0] * self.y for _i in range(self.x)]
        self.color_grid = [[WHITE] * self.y for _i in range(self.x)]

        step_x = self.width / (self.x - 1)
        step_y = self.height / (self.y - 1)
        step_u = 1.0 / (self.x - 1)
        step_v = 1.0 / (self.y - 1)

        c_x = self.width / 2.0
        c_y = self.height / 2.0
        for i, j in product(range(self.x), range(self.y)):
            self._vertices.append([(i * step_x) - c_x, (j * step_y) - c_y, self.grid[i][j]])
            self._texcoords.append([i * step_u, j * step_v])
            self._vertex_colors.append(WHITE)

            if i < self.x - 1 and j < self.y - 1:
                left = (self.x * i) + j
                right = (self.x * i) + (j + 1)
                top_left = (self.x * (i + 1)) + j
                top_right = (self.x * (i + 1)) + (j + 1)
                self._indices.append([top_right, right, left])
                self._indices.append([left, top_left, top_right])

        self.material._indices = self._indices
        self.fix_normals()
        self.refresh()
