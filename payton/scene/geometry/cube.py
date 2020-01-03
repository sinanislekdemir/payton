from typing import Any, List, Optional

from payton.math.vector import min_max
from payton.scene.geometry.mesh import Mesh
from payton.scene.material import DEFAULT


class Cube(Mesh):
    def __init__(
        self,
        width: float = 1.0,
        depth: float = 1.0,
        height: float = 1.0,
        from_corner: Optional[List[float]] = None,
        to_corner: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> None:
        super().__init__(**kwargs)
        width *= 0.5
        depth *= 0.5
        height *= 0.5

        if from_corner is not None and to_corner is not None:
            vmin, vmax = min_max([from_corner, to_corner])

            width = (vmax[0] - vmin[0]) / 2
            depth = (vmax[1] - vmin[1]) / 2
            height = (vmax[2] - vmin[2]) / 2
            self.position = [
                vmin[0] + width,
                vmin[1] + depth,
                vmin[2] + height,
            ]

        _vertices = [
            [width, depth, height],
            [width, depth, -height],
            [width, -depth, height],
            [width, -depth, -height],
            [-width, depth, height],
            [-width, depth, -height],
            [-width, -depth, height],
            [-width, -depth, -height],
        ]

        _normals = [
            [0.0, 0.0, 1.0],
            [0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ]

        _texcoords = [
            [0.50, 0.333333],
            [0.25, 0.000000],
            [0.25, 0.333333],
            [1.00, 0.333333],
            [0.75, 0.666666],
            [1.00, 0.666666],
            [0.75, 0.333333],
            [0.50, 0.666666],
            [0.25, 0.666666],
            [0.50, 1.000000],
            [0.50, 0.666666],
            [0.25, 0.333333],
            [0.00, 0.666666],
            [0.25, 0.666666],
            [0.50, 0.333333],
            [0.25, 0.666666],
            [0.50, 0.666666],
            [0.50, 0.000000],
            [0.50, 0.333333],
            [0.25, 1.000000],
            [0.00, 0.333333],
            [0.25, 0.333333],
        ]

        _faces = [
            [[4, 0, 0], [2, 1, 0], [0, 2, 0]],
            [[2, 3, 1], [7, 4, 1], [3, 5, 1]],
            [[6, 6, 2], [5, 7, 2], [7, 4, 2]],
            [[1, 8, 3], [7, 9, 3], [5, 10, 3]],
            [[0, 11, 4], [3, 12, 4], [1, 13, 4]],
            [[4, 14, 5], [1, 15, 5], [5, 16, 5]],
            [[4, 0, 0], [6, 17, 0], [2, 1, 0]],
            [[2, 3, 1], [6, 6, 1], [7, 4, 1]],
            [[6, 6, 2], [4, 18, 2], [5, 7, 2]],
            [[1, 8, 3], [3, 19, 3], [7, 9, 3]],
            [[0, 11, 4], [2, 20, 4], [3, 12, 4]],
            [[4, 14, 5], [0, 21, 5], [1, 15, 5]],
        ]

        i = 0
        for index in _faces:
            ind = []
            for f in index:
                l_vertex = _vertices[f[0]]
                l_normal = _normals[f[2]]
                l_tex = _texcoords[f[1]]
                self._vertices.append(l_vertex)
                self._normals.append(l_normal)
                self._texcoords.append(l_tex)
                ind.append(i)
                i += 1
            self.materials[DEFAULT]._indices.append(ind)
        self._indices = self.materials[DEFAULT]._indices

        return None
