"""Cube object module."""
from typing import Any, Optional

from payton.math.functions import min_max
from payton.math.vector import Vector3D
from payton.scene.geometry.mesh import Mesh
from payton.scene.material import DEFAULT

_BULLET = False
try:
    import pybullet

    _BULLET = True
except ModuleNotFoundError:
    _BULLET = False


class Cube(Mesh):
    """Basic Cube Mesh."""

    def __init__(
        self,
        width: float = 1.0,
        depth: float = 1.0,
        height: float = 1.0,
        from_corner: Optional[Vector3D] = None,
        to_corner: Optional[Vector3D] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize the Cube object.

        Instead of using W/D/H arguments, you can define a Cube
        starting from a point A filling up to point B.

        Keyword arguments:
        width -- Width of the cube (default = 1)
        depth -- Depth of the cube (default = 1)
        height -- Height of the cube (default = 1)
        from_corner -- (Optional) Starting point A
        to_corner -- (Optional) Ending point B
        """
        super().__init__(**kwargs)
        width *= 0.5
        depth *= 0.5
        height *= 0.5
        self._width = width
        self._depth = depth
        self._height = height

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

    def _create_collision_shape(self) -> None:
        self._bullet_shape_id = pybullet.createCollisionShape(
            pybullet.GEOM_BOX, halfExtents=[self._width, self._depth, self._height]
        )
