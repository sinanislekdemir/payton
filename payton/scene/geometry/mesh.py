# pylama:ignore=C901
import logging
from typing import Any, Optional

from payton.scene.geometry import Object
from payton.math.vector import plane_normal, vector_angle
from payton.scene.types import VList


class Mesh(Object):
    """Mesh Object

    Mesh is almost like the Object except with some extra methods to make
    things easier. If you want to have custom geometries/shapes, it is
    better to extend `payton.scene.geometry.Mesh` instead of
    `payton.scene.geometry.Object`. Because Mesh will give you better
    and easier constructing capabilities such as adding triangles on the fly
    or sub-division or cutting and so forth. It is a way of designing objects
    by code.


    Example use case:

        .. include:: ../../../examples/basics/09_mesh.py
    """

    def __init__(self, **args: Any) -> None:
        super().__init__(**args)
        self.static = False

    def clear_triangles(self) -> None:
        """Clear all triangles inside the Mesh"""
        self._vertices = []
        self._indices = []
        self._normals = []
        self._texcoords = []
        self._vertex_colors = []
        self.refresh()

    def fix_normals(self) -> None:
        """Try to re-calculate Mesh normals, if your object has already perfect
        normals, do not call this method"""
        self._normals = [[0, 0, 1.0]] * len(self._vertices)

        for face in self._indices:
            v1, v2, v3 = (
                self._vertices[face[0]],
                self._vertices[face[1]],
                self._vertices[face[2]],
            )
            normal = plane_normal(v1, v2, v3)
            self._normals[face[0]] = normal
            self._normals[face[1]] = normal
            self._normals[face[2]] = normal

    def fix_texcoords(self, u: int = 1, v: int = 1) -> None:
        """Try to recalculate mesh texture coordinates by cube projection
        """
        self._texcoords = []
        self._calc_bounds()
        bbox = self._bounding_box
        vmin, vmax = bbox[0], bbox[1]
        width = vmax[0] - vmin[0]
        depth = vmax[1] - vmin[1]
        height = vmax[2] - vmin[2]
        normals = [
            [0.0, -1.0, 0.0],  # front
            [1.0, 0.0, 0.0],  # right
            [-1.0, 0.0, 0.0],  # left
            [0.0, 1.0, 0.0],  # back
            [0.0, 0.0, 1.0],  # top
            [0.0, 0.0, -1.0],  # bottom
        ]
        for face in self._indices:
            v1, v2, v3 = (
                self._vertices[face[0]],
                self._vertices[face[1]],
                self._vertices[face[2]],
            )
            # find face normal
            normal = plane_normal(v1, v2, v3)
            # determine which face of the cube
            angles = [vector_angle(normal, n) for n in normals]
            face_dir = angles.index(min(angles))
            if face_dir == 0:
                t1 = (v1[0] - vmin[0]) / width
                s1 = (v1[2] - vmin[2]) / height
                t2 = (v2[0] - vmin[0]) / width
                s2 = (v2[2] - vmin[2]) / height
                t3 = (v3[0] - vmin[0]) / width
                s3 = (v3[2] - vmin[2]) / height
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
            if face_dir == 1:
                t1 = (v1[1] - vmin[1]) / depth
                s1 = (v1[2] - vmin[2]) / height
                t2 = (v2[1] - vmin[1]) / depth
                s2 = (v2[2] - vmin[2]) / height
                t3 = (v3[1] - vmin[1]) / depth
                s3 = (v3[2] - vmin[2]) / height
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
            if face_dir == 2:
                t1 = (vmax[1] - v1[1]) / depth
                s1 = (v1[2] - vmin[2]) / height
                t2 = (vmax[1] - v2[1]) / depth
                s2 = (v2[2] - vmin[2]) / height
                t3 = (vmax[1] - v3[1]) / depth
                s3 = (v3[2] - vmin[2]) / height
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
            if face_dir == 3:
                t1 = (vmax[0] - v1[0]) / width
                s1 = (v1[2] - vmin[2]) / height
                t2 = (vmax[0] - v2[0]) / width
                s2 = (v2[2] - vmin[2]) / height
                t3 = (vmax[0] - v3[0]) / width
                s3 = (v3[2] - vmin[2]) / height
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
            if face_dir == 4:
                t1 = (v1[0] - vmin[0]) / width
                s1 = (v1[1] - vmin[1]) / depth
                t2 = (v2[0] - vmin[0]) / width
                s2 = (v2[1] - vmin[1]) / depth
                t3 = (v3[0] - vmin[0]) / width
                s3 = (v3[1] - vmin[1]) / depth
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
            if face_dir == 5:
                t1 = (v1[0] - vmin[0]) / width
                s1 = (v1[1] - vmin[1]) / depth
                t2 = (v2[0] - vmin[0]) / width
                s2 = (v2[1] - vmin[1]) / depth
                t3 = (v3[0] - vmin[0]) / width
                s3 = (v3[1] - vmin[1]) / depth
                self._texcoords.extend([[t1, s1], [t2, s2], [t3, s3]])
        self._texcoords = [[t[0] * u, t[1] * v] for t in self._texcoords]

    def scale(self, x: float, y: float, z: float) -> None:
        """Scale Mesh by given factors

        This does not create a scale matrix and multiply existing matrix
        with it. Instead, it will scale the vertices by given factors.
        """
        self._vertices = list(
            map(lambda v: [v[0] * x, v[1] * y, v[2] * z], self._vertices)
        )
        self.fix_normals()
        self.refresh()

    def add_triangle(
        self,
        vertices: VList,
        normals: Optional[VList] = None,
        texcoords: Optional[VList] = None,
        colors: Optional[VList] = None,
    ) -> None:
        """Add triangle to Mesh

        Args:
          vertices: Vertices of the triangle. This is required. Ex:
                    `[[0, 0, 0], [2, 0, 0], [1, 1, 0]]`
          normals: Normals of the triangle. _(When left as None, Payton will
                   calculate the surface normal based on vertices and assign
                   it per given vertex.)_
          texcoords: Texture UV coordinates.
          colors: Per vertex colors (optional)
        """
        if len(vertices) != 3:
            logging.error("A triangle must have 3 vertices")
            return

        if normals is not None and len(normals) != 3:
            logging.error("There must be one normal per vertex")
            return

        if texcoords is not None and len(texcoords) != 3:
            logging.error("There must be one texcoord per vertex")
            return

        if normals is None:
            v1, v2, v3 = vertices[0], vertices[1], vertices[2]
            normal = plane_normal(v1, v2, v3)
            normals = [normal, normal, normal]
        if texcoords is None:
            texcoords = [[0, 0], [1, 0], [1, 1]]
        if colors:
            for color in colors:
                self._vertex_colors.append(color)

        for v in vertices:
            self._vertices.append(v)

        i = len(self._indices) * 3
        self._indices.append([i, i + 1, i + 2])
        for normal in normals:
            self._normals.append(normal)
        for t in texcoords:
            self._texcoords.append(t)
        self.refresh()
