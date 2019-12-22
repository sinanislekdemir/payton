import logging
from typing import Any, Optional

from payton.scene.geometry.base import Object
from payton.scene.material import DEFAULT, POINTS
from payton.scene.types import VList


class PointCloud(Object):
    """Point cloud

    If you change the vertices, do not forget to do a `refresh` to take
    effect.

    Example use case:

        .. include:: ../../../examples/basics/11_point_cloud.py
    """

    def __init__(
        self,
        vertices: Optional[VList] = None,
        colors: Optional[VList] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize Point Cloud

        Args:
          vertices: List of point vertices
          colors: List of colors per vertex, follows the same index as vertices
        """
        super().__init__(**kwargs)
        self._vertices: VList = [] if vertices is None else vertices
        # Expose vertices by reference for modification
        self.vertices: VList = self._vertices
        self._vertex_colors: VList = [] if colors is None else colors
        self._vertex_history: VList = []
        self.material.display = POINTS

    def toggle_wireframe(self) -> None:
        """Toggle wireframe overwrite to disable mode change"""
        pass

    def track(self) -> bool:
        """Tracking point cloud is not possible at the moment

        Returns:
            bool: `False`. Tracking of point clouds not implemented.
        """
        return False

    def add(
        self,
        vertices: VList,
        colors: Optional[VList] = None,
        material: str = DEFAULT,
    ) -> None:
        """Add a point to the cloud

        Args:
          vertices: Vertices to add
          colors: Colors of the vertices in the same order. (Optional)
        """
        i = len(self._indices)
        for vertex in vertices:
            self._vertices.append(vertex)
            self._indices.append([i])
            self.materials[material]._indices.append([i])
            i += 1

        if colors is not None:
            if len(colors) != len(vertices):
                logging.error("len(colors) != len(vertices)")
                return
            for color in colors:
                self._vertex_colors.append(color)

        self._needs_update = True
