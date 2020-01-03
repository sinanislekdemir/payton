import logging
from typing import Any, Optional

from payton.scene.geometry.base import Object
from payton.scene.material import DEFAULT, POINTS
from payton.scene.types import VList


class PointCloud(Object):
    def __init__(self, vertices: Optional[VList] = None, colors: Optional[VList] = None, **kwargs: Any,) -> None:
        super().__init__(**kwargs)
        self._vertices: VList = [] if vertices is None else vertices
        # Expose vertices by reference for modification
        self.vertices: VList = self._vertices
        self._vertex_colors: VList = [] if colors is None else colors
        self._vertex_history: VList = []
        self.material.display = POINTS

    def toggle_wireframe(self) -> None:
        pass

    def track(self) -> bool:
        return False

    def add(self, vertices: VList, colors: Optional[VList] = None, material: str = DEFAULT,) -> None:
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
