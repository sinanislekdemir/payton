"""
Grid module

Grid is a basic layout *(virtual ground)* for the entire scene which centers
the origin of the scene (0, 0, 0) and can not be moved.

Grid size can be adjusted. Grid is a perfect way to visually see the movement
and positions of objects in space.
"""
import numpy as np  # type: ignore
import ctypes

from typing import Any, List, Optional

from OpenGL.GL import (
    glDeleteVertexArrays,
    glIsVertexArray,
    glPolygonMode,
    GL_FRONT_AND_BACK,
    GL_LINE,
    glDrawElements,
    glBindVertexArray,
    GL_FILL,
    glBindBuffer,
    glBufferData,
    glGenVertexArrays,
    glGenBuffers,
    GL_ARRAY_BUFFER,
    glEnableVertexAttribArray,
    glVertexAttribPointer,
    GL_FLOAT,
    GL_STATIC_DRAW,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_LINES,
    GL_UNSIGNED_INT,
    glDeleteBuffers,
)

from payton.scene.material import Material
from payton.scene.light import Light


class Grid(object):
    """
    Properties of Grid:

    - `grid_size`: Default `10`
    - `grid_spacing`: Default `1.0`
    - `visible`: Default `True`

    Example usage:

        from payton.scene import Scene


        my_scene = Scene()
        my_scene.grid.grid_size = 20 #  we need more space
        my_scene.run()
    """

    def __init__(self, **args: Any) -> None:
        """Initialize Grid

        Args:
          xres: Number of lines in X
          yres: Number of lines in Y
        """
        xres: int = args.get("xres", 20)
        yres: int = args.get("yres", 20)
        self._color: List[float] = args.get("color", [0.4, 0.4, 0.4])

        self.static: bool = True
        self.matrix: List[float] = [
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        ]
        self._vertices: List[float] = []
        self._indices: List[int] = []
        self._vertex_count: int = 0
        self._model_matrix: Optional[np.ndarray] = None
        self._material: Material = Material(display=1, lights=False)
        self._material.color = self._color

        # Vertex Array Object pointer
        self._vao: int = -1
        self.visible: bool = True

        self.resize(xres, yres)

    def destroy(self) -> bool:
        """
        Destroy objects self
        """
        if self._vao > -1:
            glDeleteVertexArrays(1, [self._vao])
        return True

    def render(
        self, proj: np.ndarray, view: np.ndarray, _lights: List[Light]
    ) -> bool:
        """
        Virtual function for rendering the object.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
        """

        if not self.visible:
            return True

        if self._vao == -1:
            self.build()

        self._model_matrix = np.array(self.matrix, dtype=np.float32)
        self._material.render(proj, view, self._model_matrix, [])

        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glDrawElements(
                GL_LINES,
                self._vertex_count,
                GL_UNSIGNED_INT,
                ctypes.c_void_p(0),
            )
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)
        self._material.end()
        return True

    def resize(self, xres: int, yres: int, spacing: float = 1.0) -> None:
        self._vertices = []
        self._indices = []
        self._vertex_count = 0
        ystart = -(yres * spacing / 2.0)
        xstart = -(xres * spacing / 2.0)
        for j in range(0, yres):
            y = ystart + (j * spacing)
            for i in range(0, xres):
                x = xstart + (i * spacing)
                self._vertices += [x, y, 0.0]

        for j in range(0, yres - 1):
            offset = j * xres
            for i in range(0, xres - 1):
                k = offset + i
                self._indices += [
                    k,
                    k + 1,
                    k + 1,
                    k + xres + 1,
                    k + xres + 1,
                    k + xres,
                    k + xres,
                    k,
                ]

        self._vertex_count = len(self._indices)
        if self._vao > -1:
            glDeleteVertexArrays(1, [self._vao])
        self._vao = -1

    @property
    def color(self) -> List[float]:
        return self._color

    @color.setter
    def color(self, color: List[float]) -> None:
        self._color = color
        self._material.color = color

    def build(self) -> None:
        self._vao = glGenVertexArrays(1)
        vbos = glGenBuffers(2)
        glBindVertexArray(self._vao)

        self._material.build_shader()
        vertices = np.array(self._vertices, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[0])
        glEnableVertexAttribArray(0)  # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(
            GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW
        )

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos[1])
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW
        )
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        # glDisableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDeleteBuffers(2, vbos)

        if self.static:
            # we can clear this data to free some more memory
            self._vertices = []
            self._indices = []
