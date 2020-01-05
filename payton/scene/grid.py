import ctypes
from typing import Any, List, Optional

import numpy as np  # type: ignore
from OpenGL.GL import (
    GL_ARRAY_BUFFER,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_FILL,
    GL_FLOAT,
    GL_FRONT_AND_BACK,
    GL_LINE,
    GL_LINES,
    GL_STATIC_DRAW,
    GL_UNSIGNED_INT,
    glBindBuffer,
    glBindVertexArray,
    glBufferData,
    glDeleteBuffers,
    glDeleteVertexArrays,
    glDrawElements,
    glEnableVertexAttribArray,
    glGenBuffers,
    glGenVertexArrays,
    glIsVertexArray,
    glPolygonMode,
    glVertexAttribPointer,
)

from payton.scene.geometry.base import Line
from payton.scene.material import Material
from payton.scene.shader import Shader


class Grid(object):
    def __init__(self, xres: int = 20, yres: int = 20, color: Optional[List[float]] = None, **kwargs: Any,) -> None:
        if color is None:
            self._color: List[float] = [0.4, 0.4, 0.4]
        else:
            self._color = color

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
        self._lines: List[Line] = [
            Line(vertices=[[0.0, 0.0, 0.01], [xres / 2.0, 0.0, 0.01]], color=[1.0, 0.0, 0.0],),
            Line(vertices=[[0.0, 0.0, 0.01], [0.0, yres / 2.0, 0.01]], color=[0.0, 1.0, 0.0],),
            Line(vertices=[[0.0, 0.0, 0.01], [0.0, 0.0, yres / 2.0]], color=[0.0, 0.0, 1.0],),
        ]

        # Vertex Array Object pointer
        self._vao: int = -1
        self.visible: bool = True

        self.resize(xres, yres)

    def destroy(self) -> bool:
        if self._vao > -1:
            glDeleteVertexArrays(1, [self._vao])
            self._vao = -1
        for l in self._lines:
            l.destroy()
        return True

    def render(self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None,) -> bool:
        if not self.visible:
            return True

        if self._vao == -1:
            self.build()

        shader.set_matrix4x4_np("model", self._model_matrix)
        self._material.render(False, shader)

        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glDrawElements(
                GL_LINES, self._vertex_count, GL_UNSIGNED_INT, ctypes.c_void_p(0),
            )
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)

        for l in self._lines:
            l.render(False, shader, None)
        return True

    def resize(self, xres: int, yres: int, spacing: float = 1.0) -> None:
        self._vertices = []
        self._indices = []
        self._vertex_count = 0
        ystart = -(yres * spacing / 2.0)
        xstart = -(xres * spacing / 2.0)
        self._model_matrix = np.asfortranarray(np.array(self.matrix, dtype=np.float32), dtype=np.float32)
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

        vertices = np.array(self._vertices, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[0])
        glEnableVertexAttribArray(0)  # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes, vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos[1])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        # glDisableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDeleteBuffers(2, vbos)
