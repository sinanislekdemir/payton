"""
Grid module

Grid is a basic layout *(virtual ground)* for the entire scene which centers
the origin of the scene (0, 0, 0) and can not be moved.

Grid size can be adjusted. Grid is a perfect way to visually see the movement
and positions of objects in space.
"""
import numpy as np
import ctypes

from OpenGL.GL import (glDeleteVertexArrays, glIsVertexArray, glPolygonMode,
                       GL_FRONT_AND_BACK, GL_LINE, glDrawElements,
                       glBindVertexArray, GL_FILL, glBindBuffer, glBufferData,
                       glGenVertexArrays, glGenBuffers, GL_ARRAY_BUFFER,
                       glEnableVertexAttribArray, glVertexAttribPointer,
                       GL_FLOAT, GL_STATIC_DRAW, GL_ELEMENT_ARRAY_BUFFER,
                       GL_LINES, GL_UNSIGNED_INT, glDeleteBuffers)
from payton.scene.shader import lightless_fragment_shader, Shader

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
    def __init__(self, **args):
        """Initialize Grid

        Args:
          xres: Number of lines in X
          yres: Number of lines in Y
        """
        xres = args.get('xres', 20)
        yres = args.get('yres', 20)
        self.color = args.get('color', [0.4, 0.4, 0.4])

        self.static = True
        self.matrix = [1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0]
        self._vertices = []
        self._indices = []
        self._vertex_count = 0
        self._model_matrix = None

        # Vertex Array Object pointer
        self._vao = None
        variables = ['model', 'view', 'projection',
                     'light_pos', 'light_color', 'object_color']
        self._shader = Shader(fragment=lightless_fragment_shader,
                              variables=variables)
        self.visible = True

        self.resize(xres, yres)

    def destroy(self):
        """
        Destroy objects self
        """
        if self._vao:
            glDeleteVertexArrays(1, [self._vao])
        return True

    def render(self, proj, view, _lights):
        """
        Virtual function for rendering the object.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
        """

        if not self.visible:
            return True

        if not self._vao:
            self.build()

        # Setup shader arguments
        self._shader.use()
        self._model_matrix = np.array(self.matrix, dtype=np.float32)
        self._shader.set_matrix4x4_np('model', self._model_matrix)
        self._shader.set_matrix4x4_np('view', view)
        self._shader.set_matrix4x4_np('projection', proj)
        self._shader.set_vector3('object_color', np.array(self.color,
                                                          dtype=np.float32))

        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE)
            glDrawElements(GL_LINES, self._vertex_count,
                           GL_UNSIGNED_INT, ctypes.c_void_p(0))
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)

        # render
        self._shader.end()

    def resize(self, xres, yres, spacing=1):
        self._vertices = []
        self._indices = []
        self._vertex_count = 0
        ystart = -(yres*spacing / 2.0)
        xstart = -(xres*spacing / 2.0)
        for j in range(0, yres):
            y = ystart + (j * spacing)
            for i in range(0, xres):
                x = xstart + (i * spacing)
                self._vertices += [x, y, 0.0]

        for j in range(0, yres - 1):
            offset = j * xres
            for i in range(0, xres - 1):
                k = offset + i
                self._indices += [k, k + 1,
                                  k + 1, k + xres + 1,
                                  k + xres + 1, k + xres,
                                  k + xres, k]

        self._vertex_count = len(self._indices)
        if self._vao:
            glDeleteVertexArrays(1, [self._vao])
        self._vao = None

    def set_color(self, color):
        self.color = color

    def build(self):
        self._vao = glGenVertexArrays(1)
        vbos = glGenBuffers(2)
        glBindVertexArray(self._vao)

        self._shader.build()
        vertices = np.array(self._vertices, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)

        glBindBuffer(GL_ARRAY_BUFFER, vbos[0])
        glEnableVertexAttribArray(0) # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        glBufferData(GL_ARRAY_BUFFER, vertices.nbytes,
                     vertices, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vbos[1])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        # glDisableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDeleteBuffers(2, vbos)

        if self.static:
            # we can clear this data to free some more memory
            self._vertices = []
            self._indices = []

        return True
