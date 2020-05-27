# pylama:ignore=C901
import ctypes
import logging
import pickle
from copy import deepcopy
from typing import Any, Dict, List, Optional, Union

import numpy as np  # type: ignore
from OpenGL.GL import (
    GL_ARRAY_BUFFER,
    GL_DEPTH_TEST,
    GL_DYNAMIC_DRAW,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_FILL,
    GL_FLOAT,
    GL_FRONT_AND_BACK,
    GL_LINE,
    GL_LINE_STRIP,
    GL_POINTS,
    GL_TRIANGLES,
    GL_UNSIGNED_INT,
    glBindBuffer,
    glBindVertexArray,
    glBufferData,
    glBufferSubData,
    glDeleteVertexArrays,
    glDisable,
    glDrawElements,
    glEnable,
    glEnableVertexAttribArray,
    glGenBuffers,
    glGenVertexArrays,
    glIsVertexArray,
    glPolygonMode,
    glVertexAttribPointer,
)

from payton.math.geometry import raycast_sphere_intersect
from payton.math.matrix import create_rotation_matrix, scale_matrix
from payton.math.vector import add_vectors, cross_product, normalize_vector, scale_vector, sub_vector, vector_transform
from payton.scene.material import DEFAULT, NO_INDICE, NO_VERTEX_ARRAY, POINTS, SOLID, WIREFRAME, Material
from payton.scene.shader import DEFAULT_SHADER, PARTICLE_SHADER, Shader
from payton.scene.types import IList, VList


class Object(object):
    def __init__(self, name="", visible=True, track_motion=False, **kwargs: Dict[str, Any],) -> None:
        self.children: Dict[str, Object] = {}

        # store diffeerent materials
        self.materials: Dict[str, Material] = {DEFAULT: Material()}

        self._vao: int = NO_VERTEX_ARRAY
        self._vbos: List[int] = []
        self._no_missing_vao = False

        self.name = name
        self._visible = visible
        self.matrix: VList = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
        # Object vertices. Each vertex has 3 decimals (X, Y, Z). Vertices
        # are continuous. [X, Y, Z, X, Y, Z, X, Y, Z, X, ... ]
        #                  -- 1 --  -- 2 --  -- 3 --  -- 4 --

        self._vertices: VList = []  # Object vertex list

        # @NOTE: we have separate indices for materials but this base
        #        index list holds all indice definitions for fast access
        self._indices: IList = []  # Indices
        self._normals: List[List[float]] = []  # Vertex normals, 1 normal coordinate for 1 Vertex
        self._texcoords: List[List[float]] = []  # Texture coordinates, 1 coordinate per Vertex
        self._vertex_colors: List[List[float]] = []  # per-vertex colors, optional.
        self._has_vertex_colors: bool = False  # flag for using vertex colors

        # Vertices do not mean anything unless we define how to use them.
        # For instance, 3 vertices make a triangle or 2 vertices define a line
        # order of vertices are defined in materials indices
        self._vertex_count: int = 0  # Number of vertices to report to OpenGL.

        # This is an optimization technique for dynamic objects where there are
        # increasing number of vertices. We allocate some buffer before-hand
        # and if we fill all of it, we resize it.
        self._buffer_size: float = 500 * 12
        self._t_buffer_size: float = 500 * 8
        self._model_matrix: np.ndarray = []  # Model matrix.
        self._model_matrix_fortran: np.ndarray = []  # Model matrix as fortran
        # Check if buffer size allocated for the object has changed.
        self._buffer_size_changed: bool = True
        self._t_buffer_size_changed: bool = True

        self._absolute_cache: Dict[bytes, List[float]] = {}
        self.shader = DEFAULT_SHADER

        # Track object motion
        self.track_motion = track_motion
        # Motion path, stores every matrix change.
        self._motion_path: List[VList] = []

        if not isinstance(self, Line):
            # _motion_path_line is used to display the motion path in scene
            self._motion_path_line = Line()
        self._previous_matrix: Union[np.ndarray, None] = None

        # For raycast tests - bounding radius is the radius of the bounding
        # sphere.
        self._bounding_radius: float = 0
        self._bounding_box: VList = []
        self._selected: bool = False

        # Vertex Array Object pointer
        self._needs_update: bool = False  # Object geometry has changed.
        self._hit: bool = False

        self._absolute_vertices: Optional[List[List[float]]] = None

    def refresh(self) -> None:
        self._needs_update = True

    @property
    def material(self) -> Material:
        return self.materials[DEFAULT]

    @material.setter
    def material(self, mat: Material) -> None:
        self.materials[DEFAULT] = mat

    def add_material(self, name: str, material: Material) -> None:
        if name in self.materials:
            raise Exception(f"Name {name} already exists")
        self.materials[name] = deepcopy(material)

    @property
    def direction(self) -> List[float]:
        return self.matrix[1][:3]

    @direction.setter
    def direction(self, v: List[float]):
        if len(v) < 3:
            raise Exception("Direction needs 3 components (x,y,z)")
        self.matrix[1][0] = v[0]
        self.matrix[1][1] = v[1]
        self.matrix[1][2] = v[2]
        left = cross_product(self.matrix[1], self.matrix[2])
        left += [0]
        self.matrix[0] = normalize_vector(left)
        up = cross_product(self.matrix[0], self.matrix[1])
        up += [0]
        self.matrix[2] = normalize_vector(up)
        self._absolute_vertices = None

    def direct_to(self, v: List[float]):
        diff = sub_vector(v, self.position)
        diff = normalize_vector(diff)
        self.direction = diff

    def rotate_around_z(self, angle: float) -> None:
        rot_matrix = create_rotation_matrix([0, 0, 1], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._absolute_vertices = None

    def rotate_around_x(self, angle: float) -> None:
        rot_matrix = create_rotation_matrix([1, 0, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._absolute_vertices = None

    def rotate_around_y(self, angle: float) -> None:
        rot_matrix = create_rotation_matrix([0, 1, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._absolute_vertices = None

    def scale(self, x: float, y: float, z: float) -> None:
        sm = scale_matrix(x, y, z)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = sm.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._absolute_vertices = None

    def scale_texture(self, x: float, y: float) -> None:
        self._texcoords = [[coord[0] * x, coord[1] * y] for coord in self._texcoords]
        self.refresh()

    def select(self, start: np.ndarray, vector: np.ndarray) -> bool:
        self._selected = raycast_sphere_intersect(
            start, vector, np.array(self.matrix[3], dtype=np.float32), self.bounding_radius,
        )

        for obj in self.children:
            x = self.children[obj].select(start, vector)
            if not self._selected and x:
                self._selected = True

        return self._selected

    def destroy(self) -> bool:
        for material in self.materials.values():
            if material._vao > NO_VERTEX_ARRAY:
                glDeleteVertexArrays(1, [self._vao])
                material._vao = NO_VERTEX_ARRAY
                material._initialized = False

        self._buffer_size_changed = True
        self._t_buffer_size_changed = True
        self._no_missing_vao = False
        if self._vao > NO_VERTEX_ARRAY:
            glDeleteVertexArrays(1, [self._vao])
            self._vao = NO_VERTEX_ARRAY

        for child in self.children.values():
            child.destroy()

        return True

    def step_back(self, steps: int = 1) -> bool:
        steps += 1
        if not self.track_motion:
            raise Exception("track_motion should be True")
        if len(self._motion_path) < steps:
            return False

        self.matrix = self._motion_path[-steps]
        del self._motion_path[-steps + 1 :]  # noqa
        self._absolute_vertices = None
        return True

    def forward(self, distance: float) -> None:
        diff = scale_vector(self.matrix[1], distance)
        self.matrix[3] = add_vectors(self.matrix[3], diff)
        self.matrix[3][3] = 1.0
        self._absolute_vertices = None

    def update_matrix(self, parent_matrix: Optional[np.ndarray] = None) -> None:
        # Turn matrix into numpy array. Numpy arrays are C Type arrays
        # suitable for OpenGL Pipeline
        self._model_matrix = np.array(self.matrix, dtype=np.float32)

        # When there is a parent object, child object follows parents matrix
        if parent_matrix is not None and len(parent_matrix) > 0:
            self._model_matrix = self._model_matrix.dot(parent_matrix)

        # Fortran array is used to push matrix to opengl context
        self._model_matrix_fortran = np.asfortranarray(self._model_matrix, dtype=np.float32)
        self._absolute_cache = {}

    def track(self) -> bool:
        if not self.track_motion:
            return False

        if self._previous_matrix == self.matrix[3]:
            return True

        # Add the new matrix to motion path records
        self._motion_path.append(deepcopy(self.matrix))
        # Add the matrix position to motion math line for visualisation
        if self._motion_path_line is not None:
            self._motion_path_line.append([[self.matrix[3][0], self.matrix[3][1], self.matrix[3][2]]])

        # Python trick here! need to .copy or it will pass reference.
        self._previous_matrix = self.matrix[3].copy()
        return True

    @property
    def visible(self) -> bool:
        return self._visible

    def show(self) -> None:
        self._visible = True

    def hide(self) -> None:
        self._visible = False

    def clone(self) -> "Object":
        return deepcopy(self)

    @property
    def has_missing_vao(self) -> bool:
        if self._no_missing_vao:
            return False
        result = any(
            [material._vao == NO_VERTEX_ARRAY and len(material._indices) > 1 for material in self.materials.values()]
        )
        if not result:
            self._no_missing_vao = True
        return result

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ) -> None:
        if not self._visible:
            return

        if self.has_missing_vao or self._needs_update:
            self.build()

        self.update_matrix(parent_matrix=parent_matrix)
        self.track()

        if self._vertex_count == 0:
            # dummy object, render children and leave
            # render children
            for child in self.children:
                self.children[child].render(
                    lit, shader, self._model_matrix,
                )

            return

        # Material shading mode.
        mode = None
        if self._has_vertex_colors:
            mode = Shader.PER_VERTEX_COLOR

        shader.set_matrix4x4_np("model", self._model_matrix_fortran)
        indice_0 = ctypes.c_void_p(0)

        if self.shader == PARTICLE_SHADER:
            glDisable(GL_DEPTH_TEST)

        for material in self.materials.values():
            if not material.display == SOLID and shader._depth_shader:
                continue

            material.render(
                lit, shader, mode,
            )

            # Actual rendering
            if material._vao > NO_VERTEX_ARRAY and glIsVertexArray(material._vao):
                glBindVertexArray(material._vao)
                pmode = GL_LINE
                primitive = GL_TRIANGLES
                if material.display == SOLID:
                    pmode = GL_FILL
                    primitive = GL_TRIANGLES
                if material.display == POINTS:
                    pmode = GL_FILL
                    primitive = GL_POINTS
                if _primitive is not None:
                    primitive = _primitive
                glPolygonMode(GL_FRONT_AND_BACK, pmode)

                glDrawElements(
                    primitive, material._vertex_count, GL_UNSIGNED_INT, indice_0,
                )

                if pmode != GL_FILL:
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
                glBindVertexArray(0)

        if self.shader == PARTICLE_SHADER:
            glEnable(GL_DEPTH_TEST)

        # Render motion path
        if self.track_motion:
            self._motion_path_line.render(lit, shader, parent_matrix)

        # render children
        for child in self.children:
            self.children[child].render(lit, shader, self._model_matrix)

    @property
    def position(self) -> List[float]:
        return self.matrix[3][:3]

    @position.setter
    def position(self, pos: List[float]) -> None:
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0.0]
        self.matrix[3][0] = pos[0]
        self.matrix[3][1] = pos[1]
        self.matrix[3][2] = pos[2]
        self._absolute_vertices = None

    def add_child(self, name: str, obj: "Object") -> bool:
        if name in self.children:
            logging.error(f"Name {name} exists in object children")
            return False
        if not isinstance(obj, Object):
            logging.error("Object type is not valid")
            return False
        self.children[name] = obj
        return True

    def to_absolute(self, coordinates: List[float]) -> List[float]:
        key = pickle.dumps(coordinates)
        if key in self._absolute_cache:
            return self._absolute_cache[key]
        trans = vector_transform(coordinates, self.matrix)
        self._absolute_cache[key] = trans
        return trans

    def absolute_vertices(self) -> List[List[float]]:
        if self._absolute_vertices is None:
            self._absolute_vertices = [self.to_absolute(v) for v in self._vertices]

        return self._absolute_vertices

    def toggle_wireframe(self) -> None:
        d = (self.material.display + 1) % 3

        for mat in self.materials.values():
            mat.display = d

        if d == POINTS:
            self.shader = PARTICLE_SHADER
            self.refresh()
        else:
            self.shader = DEFAULT_SHADER
            self.refresh()

        for child in self.children.values():
            child.toggle_wireframe()

    def _calc_bounds(self) -> float:
        bmin: Optional[List[float]] = None
        bmax: Optional[List[float]] = None
        absolute_vertices = self.absolute_vertices()
        x = [v[0] for v in absolute_vertices]
        y = [v[1] for v in absolute_vertices]
        z = [v[2] for v in absolute_vertices]

        if len(x) == 0 or len(y) == 0 or len(z) == 0:
            bmin = [0.0, 0.0, 0.0]
            bmax = [0.0, 0.0, 0.0]
        else:
            bmin = [min(x), min(y), min(z)]
            bmax = [max(x), max(y), max(z)]

        a = (bmax[0] - bmin[0]) / 2.0
        b = (bmax[1] - bmin[1]) / 2.0
        c = (bmax[2] - bmin[2]) / 2.0
        self._bounding_radius = max([a, b, c])
        self._bounding_box = [bmin, bmax]

        return self._bounding_radius

    @property
    def bounding_radius(self) -> float:
        if self._bounding_radius > 0:
            return self._bounding_radius
        return self._calc_bounds()

    @property
    def bounding_box(self) -> VList:
        if self._absolute_vertices is None:
            self._calc_bounds()
        if len(self._bounding_box) > 0:
            return self._bounding_box
        self._calc_bounds()
        return self._bounding_box

    def build(self) -> bool:
        self._vertex_count = 0

        # Turn python arrays into C type arrays using Numpy.
        # This is required for OpenGL. Python memory model is a bit
        # different than raw memory model of C (OpenGL)
        vertices = np.array(self._vertices, dtype=np.float32).flatten()
        normals = np.array(self._normals, dtype=np.float32).flatten()
        texcoords = np.array(self._texcoords, dtype=np.float32).flatten()
        colors = np.array(self._vertex_colors, dtype=np.float32).flatten()

        if self._vao == NO_INDICE:
            return False

        if self._vao == NO_VERTEX_ARRAY:
            self._vao = glGenVertexArrays(1)
            self._vbos = glGenBuffers(5)

        # We will let all materials to share the same buffer objects
        # We need 4 buffers (vertex, normal, texcoord, color)

        glBindVertexArray(self._vao)

        draw = GL_DYNAMIC_DRAW

        # Buffer overflow, we need more space.
        if self._buffer_size < vertices.nbytes:
            self._buffer_size = vertices.nbytes
            self._buffer_size_changed = True
        if self._t_buffer_size < texcoords.nbytes:
            self._t_buffer_size = texcoords.nbytes
            self._t_buffer_size_changed = True

        # Bind Vertices
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[0])
        if self._buffer_size_changed:
            # glBufferData creates a new data area
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, vertices, draw)
        else:
            # glBufferSubData just replaces memory area in buffer so it is
            # much more efficient way to handle things.
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.nbytes, vertices)

        # Bind Normals
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[1])
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, normals, draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, normals.nbytes, normals)

        # Bind TexCoords
        if len(self._texcoords) == len(self._vertices):
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[2])
            if self._t_buffer_size_changed:
                glBufferData(GL_ARRAY_BUFFER, self._t_buffer_size, texcoords, draw)
            else:
                glBufferSubData(GL_ARRAY_BUFFER, 0, texcoords.nbytes, texcoords)

        # Bind Vertex Colors
        if len(self._vertex_colors) == len(self._vertices):
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[4])
            self._has_vertex_colors = True
            if self._buffer_size_changed:
                glBufferData(GL_ARRAY_BUFFER, self._buffer_size, colors, draw)
            else:
                glBufferSubData(GL_ARRAY_BUFFER, 0, colors.nbytes, colors)

        glBindVertexArray(0)
        self._buffer_size_changed = False
        self._t_buffer_size_changed = False

        for material in self.materials.values():
            if len(material._indices) == 0:
                material._vao == NO_INDICE
                continue

            if material._vao == NO_VERTEX_ARRAY:
                # Generate Vertex Array
                material._vao = glGenVertexArrays(1)
                # We need 1 buffer for material as indices
                material._vbos = [glGenBuffers(1)]

            glBindVertexArray(material._vao)

            indices = np.array(material._indices, dtype=np.int32).flatten()

            # Bind Vertices
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[0])
            glEnableVertexAttribArray(0)  # shader layout location
            glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))

            # Bind Normals
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[1])
            glEnableVertexAttribArray(1)  # shader layout location
            glVertexAttribPointer(1, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))

            # Bind TexCoords
            if len(self._texcoords) == len(self._vertices):
                glBindBuffer(GL_ARRAY_BUFFER, self._vbos[2])
                glEnableVertexAttribArray(2)  # shader layout location
                glVertexAttribPointer(2, 2, GL_FLOAT, False, 0, ctypes.c_void_p(0))

            # Bind Vertex Colors
            if len(self._vertex_colors) == len(self._vertices):
                glBindBuffer(GL_ARRAY_BUFFER, self._vbos[4])
                glEnableVertexAttribArray(3)  # shader layout location
                glVertexAttribPointer(3, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))

            # Bind Indices
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, material._vbos[0])
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, draw)
            i_len = len(indices)
            material._vertex_count = i_len
            self._vertex_count += i_len

            glBindVertexArray(0)
            glBindBuffer(GL_ARRAY_BUFFER, 0)

        self._needs_update = False
        return True

    def to_dict(self) -> Dict[str, Any]:
        return {
            "vertices": self._vertices,
            "normals": self._normals,
            "texcoords": self._texcoords,
            "matrix": self.matrix,
            "materials": {mat: self.materials[mat].to_dict() for mat in self.materials},
            "children": {n: self.children[n].to_dict() for n in self.children},
        }


class Line(Object):
    def __init__(self, vertices: Optional[VList] = None, color: Optional[List[float]] = None, **kwargs: Any,) -> None:
        super().__init__(**kwargs)
        self._vertices: VList = [] if vertices is None else vertices
        self.material.color = [1.0, 1.0, 1.0] if color is None else color

        self.material.display = WIREFRAME
        self.material.lights = False
        self.build_lines()

    def toggle_wireframe(self) -> None:
        """Toggle Wireframe overwrite to disable mode change"""

    def add_material(self, name: str, material: Material) -> None:
        """@TODO Implement this later! Not urgent"""
        logging.error("Can't add materials to Line object")

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ) -> None:
        super().render(lit, shader, parent_matrix, GL_LINE_STRIP)

    def append(self, vertices: VList, material: str = DEFAULT) -> None:
        diff = len(vertices)  # Number of vertices added
        last_index = len(self._vertices)
        self._vertices += vertices

        self._texcoords += [[0, 0]] * diff
        self._normals += [[0, 0, 1]] * diff
        self._vertex_count = len(self._vertices)
        self.material._vertex_count = self._vertex_count

        indices = list(map(lambda x: x + last_index, range(diff)))
        self.material._indices.extend([indices])
        self._indices = self.material._indices

        self._needs_update = True

    def build_lines(self, vertices: Optional[VList] = None, color: Optional[List[float]] = None) -> None:
        if vertices is not None:
            self._vertices = vertices
        if color is not None:
            self.material.color = color
        self._vertex_count = len(self._vertices)
        self.material._vertex_count = self._vertex_count

        for i in range(self._vertex_count - 1):
            self.material._indices.append([i, i + 1])
            self._indices = self.material._indices

        for i in range(self._vertex_count):
            self._normals.append([0, 0, 0])
            self._texcoords.append([0, 0])
        self._needs_update = True
