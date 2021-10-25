"""Base geometry definitions."""
# pylama:ignore=C901
import ctypes
import logging
from copy import deepcopy
from functools import lru_cache
from typing import Any, Dict, List, Optional, Tuple, Union, cast

import numpy as np
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
from pyrr import Quaternion

from payton.math.functions import (
    add_vectors,
    create_rotation_matrix,
    cross_product,
    normalize_vector,
    scale_matrix,
    scale_vector,
    sub_vector,
    to_4,
    vector_transform,
)
from payton.math.geometry import raycast_sphere_intersect
from payton.math.matrix import IDENTITY_MATRIX, Matrix, bullet_to_matrix
from payton.math.vector import Vector2D, Vector3D
from payton.scene.material import DEFAULT, NO_INDICE, NO_VERTEX_ARRAY, POINTS, SOLID, WIREFRAME, Material
from payton.scene.shader import DEFAULT_SHADER, PARTICLE_SHADER, Shader
from payton.scene.types import IList, VList

_BULLET = False
try:
    import pybullet
    import pybullet_data

    _BULLET = True
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
except ModuleNotFoundError:
    _BULLET = False


class Object:
    """Base 3D geometry object."""

    def __init__(
        self,
        name: str = "",
        visible: bool = True,
        track_motion: bool = False,
        mass: float = 0,
        force_concave: bool = False,
        heightfield: bool = False,
        **kwargs: Dict[str, Any],
    ) -> None:
        """Initialize the object.

        This is the base Object of Payton and almost all scene objects as well
        as HUD objects.

        Keyword arguments:
        name -- Name of the object
        visible -- Set if the object is visible or not.
        track_motion -- Set only if you really need to track the object's motion path.
                        This comes with an over-head.
        """
        self.children: Dict[str, Object] = {}

        # store diffeerent materials
        self.materials: Dict[str, Material] = {DEFAULT: Material()}

        self._vao: int = NO_VERTEX_ARRAY
        self._vbos: List[int] = []
        self._no_missing_vao = False

        self.name = name
        self._visible = visible
        self.matrix = deepcopy(IDENTITY_MATRIX)
        # BULLET PHYSICS
        self._bullet_id = -1
        self._bullet_shape_id = -1
        self._bullet_dynamics: Dict[str, float] = {
            'mass': mass,
        }
        self._bullet_linear_velocity: List[float] = [0, 0, 0]
        self._bullet_force_concave = force_concave
        self._bullet_heightfield = heightfield
        self._bullet_constraints: List[Dict[str, Any]] = []

        # Object vertices. Each vertex has 3 decimals (X, Y, Z). Vertices
        # are continuous. [X, Y, Z, X, Y, Z, X, Y, Z, X, ... ]
        #                  -- 1 --  -- 2 --  -- 3 --  -- 4 --

        self._vertices: List[Vector3D] = []  # Object vertex list

        # @NOTE: we have separate indices for materials but this base
        #        index list holds all indice definitions for fast access
        self._indices: IList = []  # Indices
        self._total_indices: List[int] = []
        self._normals: List[Vector3D] = []  # Vertex normals, 1 normal coordinate for 1 Vertex
        self._texcoords: List[Vector2D] = []  # Texture coordinates, 1 coordinate per Vertex
        self._vertex_colors: List[Vector3D] = []  # per-vertex colors, optional.
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
        self._model_matrix: np.ndarray = np.array([], dtype=np.float64)  # Model matrix.
        self._model_matrix_fortran: np.ndarray = np.array([], dtype=np.float64)  # Model matrix as fortran
        # Check if buffer size allocated for the object has changed.
        self._buffer_size_changed: bool = True
        self._t_buffer_size_changed: bool = True

        self.shader = DEFAULT_SHADER

        # Track object motion
        self.track_motion = track_motion
        # Motion path, stores every matrix change.
        self._motion_path: List[Matrix] = []

        if not isinstance(self, Line):
            # _motion_path_line is used to display the motion path in scene
            self._motion_path_line = Line()
        self._previous_matrix: Optional[Union[np.ndarray, List[float]]] = None

        # For raycast tests - bounding radius is the radius of the bounding
        # sphere.
        self._bounding_radius: float = 0
        self._bounding_box: VList = []
        self._selected: bool = False

        # Vertex Array Object pointer
        self._needs_update: bool = False  # Object geometry has changed.
        self._hit: bool = False

        self._absolute_vertices: Optional[List[Vector3D]] = None

    def refresh(self) -> None:
        """Refresh the object.

        Object has another memory area in the graphics card. So, for every
        geometrical change in the objects, you need to refresh the object
        so that it gets updated in the graphics card memory.
        """
        self._needs_update = True

    @property
    def material(self) -> Material:
        """Return the DEFAULT material of the object."""
        return self.materials[DEFAULT]

    @material.setter
    def material(self, mat: Material) -> None:
        """Set the DEFAULT material of the object.

        Note: This is a regular python pass-by-reference and doesn't deepcopy
        the actual material properties.

        Keyword arguments:
        mat -- Material to set
        """
        self.materials[DEFAULT] = mat

    def add_material(self, name: str, material: Material) -> None:
        """Add a material to the object.

        An object can have multiple materials for multiple polygons

        Keyword arguments:
        name -- Name of the new material
        material -- Material to set
        """
        if name in self.materials:
            raise Exception(f"Name {name} already exists")
        self.materials[name] = deepcopy(material)

    @property
    def direction(self) -> Vector3D:
        """Get the direction of the Object.

        Ideally, this is the direction that you are facing as a human.
        """
        return self.matrix[1][:3]

    @direction.setter
    def direction(self, v: Vector3D) -> None:
        """Set the direction towards a given vector.

        Note: This is a unit vector and be sure to provide a unit vector!
        Otherwise, please use `direct_to` method to point the object to an exact
        position in space.

        Keyword arguments:
        v -- Unit vector to set
        """
        self.matrix = [self.matrix[0], to_4(v, 0), self.matrix[2], self.matrix[3]]

        left = cross_product(self.matrix[1], self.matrix[2])

        self.matrix = [to_4(normalize_vector(left), 0), self.matrix[1], self.matrix[2], self.matrix[3]]

        up = cross_product(self.matrix[0], self.matrix[1])

        self.matrix = [self.matrix[0], self.matrix[1], to_4(up, 0), self.matrix[3]]
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def direct_to(self, v: Vector3D) -> None:
        """Direct object to given coordinates in space.

        Keyword arguments:
        v -- A point in space. Not a unit vector
        """
        diff = sub_vector(v, self.position)
        diff = normalize_vector(diff)
        self.direction = diff

    def rotate_around_z(self, angle: float) -> None:
        """Rotate the object around Z axis in given angle in Radians.

        Keyword arguments:
        angle -- Angle in radians
        """
        rot_matrix = create_rotation_matrix([0, 0, 1], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def rotate_around_x(self, angle: float) -> None:
        """Rotate the object around X axis in given angle in Radians.

        Keyword arguments:
        angle -- Angle in radians
        """
        rot_matrix = create_rotation_matrix([1, 0, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def rotate_around_y(self, angle: float) -> None:
        """Rotate the object around Y axis in given angle in Radians.

        Keyword arguments:
        angle -- Angle in radians
        """
        rot_matrix = create_rotation_matrix([0, 1, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def scale(self, x: float, y: float, z: float) -> None:
        """Scale the look of the object in the scene.

        This method just applies a scale matrix to the object in space.
        Does not change it's internal coordinates / vertices.

        Keyword arguments:
        x -- Scale in X direction
        y -- Scale in Y direction
        z -- Scale in Z direction
        """
        sm = scale_matrix(x, y, z)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = sm.dot(local_matrix)
        self.matrix = local_matrix.tolist()
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def scale_texture(self, x: float, y: float) -> None:
        """Scale texture coordinates of the object.

        Assuming that the object has a stretched texture on it's surface.
        By giving X and Y as > 1, you can repeat the texture on the surface.

        Keyword arguments:
        x -- Scale factor in X
        y -- Scale factor in Y
        """
        self._texcoords = [[coord[0] * x, coord[1] * y] for coord in self._texcoords]
        self.refresh()

    def select(self, start: np.ndarray, vector: np.ndarray) -> bool:
        """Check if the given ray manages to select the object. Returns boolean.

        This method does a bounding-sphere intersection test as it's the fastest
        intersection test. As a result, this method wouldn't be accurate
        in some cases especially with long and thin objects.

        This is by design and unless an equally fast and more accurate algorithm
        comes to my mind, this will remain the same.

        Keyword arguments:
        start -- Starting position of the ray vector
        vector -- Direction of the ray vector
        """
        self._selected = raycast_sphere_intersect(
            start,
            vector,
            np.array(self.matrix[3], dtype=np.float32),
            self.bounding_radius,
        )

        for obj in self.children:
            x = self.children[obj].select(start, vector)
            if not self._selected and x:
                self._selected = True

        return self._selected

    def destroy(self) -> bool:
        """Destroy the object.

        This method frees the graphics card memory/buffer used for the rendering.
        And sets the virtual pointers to zero to make them re-usable again.
        """
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
        """Step back in time if the object is in track_motion mode.

        Note: this is not reversable.

        Keyword arguments:
        steps -- Number of steps to go back in time.
        """
        steps += 1
        if not self.track_motion:
            raise Exception("track_motion should be True")
        if len(self._motion_path) < steps:
            return False

        self.matrix = self._motion_path[-steps]
        self._to_absolute.cache_clear()
        del self._motion_path[-steps + 1 :]  # noqa
        self._absolute_vertices = None
        return True

    def forward(self, distance: float) -> None:
        """Move the object forward by given distance.

        Forward is the object's direction.

        Keyword arguments:
        distance -- Distance to move forward.
        """
        diff = scale_vector(self.matrix[1], distance)
        self.matrix = [self.matrix[0], self.matrix[1], self.matrix[2], to_4(add_vectors(self.matrix[3], diff))]
        self._to_absolute.cache_clear()
        self._absolute_vertices = None

    def update_matrix(self, parent_matrix: Optional[np.ndarray] = None) -> None:
        """Update the objects matrix.

        Object's matrix is stored as Python's native List[List[float]] but
        that type is not suitable for the graphics card.

        Graphics card works with a well-aligned packed byte array whereas Python lists
        have extra bytes for stuff.

        So we turn the python's list into Numpy array, which is compatible to use with
        the graphics card.

        Keyword arguments:
        parent_matrix -- Parent matrix to multiply our initial matrix. (Optional)
        """
        # Turn matrix into numpy array. Numpy arrays are C Type arrays
        # suitable for OpenGL Pipeline
        self._model_matrix = np.array(self.matrix, dtype=np.float32)

        # When there is a parent object, child object follows parents matrix
        if parent_matrix is not None and len(parent_matrix) > 0:
            self._model_matrix = self._model_matrix.dot(parent_matrix)

        # Fortran array is used to push matrix to opengl context
        self._model_matrix_fortran = np.asfortranarray(self._model_matrix, dtype=np.float32)

    def track(self) -> bool:
        """Track the object movement if it has changed from the previous frame."""
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
        self._previous_matrix = self.matrix[3]
        return True

    @property
    def visible(self) -> bool:
        """Return if the object is visible."""
        return self._visible

    def show(self) -> None:
        """Show the object (set visible = True)."""
        self._visible = True

    def hide(self) -> None:
        """Hide the object (set visible = False)."""
        self._visible = False

    def clone(self) -> "Object":
        """Create the clone of an object - deepcopy.

        Caution! This will copy literally everything, that includes the OpenGL buffer pointers
        so changing one of the objects will result with changing the same memory area
        in graphics card!
        """
        return deepcopy(self)

    @property
    def has_missing_vao(self) -> bool:
        """Check if the object is missing any Virtual Array Objects. Return boolean."""
        if self._no_missing_vao:
            return False
        result = any(
            material._vao == NO_VERTEX_ARRAY and len(material._indices) > 1 for material in self.materials.values()
        )

        if not result:
            self._no_missing_vao = True
        return result

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
        _primitive: int = None,
    ) -> None:
        """Render cycle of the object.

        Keyword arguments:
        lit -- Is the object illuminated?
        shader -- Shader to use while rendering the object
        parent_matrix -- Parent object's matrix if this is a child object
        _primitive -- override the scene wide primitive (rendering) mode. (Point, Wire, Solid)
        """
        if not self._visible:
            return

        self.update_matrix(parent_matrix=parent_matrix)

        if self.has_missing_vao or self._needs_update:
            self.build()

        if _BULLET:
            self._build_constraints()

        self.track()

        if self._vertex_count == 0:
            # dummy object, render children and leave
            # render children
            for child in self.children:
                self.children[child].render(
                    lit,
                    shader,
                    self._model_matrix,
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
            if material.display != SOLID and shader._depth_shader:
                continue

            material.render(
                lit,
                shader,
                mode,
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
                    primitive,
                    material._vertex_count,
                    GL_UNSIGNED_INT,
                    indice_0,
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

    def set_position(self, x: float, y: float, z: float) -> None:
        """
        Set the position of the object.

        Keyword arguments:
        x -- X Position of the object
        y -- Y Position of the object
        z -- Z Position of the object
        """
        self.matrix[3] = [x, y, z, 1.0]
        self._to_absolute.cache_clear()

    @property
    def position(self) -> Vector3D:
        """Return the object's position."""
        return self.matrix[3][:3]

    @position.setter
    def position(self, pos: Vector3D) -> None:
        """Set the position of the object by a given Vector.

        In this context, the definition of the position is extended to
        support List[float] and Tuple as well to give some space to developers

        Note: This position is relative to it's parent.

        Keyword arguments:
        pos -- Position to set
        """
        fpos = to_4(pos, 1.0)
        self.matrix[3] = fpos
        self._absolute_vertices = None
        self._to_absolute.cache_clear()

    def add_child(self, name: str, obj: "Object") -> bool:
        """Add a child object to the object.

        Child objects follow their parent's movements, visibility and etc.
        Think of them as your arms and legs.

        Keyword arguments:
        name -- Name of the child object (must be unique within the same object)
        obj -- Object to add
        """
        if name in self.children:
            logging.error(f"Name {name} exists in object children")
            return False
        if not isinstance(obj, Object):
            logging.error("Object type is not valid")
            return False
        self.children[name] = obj
        return True

    @lru_cache(maxsize=10240)
    def _to_absolute(self, coordinate: Tuple) -> Vector3D:
        return vector_transform(list(coordinate), self.matrix)

    def to_absolute(self, coordinate: Vector3D) -> Vector3D:
        """
        Convert the given coordinates to absolute space coordinates. Returns the converted vector.

        Child object's coordinates are relative to it's parent.
        You can use this method if you want to know the absoltue (root) coordinates of a local coordinate.

        Keyword arguments:
        coordinate -- Coordinate to convert
        """
        return self._to_absolute(tuple(coordinate))

    def absolute_vertices(self) -> List[Vector3D]:
        """
        Convert all object vertices into absolute vertices.

        This is a cpu intesive operation and should be done with caution.
        Detailed collision detection calls this method.

        Sets `self._absolute_vertices` if None, else returns it as is.
        """
        if self._absolute_vertices is None:
            self._absolute_vertices = [self._to_absolute(tuple(v)) for v in self._vertices]

        return self._absolute_vertices

    def toggle_wireframe(self) -> None:
        """Toggle the material display type from Wireframe / Particle / Solid."""
        d = (self.material.display + 1) % 3

        for mat in self.materials.values():
            mat.display = d

        self.shader = PARTICLE_SHADER if d == POINTS else DEFAULT_SHADER
        self.refresh()
        for child in self.children.values():
            child.toggle_wireframe()

    def _calc_bounds(self) -> float:
        bmin: Optional[Vector3D] = None
        bmax: Optional[Vector3D] = None
        absolute_vertices = self.absolute_vertices()
        x = [v[0] for v in absolute_vertices]
        y = [v[1] for v in absolute_vertices]
        z = [v[2] for v in absolute_vertices]

        bmin = [0.0, 0.0, 0.0]
        bmax = [0.0, 0.0, 0.0]
        if len(x) != 0 and len(y) != 0 and len(z) != 0:
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
        """Get the bounding sphere radius."""
        if self._bounding_radius > 0:
            return self._bounding_radius
        return self._calc_bounds()

    @property
    def bounding_box(self) -> VList:
        """Get the bounding box (AABB) coordinates."""
        if self._absolute_vertices is None:
            self._calc_bounds()
        if len(self._bounding_box) > 0:
            return self._bounding_box
        self._calc_bounds()
        return self._bounding_box

    def build(self) -> bool:
        """
        Build the object so that it can be rendered in pipeline.

        Conver object vertices / normals / texture coordinates / vertex colors
        into Numpy arrays. Generate the Vertex Arrays and Buffer for the object
        in Graphics Pipeline. Load the data into graphics memory.
        Set the relevant pointer informations.
        """
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

        total_indices = np.array([], dtype=np.int32)

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
            if _BULLET:
                total_indices = np.append(total_indices, indices)  # type: ignore

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

        self._total_indices = total_indices.tolist()
        self._needs_update = False
        self._build_collision_shape()
        return True

    def _create_collision_shape(self) -> None:
        flags = None
        if self._bullet_force_concave:
            flags = pybullet.GEOM_FORCE_CONCAVE_TRIMESH
        if self._bullet_heightfield:
            flags = pybullet.GEOM_HEIGHTFIELD
        try:
            if flags is not None:
                self._bullet_shape_id = pybullet.createCollisionShape(
                    pybullet.GEOM_MESH, vertices=self._vertices, indices=self._total_indices, flags=flags
                )
            else:
                self._bullet_shape_id = pybullet.createCollisionShape(
                    pybullet.GEOM_MESH, vertices=self._vertices, indices=self._total_indices
                )
        except Exception:
            print(f"\nCould not activate Physics for {self}")

    def _build_constraints(self) -> None:
        for constraint in self._bullet_constraints:
            if constraint["active"]:
                continue
            if constraint["target"]._bullet_id == -1:
                continue
            if constraint["type"] == "p2p":
                constraint["active"] = True
                pybullet.createConstraint(
                    self._bullet_id,
                    -1,
                    constraint["target"]._bullet_id,
                    -1,
                    pybullet.JOINT_POINT2POINT,
                    [0, 0, 0],
                    constraint["local_point"],
                    constraint["target_point"],
                )

    def _build_collision_shape(self) -> None:
        if _BULLET and self.physics:
            self._create_collision_shape()
            q = Quaternion.from_matrix(self._model_matrix)
            self._bullet_id = pybullet.createMultiBody(
                baseMass=self.mass,
                baseCollisionShapeIndex=self._bullet_shape_id,
                basePosition=self.position,
                baseOrientation=q.xyzw,
            )
            if len(self._bullet_dynamics.keys()) > 0:
                pybullet.changeDynamics(self._bullet_id, -1, **self._bullet_dynamics)
            pybullet.resetBaseVelocity(self._bullet_id, linearVelocity=self._bullet_linear_velocity)

    def change_dynamics(self, **kwargs: Dict[str, Any]) -> None:
        """Apply change dynamics of bullet physics."""
        self._bullet_dynamics = {**self._bullet_dynamics, **kwargs}  # type: ignore
        if self._bullet_id != -1:
            pybullet.changeDynamics(self._bullet_id, -1, **kwargs)

    def constraint_point(self, target: "Object", local_point: Vector3D, target_point: Vector3D) -> None:
        """Create point2point contraint."""
        self._bullet_constraints.append(
            {"type": "p2p", "target": target, "local_point": local_point, "target_point": target_point, "active": False}
        )

    @property
    def linear_velocity(self) -> List[float]:
        """Return linear velocity."""
        return self._bullet_linear_velocity

    @linear_velocity.setter
    def linear_velocity(self, val: List[float]) -> None:
        """Set linear velocity."""
        self._bullet_linear_velocity = val
        if self._bullet_id != -1:
            pybullet.resetBaseVelocity(linearVelocity=self._bullet_linear_velocity)

    def to_dict(self) -> Dict[str, Any]:
        """Convert the object into a dictionary for export / debug."""
        return {
            "vertices": self._vertices,
            "normals": self._normals,
            "texcoords": self._texcoords,
            "matrix": self.matrix,
            "materials": {mat: self.materials[mat].to_dict() for mat in self.materials},
            "children": {n: self.children[n].to_dict() for n in self.children},
        }

    @property
    def physics(self) -> bool:
        """Return true if this object reacts to physics."""
        return False

    @property
    def mass(self) -> float:
        """Return the self bullet mass."""
        if self._bullet_dynamics is None:
            return 0
        return cast(float, self._bullet_dynamics.get('mass', 0))

    @mass.setter
    def mass(self, val: float) -> None:
        """Set the self bullet physics mass."""
        if self._bullet_dynamics is None:
            self._bullet_dynamics = {}
        self._bullet_dynamics['mass'] = val
        if self._bullet_id != -1:
            pybullet.changeDynamics(self._bullet_id, -1, **self._bullet_dynamics)

    def _bullet_physics(self) -> bool:
        """Responds to physics."""
        if self._bullet_id != -1 and self.mass > 0:
            pos, ori = pybullet.getBasePositionAndOrientation(self._bullet_id)
            self.matrix = bullet_to_matrix(ori)
            self.set_position(pos[0], pos[1], pos[2])
            return True

        if self.mass > 0:
            raise NotImplementedError("Physics is not defined for this object type")
        return True


class Line(Object):
    """Line Object."""

    def __init__(
        self,
        vertices: Optional[VList] = None,
        color: Optional[Vector3D] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize a Line object.

        Keyword arguments:
        vertices -- Optional Vertices information for the line
        color -- Optional color of the line material.
        """
        super().__init__(**kwargs)
        self._vertices: VList = [] if vertices is None else vertices
        self.material.color = [1.0, 1.0, 1.0] if color is None else color

        self.material.display = WIREFRAME
        self.material.lights = False
        self.build_lines()

    def toggle_wireframe(self) -> None:
        """Toggle Wireframe overwrite to disable mode change."""
        pass

    def add_material(self, name: str, material: Material) -> None:
        """@TODO Implement this later! Not urgent."""
        logging.error("Can't add materials to Line object")

    @property
    def physics(self) -> bool:
        """Physics is not applicable."""
        return False

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
        _primitive: int = None,
    ) -> None:
        """Almost same as the Object Render but with explicitly defined OpenGL primitive type."""
        super().render(lit, shader, parent_matrix, GL_LINE_STRIP)

    def append(self, vertices: VList, material: str = DEFAULT) -> None:
        """
        Append vertices to the line.

        Keyword arguments:
        vertices -- List of vertices to append
        material -- Material name to append lines into
        """
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

    def build_lines(self, vertices: Optional[VList] = None, color: Optional[Vector3D] = None) -> None:
        """
        Build lines information.

        Keyword arguments:
        vertices -- (Optional) if you want to reset the whole line vertices in one go.
        color -- Color of the DEFAULT material
        """
        if vertices is not None:
            self._vertices = vertices
        if color is not None:
            self.material.color = color
        self._vertex_count = len(self._vertices)
        self.material._vertex_count = self._vertex_count

        for i in range(self._vertex_count - 1):
            self.material._indices.append([i, i + 1])
            self._indices = self.material._indices

        for _ in range(self._vertex_count):
            self._normals.append([0, 0, 0])
            self._texcoords.append([0, 0])
        self._needs_update = True
