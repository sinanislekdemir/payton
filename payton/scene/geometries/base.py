# pylama:ignore=C901
import pyrr
import numpy as np  # type: ignore
import ctypes
import logging
from typing import Union, List, Dict, Type, Any, Iterator, Optional

from OpenGL.GL import (
    glDeleteVertexArrays,
    glIsVertexArray,
    glBindVertexArray,
    GL_LINE,
    GL_FILL,
    GL_TRIANGLES,
    glPolygonMode,
    GL_LINE_STRIP,
    glGenVertexArrays,
    glGenBuffers,
    GL_ARRAY_BUFFER,
    glEnableVertexAttribArray,
    glVertexAttribPointer,
    GL_FLOAT,
    GL_STATIC_DRAW,
    GL_DYNAMIC_DRAW,
    glBindBuffer,
    glBufferData,
    glBufferSubData,
    GL_ELEMENT_ARRAY_BUFFER,
    glDeleteBuffers,
    GL_POINT,
    GL_POINTS,
    GL_FRONT_AND_BACK,
    glDrawElements,
    GL_UNSIGNED_INT,
)

from payton.math.geometry import raycast_sphere_intersect
from payton.math.vector import vector_transform
from payton.math.matrix import create_rotation_matrix
from payton.scene.material import Material, SOLID, POINTS, WIREFRAME
from payton.scene.shader import Shader
from payton.scene.light import Light
from payton.scene.types import VList, IList


class Object(object):
    """Main Payton Object.

    This is an abstract class to define common properties and methods between
    Mesh / Cube / Sphere/ Shape2D / PointCloud, etc.

    Objects are not actually built as 3D vertex arrays until they are rendered.
    Render function calls `build` function if needed. Build function creates
    the OpenGL Vertex Array Object. VAO is static data so, once the object
    is built, changing vertices or indices will not take effect at the scene.

    You need to call `payton.scene.geometry.Object.build` function to refresh
    Vertex Array Object.

    OpenGL can not magically extend a memory buffer, so for every new vertices
    added to the object, OpenGL needs to re-create the buffer area. This is
    not an efficitient technique if number of vertices increase in time.
    As a result Payton allocates buffer for 500 vertices in the beginning and
    uses part of it. If the object exceeds 500 vertices, a new buffer is
    created with 500 vertices more, copies existing vertices to the new
    buffer and the old buffer is deleted.

    """

    def __init__(self, **args: Any) -> None:
        """
        Initialize the basic object properties.

        Properties:
          children: Children hash for object. Each child object follows parent
                    object. They take their parent object as origin and their
                    coordinate system is relative to their parent. This
                    behaviour resembles stars, planets and their moons.
          material: Material definitions of the object.
          matrix: Matrix definition of the object. This is a 4x4 Uniform Matrix
                  but data is set as an array for easier transformations. First
                  4 decimals are "Left" vector, second 4 are "Direction", third
                  4 are "Up" and last four decimals are "Position" vectors.

        Args:
          track_motion: Track object motion (default: false). Object tracking
                        is time independent. It just saves the object matrix
                        for every change. Uses matrix position for drawing the
                        motion path.
          static: (Default `True`) Indicates if object geometry is expected
                  to be changed in the future. If object is not static, then
                  its' vertex buffer object references and vertex informations
                  will not be deleted to be used for future reference.
          name: Name of the object (optional, default '') Note that, when
                object gets added to a Scene with a name, Scene will assign
                that name to the object, overwriting any existing name of the
                object.
          visible: Is this object visible at the scene, default: `True`. To
                   hide an object, you can call `object.hide()` and to show
                   it again use: `object.show()`
        """
        self.children: Dict[str, Object] = {}
        self.material: Material = Material()
        self.static = args.get("static", True)
        self.name = args.get("name", "")
        self._visible = args.get("visible", True)
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
        self._normals: List[
            List[float]
        ] = []  # Vertex normals, 1 normal coordinate for 1 Vertex
        self._texcoords: List[
            List[float]
        ] = []  # Texture coordinates, 1 coordinate per Vertex
        self._vertex_colors: List[
            List[float]
        ] = []  # per-vertex colors, optional.
        self._has_vertex_colors: bool = False  # flag for using vertex colors

        # Vertices do not mean anything unless we define how to use them.
        # For instance, 3 vertices make a triangle or 2 vertices define a line
        # order of vertices are defined in self._indices.
        self._indices: IList = []
        self._vertex_count: int = 0  # Number of vertices to report to OpenGL.

        # This is an optimization technique for dynamic objects where there are
        # increasing number of vertices. We allocate some buffer before-hand
        # and if we fill all of it, we resize it.
        self._buffer_size: float = 500 * 12
        self._t_buffer_size: float = 500 * 8
        self._model_matrix: np.ndarray = []  # Model matrix.
        # Check if buffer size allocated for the object has changed.
        self._buffer_size_changed: bool = True
        self._t_buffer_size_changed: bool = True

        # Track object motion
        self.track_motion = args.get("track_motion", False)
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
        self._vao: int = -1
        self._needs_update: bool = False  # Object geometry has changed.
        self._hit: bool = False
        """ I personally prefer not to delete vbos as in some cases I need to
        # refer to VBOs to update them partially. I don't want to loose
        # their reference and make things harder. I am not naming them
        # anyways.
        """
        self._vbos: List[int] = []

    def refresh(self) -> None:
        """Refresh object

        Forces object to get built again
        """
        self._needs_update = True

    def yaw(self, angle: float) -> None:
        """Yaw - Rotate around Z Axis

        Args:
          angle: Angle in radians
        """
        rot_matrix = create_rotation_matrix([0, 0, 1], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()

    def rotate_around_z(self, angle: float) -> None:
        """Rotate around Z axis, alias for yaw function"""
        return self.yaw(angle)

    def rotate_around_x(self, angle: float) -> None:
        """Pitch - Rotate around X axis

        Args:
          angle: Angle in radians
        """
        rot_matrix = create_rotation_matrix([1, 0, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()

    def rotate_around_y(self, angle: float) -> None:
        """Roll - Rotate around Y Axis (Direction)

        Args:
          angle: Angle in radians
        """
        rot_matrix = create_rotation_matrix([0, 1, 0], angle)
        local_matrix = np.array(self.matrix, dtype=np.float32)
        local_matrix = rot_matrix.dot(local_matrix)
        self.matrix = local_matrix.tolist()

    def select(self, start: np.ndarray, vector: np.ndarray) -> bool:
        """Select test for object using bounding Sphere.

        This method is not 100% accurate as it is based on a rough
        assumption. Sphere area will be larger than actual object.

        If you want to have a more accurate way to handle this, try
        using raycast triangle intersect

        Args:
          start: Starting point of the ray (such as eye position)
          vector: Ray direction. This is not the end point of a line!
                  This is a unit vector showing the ray direction.
        """
        self._selected = raycast_sphere_intersect(
            start,
            vector,
            np.array(self.matrix[3], dtype=np.float32),
            self._bounding_radius,
        )

        for obj in self.children:
            x = self.children[obj].select(start, vector)
            if not self._selected and x:
                self._selected = True

        return self._selected

    def destroy(self) -> bool:
        """
        Destroy objects self.

        Returns:
            bool: `True` on successful destroy of `self`.
        """
        if self.has_vao:
            glDeleteVertexArrays(1, [self._vao])
            self._vao = -1
        return True

    def update_matrix(
        self, parent_matrix: Optional[np.ndarray] = None
    ) -> None:
        """Update matrix

        Turn object matrix into numpy array.

        Args:
          parent_matrix: Parent objects matrix
        """
        # Turn matrix into numpy array. Numpy arrays are C Type arrays
        # suitable for OpenGL Pipeline
        self._model_matrix = np.array(self.matrix, dtype=np.float32)

        # When there is a parent object, child object follows parents matrix
        if parent_matrix is not None and len(parent_matrix) > 0:
            self._model_matrix = parent_matrix.dot(self._model_matrix)

    def track(self) -> bool:
        """
        Track object motion

        Returns:
            bool: `True` on successful tracking of `self`.
        """
        if not self.track_motion:
            return False
        if self._previous_matrix == self.matrix[3]:
            return True

        # Add the new matrix to motion path records
        self._motion_path.append(self.matrix)
        # Add the matrix position to motion math line for visualisation
        if self._motion_path_line is not None:
            self._motion_path_line.append(
                [[self.matrix[3][0], self.matrix[3][1], self.matrix[3][2]]]
            )

        # Python trick here! need to .copy or it will pass reference.
        self._previous_matrix = self.matrix[3].copy()
        return True

    @property
    def has_vao(self) -> bool:
        """Check if this object has an active Vertex Array Object

        Returns:
            bool: `True` if `self` has an active Vertex Array Object.
        """
        return self._vao > -1

    @property
    def visible(self) -> bool:
        """Check if object is visible

        Returns:
          bool: `True` if visible.
        """
        return self._visible

    def show(self) -> None:
        self._visible = True

    def hide(self) -> None:
        self._visible = False

    def render(
        self,
        proj: np.ndarray,
        view: np.ndarray,
        lights: List[Light],
        parent_matrix: Optional[np.ndarray] = None,
    ) -> None:
        """
        Virtual function for rendering the object. Some objects can overwrite
        this function.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
          lights: Light objects in the scene
          parent_matrix: Parent matrix is the matrix of the parent. Parent can
                         be the scene itself or another object. In case of
                         another object, object will position itself relative
                         to its parent object.
        """
        if not self._visible:
            return

        if not self.has_vao or self._needs_update:
            self.build()

        self.update_matrix(parent_matrix=parent_matrix)
        self.track()

        if self._vertex_count == 0:
            # dummy object, render children and leave
            # render children
            for child in self.children:
                self.children[child].render(
                    proj, view, lights, self._model_matrix
                )

            return

        # Material shading mode.
        mode = None
        if self._has_vertex_colors:
            mode = Shader.PER_VERTEX_COLOR

        self.material.render(proj, view, self._model_matrix, lights, mode)

        # Actual rendering
        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            pmode = GL_LINE
            primitive = GL_LINE_STRIP
            if self.material.display == SOLID:
                pmode = GL_FILL
                primitive = GL_TRIANGLES
            if self.material.display == POINTS:
                pmode = GL_POINT
                primitive = GL_POINTS
            glPolygonMode(GL_FRONT_AND_BACK, pmode)

            glDrawElements(
                primitive,
                self._vertex_count,
                GL_UNSIGNED_INT,
                ctypes.c_void_p(0),
            )
            if pmode != GL_FILL:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)

        # End using the shader program.
        self.material.end()

        # Render motion path
        if self.track_motion:
            self._motion_path_line.render(proj, view, lights, parent_matrix)

        # render children
        for child in self.children:
            self.children[child].render(proj, view, lights, self._model_matrix)

    @property
    def position(self) -> List[float]:
        """Get position of the Object.

        Return matrix position list

        Returns:
          List[float]
        """
        return self.matrix[3][:3]

    @position.setter
    def position(self, pos: List[float]) -> None:
        """
        Shortcut function for explicitly modifying matrix indices.

        Basically just sets x, y, z of the matrix. Does not change its
        direction or up vectors.

        Args:
          pos: Position list ([x, y, z])
        """
        if len(pos) == 2:
            pos = [pos[0], pos[1], 0.0]
        self.matrix[3][0] = pos[0]
        self.matrix[3][1] = pos[1]
        self.matrix[3][2] = pos[2]

    def add_child(self, name: str, obj: Type["Object"]) -> bool:
        """Add child to this object.

        In a basic example:

            .. include:: ../../../examples/basics/05_children.py

        Args:
          name: Name of the object, must be unique within its siblings
          obj: Object. Must be an instance of `payton.scene.geometry.Object`

        Returns:
          bool: False in case of an error
        """
        if name in self.children:
            logging.error(f"Name {name} exists in object children")
            return False
        if not isinstance(obj, Object):
            logging.error("Object type is not valid")
            return False
        self.children[name] = obj
        return True

    def to_absolute(self, coordinates: List[float]) -> List[float]:
        """
        Return local coordinates (tuple, list) into absolute coordinates in
        space.

        Args:
          coordinates: List[float] (x, y, z)

        Returns:
          List[float] (x', y', z')
        """
        return vector_transform(coordinates, self.matrix)

    def absolute_vertices(self) -> Iterator[List[float]]:
        """Return a map of all local vertices as absolute coordinates.

        Imagine that object B is a child of object A. In this case, B will
        always stand (follow) relative to A. If you want to know the exact
        world coordinates of all vertices in B, this method will return them.

        **Important!** This is a costly operation so use with caution!

        Returns:
          map(List[List[float]])
        """
        return map(lambda v: self.to_absolute(v), self._vertices)

    def toggle_wireframe(self) -> None:
        """Toggle wireframe view of the Object"""
        d = self.material.display
        d += 1
        d = d % 3

        self.material.display = d
        for n in self.children:
            self.children[n].toggle_wireframe()

    def _calc_bounds(self) -> float:
        """Calculate the bounding sphere radius

        Returns:
          float
        """
        vertices = np.array(self._vertices, dtype=np.float32)

        bmin: Optional[List[float]] = None
        bmax: Optional[List[float]] = None
        for v in vertices:
            if bmin is None:
                bmin = [0, 0, 0]
                bmin[0], bmin[1], bmin[2] = v[0], v[1], v[2]
            if bmax is None:
                bmax = [0, 0, 0]
                bmax[0], bmax[1], bmax[2] = v[0], v[1], v[2]
            if v[0] < bmin[0]:
                bmin[0] = v[0]
            if v[1] < bmin[1]:
                bmin[1] = v[1]
            if v[2] < bmin[2]:
                bmin[2] = v[2]
            if v[0] > bmax[0]:
                bmax[0] = v[0]
            if v[1] > bmax[1]:
                bmax[1] = v[1]
            if v[2] > bmax[2]:
                bmax[2] = v[2]

            d = pyrr.vector3.length(v)
            if d > self._bounding_radius:
                self._bounding_radius = d
        if bmin is None:
            bmin = [0.0, 0.0, 0.0]
        if bmax is None:
            bmax = [0.0, 0.0, 0.0]
        self._bounding_box = [bmin, bmax]
        return self._bounding_radius

    @property
    def bounding_radius(self) -> float:
        """Return bounding radius

        This property function *WILL NOT* update the previously
        calculated value. If you add vertices to the object, you must call
        `payton.scene.geometry.Object.refresh` function to get radius
        and the whole object updated.

        Returns:
          float
        """

        if self._bounding_radius > 0:
            return self._bounding_radius
        return self._calc_bounds()

    def build(self) -> bool:
        """
        Build OpenGL Vertex Array for the object

        This function gets automatically called if `self._vao` does not
        exists in the first render cycle. Once the vba is built,
        geometry changes or material display mode changes will not be
        automatically effected. So, in every geometry or display mode
        change, a `build` call is necessary.

        If `self.static` is `True`, then the system assumes that another update
        call is not expected, thus frees `_normals`, `_textcoords`,
        `_vertices` and `_indices` lists to free memory.
        So in this case, calling `build` function twice will result in
        an invisible object (will not be drawn).

        Returns:
          bool
        """
        if len(self._indices) == 0:
            return False

        # If we don't have a VAO yet, we need to create one
        if not self.has_vao:
            # Generate Vertex Array
            self._vao = glGenVertexArrays(1)
            # We need 5 buffers (vertex, normal, texcoord, color, indices)
            self._vbos = glGenBuffers(5)
            glBindVertexArray(self._vao)
            # Material shader must be built when there is an active binding
            # to vertex array
            self.material.build_shader()
        else:
            # Ok, we already have vertex array object, just bind it to modify
            glBindVertexArray(self._vao)

        # Turn python arrays into C type arrays using Numpy.
        # This is required for OpenGL. Python memory model is a bit
        # different than raw memory model of C (OpenGL)
        vertices = np.array(self._vertices, dtype=np.float32).flatten()
        normals = np.array(self._normals, dtype=np.float32).flatten()
        texcoords = np.array(self._texcoords, dtype=np.float32).flatten()
        colors = np.array(self._vertex_colors, dtype=np.float32).flatten()
        indices = np.array(self._indices, dtype=np.int32).flatten()
        self._calc_bounds()

        # OpenGL allocates buffers in different mechanisms between
        # STATIC and DYNAMIC draw modes. If you select STATIC, then OpenGL
        # will assume that object buffer will not change and allocate it in a
        # more suitable way.
        draw = GL_STATIC_DRAW
        if not self.static:
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
        glEnableVertexAttribArray(0)  # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            # glBufferData creates a new data area
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, vertices, draw)
        else:
            # glBufferSubData just replaces memory area in buffer so it is
            # much more efficient way to handle things.
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.nbytes, vertices)

        # Bind Normals
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[1])
        glEnableVertexAttribArray(1)  # shader layout location
        glVertexAttribPointer(1, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, normals, draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, normals.nbytes, normals)

        # Bind TexCoords
        if len(self._texcoords) == len(self._vertices):
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[2])
            glEnableVertexAttribArray(2)  # shader layout location
            glVertexAttribPointer(2, 2, GL_FLOAT, False, 0, ctypes.c_void_p(0))
            if self._t_buffer_size_changed:
                glBufferData(
                    GL_ARRAY_BUFFER, self._t_buffer_size, texcoords, draw
                )
            else:
                glBufferSubData(
                    GL_ARRAY_BUFFER, 0, texcoords.nbytes, texcoords
                )

        # Bind Vertex Colors
        if len(self._vertex_colors) == len(self._vertices):
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[4])
            glEnableVertexAttribArray(3)  # shader layout location
            glVertexAttribPointer(3, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
            self._has_vertex_colors = True
            if self._buffer_size_changed:
                glBufferData(GL_ARRAY_BUFFER, self._buffer_size, colors, draw)
            else:
                glBufferSubData(GL_ARRAY_BUFFER, 0, colors.nbytes, colors)

        self._buffer_size_changed = False
        self._t_buffer_size_changed = False

        # Bind Indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self._vbos[3])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, draw)
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        if self.static:
            # we can clear this data to free some more memory
            glDeleteBuffers(4, self._vbos)
            self._vbos = []

        self._needs_update = False

        return True


class Line(Object):
    """Line object

    Exceptionally, due to hard reference for motion path feature, this class
    is defined in base.py instead of line.py
    """

    def __init__(self, **args: Any) -> None:
        """Iniitalize line

        Args:
          vertices: Vertices array for list of points.
          color: Color of the line

        Example use case:

            .. include:: ../../../examples/basics/17_line.py
        """
        super().__init__(**args)
        self._vertices: VList = args.get("vertices", [])
        self.material.color = args.get("color", [1.0, 1.0, 1.0])

        self.static: bool = False  # Do not clear the vertices each time.
        self.material.display = WIREFRAME
        self.build_lines()

    def toggle_wireframe(self) -> None:
        """Toggle Wireframe overwrite to disable mode change"""
        pass

    def append(self, vertices: VList) -> None:
        """Append vertex or vertices to line.

        Args:
          vertices: Vertex array of points.
        """

        diff = len(vertices)  # Number of vertices added
        last_index = len(self._vertices)
        self._vertices += vertices

        self._texcoords += [[0, 0]] * diff
        self._normals += [[0, 0, 0]] * diff
        self._vertex_count = len(self._vertices)
        indices = list(map(lambda x: x + last_index, range(diff)))
        self._indices.extend([indices])

        if self.has_vao:
            self._needs_update = True

    def build_lines(
        self,
        vertices: Optional[VList] = None,
        color: Optional[List[float]] = None,
    ) -> None:
        """Build lines

        Build line vertex array object.
        """
        if vertices is not None:
            self._vertices = vertices
        if color is not None:
            self.material.color = color
        self._vertex_count = len(self._vertices)
        for i in range(self._vertex_count - 1):
            self._indices.append([i, i + 1])

        for i in range(self._vertex_count):
            self._normals.append([0, 0, 0])
            self._texcoords.append([0, 0])

        if self.has_vao:
            # This is a dynamic object, destroying the object is not a good
            # idea so we just update the buffer here.
            self.build()