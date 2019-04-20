"""
Payton main geometry module

Geometry module holds the basic geometry shapes. They are all inherited
from `payton.scene.geometr.yObject` class. They are as simple as possible.

Their face informations are generated at the initialization but vertex array
object is not generated until it arrives in render pipeline.
"""

import pyrr
import math
import numpy as np
import ctypes
import logging

from OpenGL.GL import (glDeleteVertexArrays, glIsVertexArray,
                       glBindVertexArray, GL_LINE, GL_LINES, GL_FILL,
                       GL_TRIANGLES, glPolygonMode,
                       glGenVertexArrays, glGenBuffers, GL_ARRAY_BUFFER,
                       glEnableVertexAttribArray, glVertexAttribPointer,
                       GL_FLOAT, GL_STATIC_DRAW, GL_DYNAMIC_DRAW, glBindBuffer,
                       glBufferData, glBufferSubData, GL_ELEMENT_ARRAY_BUFFER,
                       glDeleteBuffers, GL_POINT, GL_POINTS,
                       GL_FRONT_AND_BACK, glDrawElements, GL_UNSIGNED_INT)

from payton.math.geometry import raycast_sphere_intersect
from payton.math.vector import plane_normal
from payton.scene.material import Material, SOLID, POINTS, WIREFRAME
from payton.scene.shader import Shader

VERTEX_BYTES = np.array([1.0, 1.0, 1.0], dtype=np.float32).nbytes


class Object(object):
    """Main Payton Object.

    This is an abstract class to define common properties and methods between
    Mesh / Particle / Virtual objects.

    Objects are not actually built as 3D vertex arrays until render.
    Render function calls `build` function if needed. Build function creates
    the OpenGL Vertex Array Object. VAO is a static data so, once the object
    is built, changing vertices or indices will not take effect at the scene.

    You need to call `payton.scene.geometry.Object.build` function to refresh
    Vertex Array Object.

    OpenGL can not magically extend a memory buffer, so for every new vertices
    added to the object, OpenGL needs to re-create the buffer area. This is
    not an efficitient technique if number of vertices increase in time.
    As a result Payton allocates buffer for 500 vertices in the beginning and
    uses a part of it. If object exceeds 500 vertices, a new buffer is created
    with 500 more vertices, copies existing vertices to new and old buffer
    is deleted.

    """
    def __init__(self, **args):
        """
        Initialize the basic object properties.

        Properties:
          children: Children hash for object. Each child object follows parent
        object. They take their parent object as origin and their coordinate
        system is relative to their parent. This behaviour resembles stars,
        planets and their moons.
          material: Material definitions of the object.
          matrix: Matrix definition of the object. This is a 4x4 Uniform Matrix
        But data is set as an array for easier transformations. First 4
        decimals are "Left" vector, Second 4 are "Direction", Third 4 are "Up"
        and last four decimals are "Position" vectors.

        Args:
          track_motion: Track object motion (default: false). Object tracking
        is time independent. It just saves the object matrix for every change.
        Uses matrix position for drawing the motion path.
          static: (Default `True`) Indicates if object geometry is expected
        to be changed in the future. If object is not static, then its'
        vertex buffer object references and vertex informations will not be
        deleted to be used for future reference.
        """
        global VERTEX_BYTES
        self.children = {}
        self.material = Material()
        self.static = args.get('static', True)
        self.matrix = [[1.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]]
        # Object vertices. Each vertex has 3 decimals (X, Y, Z). Vertices
        # are continuous. [X, Y, Z, X, Y, Z, X, Y, Z, X, ... ]
        #                  -- 1 --  -- 2 --  -- 3 --  -- 4 --

        self._vertices = []  # Object vertex list
        self._normals = []  # Vertex normals, 1 normal coordinate for 1 Vertex
        self._texcoords = []  # Texture coordinates, 1 coordinate per Vertex
        self._vertex_colors = []  # per-vertex colors, optional.
        self._has_vertex_colors = False  # flag for using vertex colors

        # Vertices do not mean anything unless we define how to use them.
        # For instance, 3 vertices make a triangle or 2 vertices define a line
        # order of vertices are defined in self._indices.
        self._indices = []
        self._vertex_count = 0  # Number of vertices to report to OpenGL.

        # This is an optimization technique for dynamic objects where there are
        # increasing number of vertices. We allocate some buffer before-hand
        # and if we fill all of it, we resize it.
        self._buffer_size = 500 * VERTEX_BYTES
        self._model_matrix = None  # Model matrix.
        # Check if buffer size allocated for the object has changed.
        self._buffer_size_changed = True

        # Track object motion
        self.track_motion = args.get('track_motion', False)
        # Motion path, stores every matrix change.
        self._motion_path = []
        if not isinstance(self, Line):
            # _motion_path_line is used to display the motion path in scene
            self._motion_path_line = Line()
        self._previous_matrix = None

        # For raycast tests - bounding radius is the radius of the bounding
        # sphere.
        self._bounding_radius = 0
        self._selected = False

        # Vertex Array Object pointer
        self._vao = None
        self._needs_update = False  # Object geometry has changed.
        self._hit = False
        # I personally prefer not to delete vbos as in some cases I need to
        # refer to VBOs to update them partially. I don't want to loose
        # their reference and make things harder. I am not naming them
        # anyways.
        self._vbos = None

    def refresh(self):
        """Refresh object

        Forces object to get built again
        """
        self._needs_update = True

    def select(self, start, vector):
        """Select test for object using bounding Sphere.

        Note: this method is not 100% accurate as it is based on a rough
        assumption. Sphere area will be larger than actual object.

        If you want to have a more accurate way to handle this, try
        using raycast triangle intersect

        Args:
          start: Starting point of the ray (such as eye position)
          vector: Ray direction. This is not the end point of a line! This is
        a unit vector showing the ray direction.
        """
        self._selected = raycast_sphere_intersect(start,
                                                  vector,
                                                  np.array(self.matrix[3],
                                                           dtype=np.float32),
                                                  self._bounding_radius)

        for obj in self.children:
            x = self.children[obj].select(start, vector)
            if not self._selected and x:
                self._selected = True

        return self._selected

    def destroy(self):
        """
        Destroy objects self
        """
        if self._vao:
            glDeleteVertexArrays(1, [self._vao])
            self._vao = None
        return True

    def update_matrix(self, parent_matrix=None):
        """Update matrix

        Turn object matrix into numpy array.

        Args:
          parent_matrix: Parent objects matrix
        """
        # Turn matrix into numpy array. Numpy arrays are C Type arrays
        # suitable for OpenGL Pipeline
        self._model_matrix = np.array(self.matrix, dtype=np.float32)

        # When there is a parent object, child object follows parents matrix
        if parent_matrix is not None:
            self._model_matrix = parent_matrix.dot(self._model_matrix)

    def render(self, proj, view, lights, parent_matrix=None):
        """
        Virtual function for rendering the object. Some objects can overwrite
        this function.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
          lights: Light objects in the scene
          parent_matrix: Parent matrix is the matrix of the parent. Parent can
        be the scene itself or another object. In case of another object,
        object will position itself relative to its parent object.
        """

        if not self._vao or self._needs_update:
            self.build()

        if self._vertex_count == 0:
            return

        self.update_matrix(parent_matrix=parent_matrix)

        if self.track_motion:
            # Has the matrix changed from previous matrix?
            if self._previous_matrix != self.matrix:
                # Add the new matrix to motion path records
                self._motion_path.append(self.matrix)
                # Add the matrix position to motion math line for visualisation
                self._motion_path_line.append(
                    [self.matrix[3][0], self.matrix[3][1], self.matrix[3][2]])

                # Python trick here! need to .copy or it will pass reference.
                self._previous_matrix = self.matrix[3].copy()

        # Material shading mode.
        mode = None
        if self._has_vertex_colors:
            mode = Shader.PER_VERTEX_COLOR

        self.material.render(proj, view, self._model_matrix, lights, mode)

        # Actual rendering
        if glIsVertexArray(self._vao):
            glBindVertexArray(self._vao)
            pmode = GL_LINE
            primitive = GL_LINES
            if self.material.display == SOLID:
                pmode = GL_FILL
                primitive = GL_TRIANGLES
            if self.material.display == POINTS:
                pmode = GL_POINT
                primitive = GL_POINTS
            glPolygonMode(GL_FRONT_AND_BACK, pmode)

            glDrawElements(primitive, self._vertex_count,
                           GL_UNSIGNED_INT, ctypes.c_void_p(0))
            if pmode != GL_FILL:
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL)
            glBindVertexArray(0)

        # End using the shader program.
        self.material.end()
        # Render motion path
        if self.track_motion:
            self._motion_path_line.render(proj,
                                          view,
                                          lights,
                                          parent_matrix)

        # render children
        for child in self.children:
            self.children[child].render(proj,
                                        view,
                                        lights,
                                        self._model_matrix)

    def set_position(self, pos):
        """
        Shortcut function for explicitly modifying matrix indices.

        Basically just sets x, y, z of the matrix. Does not change its
        direction or up vectors.

        Args:
          pos: Position list ([x, y, z])
        """
        self.matrix[3][0] = pos[0]
        self.matrix[3][1] = pos[1]
        self.matrix[3][2] = pos[2]

    def add_child(self, name, obj):
        """Add child to this object.

        In a basic example:

            from payton.scene import Scene
            from payton.scene.geometry import Sphere

            scene = Scene()
            earth = Sphere(radius=3)
            moon = Sphere()
            moon.set_position([2, 0, 0])  # Relative to earth
            earth.add_child('moon', moon)
            scene.run()

        Args:
          name: Name of the object, must be unique within its siblings
          obj: Object. Must be an instance of `payton.scene.geometry.Object`

        Return:
          bool: False in case of an error
        """
        if name in self.children:
            logging.error('Name {} exists in object children'.format(name))
            return False
        if not isinstance(obj, Object):
            logging.error('Object type is not valid')
            return False
        self.children[name] = obj

    def get_position(self):
        """Get position of the Object.

        Return matrix position list
        """
        return self.matrix[3][:3]

    def to_absolute(self, coordinates):
        """
        Return local coordinates (tuple, list) into absolute coordinates in
        space.
        """
        pass

    def to_local(self, coordinates):
        """
        Return absolute coordinates (tuple, list) into local coordinates
        """
        pass

    def toggle_wireframe(self):
        d = self.material.display
        d += 1
        d = d % 3

        self.material.display = d
        for n in self.children:
            self.children[n].toggle_wireframe()

    def _calc_bounding_radius(self):
        # Calculate the bounding sphere radius
        vertices = np.array(self._vertices, dtype=np.float32)
        for i in range(math.ceil(len(vertices) / 3)):
            d = pyrr.vector3.length(vertices[i*3:i*3+3])
            if d > self._bounding_radius:
                self._bounding_radius = d
        return self._bounding_radius

    @property
    def bounding_radius(self):
        """Return bounding radius

        Note: This property function *WILL NOT* update the previously
        calculated value. If you add vertices to the object, you must call
        `payton.scene.geometry.Object.refresh` function to get radius
        and the whole object updated.
        """

        if self._bounding_radius > 0:
            return self._bounding_radius
        return self._calc_bounding_radius()

    def build(self):
        """
        Build OpenGL Vertex Array for the object
        This function gets automatically called if `self._vao` does not
        exists in the first render cycle. Once the vba is built,
        geometry changes or material display mode changes will not be
        automatically effected. So, in every geometry or display mode
        change, a `build` call is necessary.

        if `self.static` is `True`, then system assumes that another update
        call is not expected, thus frees `_normals', `_textcoords`,
        `_vertices` and `_indices` lists to free memory.
        So in this case, calling `build` function twice will result with
        an invisible object (will not be drawn)
        """
        if len(self._indices) == 0:
            return

        # If we don't have a VAO yet, we need to create one
        if self._vao is None:
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
        vertices = np.array(self._vertices, dtype=np.float32)
        normals = np.array(self._normals, dtype=np.float32)
        texcoords = np.array(self._texcoords, dtype=np.float32)
        colors = np.array(self._vertex_colors, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)
        self._calc_bounding_radius()

        # OpenGL allocates buffers in different mechanisms between
        # STATIC and DYNAMIC draw modes. If you select STATIC, then OpenGL
        # will assume that object buffer will not change and allocate it in a
        # more suitable way.
        draw = GL_STATIC_DRAW
        if not self.static:
            draw = GL_DYNAMIC_DRAW

        # Buffer overflow, we need more space.
        # For dynamic objects, we allocate +500 vertices at all times.
        if self._buffer_size < vertices.nbytes:
            global VERTEX_BYTES
            self._buffer_size = vertices.nbytes + (500 * VERTEX_BYTES)
            self._buffer_size_changed = True

        # Bind Vertices
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[0])
        glEnableVertexAttribArray(0)  # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            # glBufferData creates a new data area
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size,
                         vertices, draw)
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
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[2])
        glEnableVertexAttribArray(2)  # shader layout location
        glVertexAttribPointer(2, 2, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, texcoords,
                         draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, texcoords.nbytes, texcoords)

        # Bind Vertex Colors
        if len(self._vertex_colors) == len(self._vertices):
            glBindBuffer(GL_ARRAY_BUFFER, self._vbos[4])
            glEnableVertexAttribArray(3)  # shader layout location
            glVertexAttribPointer(3, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
            self._has_vertex_colors = True
            if self._buffer_size_changed:
                glBufferData(GL_ARRAY_BUFFER, self._buffer_size, colors,
                             draw)
            else:
                glBufferSubData(GL_ARRAY_BUFFER, 0, colors.nbytes,
                                colors)

        self._buffer_size_changed = False

        # Bind Indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self._vbos[3])
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, draw)
        self._vertex_count = len(indices)

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)

        if self.static:
            # we can clear this data to free some more memory
            glDeleteBuffers(4, self._vbos)
            self._vbos = None

        return True


class Cube(Object):
    """
    Cube object

    This is a simple Cube object with width, height and depth.

    Cube object use case:

        from payton.scene import Scene
        from payton.scene.geometry import Cube

        my_scene = Scene()
        cube1 = Cube() # generates 1 x 1 x 1 Cube.
        cube2 = Cube(width=2.0, height=3.0, depth=5.0) # generates 2x3x5 Cube.

        cube1.matrix[12] = -3.0
        cube1.matrix[13] = -2.0

        my_scene.add_object(cube1)
        my_scene.add_object(cube2)
        my_scene.run()


    """
    def __init__(self, **args):
        """Initialize Cube

        Args:
          width: Width of the cube (size X)
          depth: Depth of the cube (size Y)
          height: Height of the cube (size Z)
        """
        super(Cube, self).__init__(**args)
        width = args.get('width', 1.0) * 0.5
        depth = args.get('depth', 1.0) * 0.5
        height = args.get('height', 1.0) * 0.5

        self._vertices = [-width, -depth, height, width, -depth, height,
                          -width, depth, height, width, depth, height,
                          -width, depth, height, width, depth, height,
                          -width, depth, -height, width, depth, -height,
                          -width, depth, -height, width, depth, -height,
                          -width, -depth, -height, width, -depth, -height,
                          -width, -depth, -height, width, -depth, -height,
                          -width, -depth, height, width, -depth, height,
                          width, -depth, height, width, -depth, -height,
                          width, depth, height, width, depth, height,
                          width, depth, -height, -width, -depth, -height,
                          -width, -depth, height, -width, depth, -height,
                          -width, depth, -height, -width, -depth, height,
                          -width, depth, height]

        self._normals = [0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0,
                         0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0,
                         0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0,
                         0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0,
                         0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0,
                         0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                         1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                         -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                         -1.0, 0.0, 0.0, -1.0, 0.0, 0.0, -1.0, 0.0, 0]

        self._texcoords = [0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0,
                           0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                           1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0,
                           1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 0.0,
                           1.0, 1.0, 0.0, 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                           1.0, 1.0, 1.0, 0.0]

        self._indices = [0, 1, 2, 2, 1, 3, 4, 5, 6, 6, 5, 7, 8, 9, 10,
                         10, 9, 11, 12, 13, 14, 14, 13, 15, 16, 17, 18,
                         19, 17, 20, 21, 22, 23, 24, 25, 26]

        return None


class Sphere(Object):
    """
    Sphere object.

    This object is generated using basic Spherical coordinates.
    Beware of using high values for parallels and meridians. You might end up
    with excessive number of vertices to render and a performance trouble.

    Parameters:

    - `radius` default: `0.5`
    - `parallels` default: `12`
    - `meridians` default: `12`

    Sphere object use case

        from payton.scene import Scene
        from payton.scene.geometry import Sphere

        my_scene = Scene()
        sun = Sphere(radius=10.0)

        earth = Sphere(radius=0.3)
        sun.children.append(earth)

        sun.material.color = [1.0, 0.3, 0.3, 1.0]
        earth.matrix[12] = 30.0 # 12th item in matrix is X coordinates in space
        earth.matrix[13] = 5.0 # 13th item in matrix is Y coordinates in space
        earth.material.color = [0.2, 0.2, 1.0, 1.0]

        my_scene.add_object(sun)
        my_scene.run()

    """
    def __init__(self, **args):
        super(Sphere, self).__init__(**args)
        self.radius = args.get('radius', 0.5)
        self.parallels = args.get('parallels', 12)
        self.meridians = args.get('meridians', 12)
        self.build_sphere()

    def build_sphere(self):
        """
        Generate the sphere
        """
        r = self.radius
        # step angle is the rotational angle to build the sphere
        step_angle = math.radians(360.0 / self.meridians)
        # step height is the arc in height
        step_height = math.radians(180.0 / self.parallels)
        indices = 0
        u_step = 1.0 / self.meridians
        v_step = 1.0 / self.parallels

        for i in range(self.parallels):
            for j in range(self.meridians):
                x1 = r * math.sin(step_height * i) * math.cos(step_angle * j)
                y1 = r * math.sin(step_height * i) * math.sin(step_angle * j)
                z1 = r * math.cos(step_height * i)
                u1 = u_step * j
                v1 = 1.0 - (v_step * i)

                x2 = (r * math.sin(step_height * (i + 1))
                      * math.cos(step_angle * j))
                y2 = (r * math.sin(step_height * (i + 1))
                      * math.sin(step_angle * j))
                z2 = r * math.cos(step_height * (i + 1))
                u2 = u_step * j
                v2 = 1.0 - (v_step * (i + 1))

                x3 = (r * math.sin(step_height * (i + 1))
                      * math.cos(step_angle * (j + 1)))
                y3 = (r * math.sin(step_height * (i + 1))
                      * math.sin(step_angle * (j + 1)))
                z3 = r * math.cos(step_height * (i + 1))
                u3 = u_step * (j + 1)
                v3 = 1.0 - (v_step * (i + 1))

                x4 = (r * math.sin(step_height * i)
                      * math.cos(step_angle * (j + 1)))
                y4 = (r * math.sin(step_height * i)
                      * math.sin(step_angle * (j + 1)))
                z4 = r * math.cos(step_height * i)
                u4 = u_step * (j + 1)
                v4 = 1.0 - (v_step * i)

                normal = plane_normal([x1, y1, z1],
                                      [x2, y2, z2],
                                      [x3, y3, z3])
                self._vertices += [x1, y1, z1]  # 0
                self._vertices += [x2, y2, z2]  # i + 1
                self._vertices += [x3, y3, z3]  # i + 2
                self._vertices += [x4, y4, z4]  # i + 3
                self._texcoords.extend([u1, v1, u2, v2, u3, v3, u4, v4])
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._indices += [indices, indices+1, indices+2]
                self._indices += [indices, indices+2, indices+3]
                indices += 4
        return True


class Line(Object):
    """Line object
    """
    def __init__(self, **args):
        """Iniitalize line

        Args:
          vertices: Vertices array for list of points.
          color: Color of the line
        """
        super(Line, self).__init__(**args)
        self._vertices = args.get('vertices', [])
        self.material.color = args.get('color', [1.0, 1.0, 1.0])

        self.visible = True
        self.static = False  # Do not clear the vertices each time.
        self.material.display = WIREFRAME
        self.build_lines()

    def toggle_wireframe(self):
        """Toggle Wireframe overwrite to disable mode change"""
        pass

    def append(self, vertices):
        """Append vertex or vertices to line.

        Args:
          vertices: Vertex array of points.
        """
        if len(vertices) % 3 != 0:
            logging.error('Number of vertices must be a power of 3')
            return False
        diff = math.ceil(len(vertices) / 3.0)  # Number of vertices added
        self._vertices.extend(vertices)
        self._texcoords.extend([0, 0] * diff)
        self._normals.extend([0, 0, 0] * diff)

        self._vertex_count = math.ceil(len(self._vertices) / 3.0)

        if self._vertex_count == 1:
            # Not enough to draw any line.
            return True

        if self._vertex_count == 2:
            self._indices.extend([0, 1])
            if self._vao is not None:
                self.build()
            return True

        last = self._indices[-1]

        for i in range(diff):
            self._indices.extend([last+i, last+i+1])

        if self._vao is not None:
            self._needs_update = True

    def build_lines(self, vertices=None, color=None):
        """Build lines

        Build line vertex array object.
        """
        if vertices is not None:
            self._vertices = vertices
        if color is not None:
            self.material.color = color
        self._vertex_count = math.ceil(len(self._vertices) / 3.0)
        for i in range(self._vertex_count - 1):
            self._indices += [i, i+1]

        for i in range(self._vertex_count):
            self._normals += [0, 0, 0]
            self._texcoords += [0, 0]

        if self._vao is not None:
            # This is a dynamic object, destroying the object is not a good
            # idea so we just update the buffer here.
            self.build()


class Mesh(Object):
    """Mesh Object

    Mesh is almost like the Object except with some extra methods to make
    things easier. If you want to have custom geometries/shapes, it is
    better to extend `payton.scene.geometry.Mesh` instead of
    `payton.scene.geometry.Object`. Because Mesh will give you better
    and easier constructing capabilities such as adding triangles on the fly
    or sub-division or cutting and so forth. It is a way of designing objects
    by code.
    """
    def add_triangle(self, vertices, normals=None, texcoords=None,
                     colors=None):
        """Add triangle to Mesh

        Args:
          vertices: Vertices of the triangle. This is required. Ex:
        `[[0, 0, 0], [2, 0, 0], [1, 1, 0]]`
          normals: Normals of the triangle. _(When left as None, Payton will
        calculate the surface normal based on vertices and assign it per
        given vertex.)_
          texcoords: Texture UV coordinates.
          colors: Per vertex colors (optional)

        Example:

            from payton.scene import Scene
            from payton.scene.geometry import Mesh


            scene = Scene()
            mesh = Mesh()
            mesh.add_triangle([[-2, 0, 0],
                               [2, 0, 0],
                               [0, 2, 0]], texcoords=[[0, 0],
                                                      [1, 0],
                                                      [1, 1]],
                              colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]])

            scene.add_object('mesh', mesh)
            scene.run()
        """
        if len(vertices) != 3:
            logging.error('A triangle must have 3 vertices')
            return False
        if normals is not None and len(normals) != 3:
            logging.error('There must be one normal per vertex')
            return False
        if texcoords is not None and len(texcoords) != 3:
            logging.error('There must be one texcoord per vertex')
            return False
        if normals is None:
            v1, v2, v3 = vertices[0], vertices[1], vertices[2]
            normal = plane_normal(v1, v2, v3)
            normals = [normal, normal, normal]
        if texcoords is None:
            texcoords = [[0, 0], [1, 0], [1, 1]]
        if colors:
            self._vertex_colors.extend(colors[0])
            self._vertex_colors.extend(colors[1])
            self._vertex_colors.extend(colors[2])

        self._vertices.extend(vertices[0])
        self._vertices.extend(vertices[1])
        self._vertices.extend(vertices[2])
        i = len(self._indices)
        self._indices.extend([i, i+1, i+2])
        self._normals.extend(normals[0])
        self._normals.extend(normals[1])
        self._normals.extend(normals[2])
        self._texcoords.extend(texcoords[0])
        self._texcoords.extend(texcoords[1])
        self._texcoords.extend(texcoords[2])


class PointCloud(Object):
    """Point cloud
    """
    def __init__(self, **args):
        super(PointCloud, self).__init__(**args)
        self._vertices = args.get('vertices', [])
        self._vertex_colors = args.get('colors', [])
        self.material.display = POINTS
        self.static = False

    def toggle_wireframe(self):
        """Toggle wireframe overwrite to disable mode change"""
        pass

    def add(self, vertices, colors=None):
        """Add a point to the cloud

        Args:
          vertices: Vertices to add
          colors: Colors of the vertices in the same order. (Optional)
        """
        i = len(self._indices)
        for vertex in vertices:
            self._vertices.extend(vertex)
            self._indices.append(i)
            i += 1

        if colors is not None:
            if len(colors) != len(vertices):
                logging.error('len(colors) != len(vertices)')
                return
            for color in colors:
                self._vertex_colors.extend(color)

        self._needs_update = True
