"""
Payton main geometry module

Geometry module holds the basic geometry shapes. They are all inherited
from `payton.scene.Object` class. They are as simple as possible.

Their face informations are generated at the initialization but display
lists are not generated until rendering.
"""

import pyrr
import math
import numpy as np
import ctypes
import logging

from OpenGL.GL import (glDeleteVertexArrays, glIsVertexArray, glBindVertexArray,
                       GL_LINE, GL_LINES, GL_FILL, GL_TRIANGLES, glPolygonMode,
                       glGenVertexArrays, glGenBuffers, GL_ARRAY_BUFFER,
                       glEnableVertexAttribArray, glVertexAttribPointer,
                       GL_FLOAT, GL_STATIC_DRAW, GL_DYNAMIC_DRAW, glBindBuffer,
                       glBufferData, glBufferSubData, GL_ELEMENT_ARRAY_BUFFER,
                       glDeleteBuffers, GL_POINT, GL_POINTS,
                       GL_FRONT_AND_BACK, glDrawElements, GL_UNSIGNED_INT)

from payton.math.geometry import raycast_sphere_intersect
from payton.math.vector import plane_normal
from payton.scene.material import WIREFRAME
from payton.scene.material import Material, SOLID, POINTS

VERTEX_BYTES = np.array([1.0, 1.0, 1.0], dtype=np.float32).nbytes


class Object(object):
    """Main Payton Object.

    This is an abstract class to define common properties between
    Mesh / Particle / Virtual objects.

    Objects are not actually built as a 3D object until rendering.
    Render function calls `build` function if needed. Build function creates
    the OpenGL Vertex Array Object. VAO is a static data so, once the object
    is built, changing vertices or indices will not take effect at the scene.

    You need to call `payton.scene.Object.build` function to refresh Vertex
    Array Object.

    Unfortunately, partial updates are not supported in this version as we
    do not know if data length is changed or not. OpenGL can replace bytes
    in an existing buffer but can not magically resize it. In case of a
    resize, a new buffer needs to be created by glBufferData.
    """
    def __init__(self, **args):
        """
        Initialize the basic object properties.

        Properties:

        *Children*: Children hash for object. Each child object follows parent
        object. They take their parent object as origin and their coordinate
        system is relative to their parent. This behaviour resembles stars,
        planets and their moons.

        *Material*: Material definitions of the object.
        *Matrix*: Matrix definition of the object. This is a 4x4 Uniform Matrix.
        But data is set as an array for easier transformations. First 4 decimals
        are "Left" vector, Second 4 are "Direction", Third 4 are "Up" and last
        four decimals are "Position" vectors.

        Args:
          track_motion: Track object motion (default: false). Object tracking is
        time independent. It just saves the object matrix for every change. Uses
        matrix position for drawing the motion path.
          static: (Default `True`) Indicates if object geometry is expected to
        be changed in the future. If object is not static, then its' vertex
        buffer object references and vertex informations will not be deleted
        to be used for future reference.
        """
        global VERTEX_BYTES
        self.children = {}
        self.material = Material()
        self.static = args.get('static', True)
        self.matrix = [[1.0, 0.0, 0.0, 0.0],
                       [0.0, 1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0, 0.0],
                       [0.0, 0.0, 0.0, 1.0]]
        # Object vertices. Each vertex has 3 decimals (X, Y, Z). Object vertices
        # are continuous. [X, Y, Z, X, Y, Z, X, Y, Z, X, ... ]
        #                  -- 1 --  -- 2 --  -- 3 --  -- 4 --
        self._vertices = []
        self._normals = [] #  Vertice normals, 1 normal coordinate for 1 Vertex.
        self._texcoords = [] # Texture coordinates, 1 coordinate for each vertex.
        self._indices = [] # Indices that make up a face.
        self._vertex_count = 0 # Number of vertices to report to OpenGL.
        # This is an optimization technique for dynamic objects where there are
        # increasing number of vertices. We allocate some buffer before-hand and
        # if we fill all of it, we resize it.
        self._buffer_size = 500 * VERTEX_BYTES
        self._model_matrix = None # Model matrix.
        self._buffer_size_changed = True

        self.track_motion = args.get('track_motion', False) # Track object motion

        self._motion_path = []
        if not isinstance(self, Line):
            self._motion_path_line = Line()
        self._previous_matrix = None

        # For raycast tests - bounding radius is the radius of the bounding
        # sphere
        self._bounding_radius = 0
        self._selected = False

        # Vertex Array Object pointer
        self._vao = None
        # I personally prefer not to delete vbos as in some cases I need to
        # refer to VBOs to update them partially. I don't want to loose
        # their reference and make things harder. I am not naming them
        # anyways.
        self._vbos = None

    def select(self, start, vector):
        """Select test for object using bounding Sphere.

        Note: this method is not 100% accurate as it is based on a rough
        assumption. Sphere area will be larger than actual object.

        If you want to have a more accurate way to handle this, try
        using raycast triangle intersect"""
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

    def render(self, proj, view, lights, parent_matrix = None):
        """
        Virtual function for rendering the object. Some objects can overwrite
        this function.

        Args:
          proj: Camera projection matrix.
          view: Camera location/view matrix.
          light_pos: Light position
          light_color: Light color

        """

        if not self._vao:
            self.build()

        self._model_matrix = np.array(self.matrix, dtype=np.float32)
        if parent_matrix is not None:
            self._model_matrix = parent_matrix.dot(self._model_matrix)

        if self.track_motion:
            if self._previous_matrix != self.matrix:
                self._motion_path.append(self.matrix)
                self._motion_path_line.append(
                    [self.matrix[3][0], self.matrix[3][1], self.matrix[3][2]])
                # Python trick here! need to .copy or it will pass reference.
                self._previous_matrix = self.matrix[3].copy()

        self.material.render(proj, view, self._model_matrix, lights)

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
        if self.track_motion:
            self._motion_path_line.render(proj,
                                          view,
                                          lights,
                                          parent_matrix)

        for child in self.children:
            self.children[child].render(proj,
                                        view,
                                        lights,
                                        self._model_matrix)


    def set_position(self, pos):
        """
        Shortcut function for explicitly modifying matrix indices.

        Basically just sets 12, 13, 14 = x, y, z
        """
        self.matrix[3][0] = pos[0]
        self.matrix[3][1] = pos[1]
        self.matrix[3][2] = pos[2]

    def add_child(self, name, obj):
        if name in self.children:
            logging.error('Name {} exists in object children'.format(name))
            return False
        if not isinstance(obj, Object):
            logging.error('Object type is not valid')
            return False
        self.children[name] = obj

    def get_position(self):
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
        if self._vao is None:
            self._vao = glGenVertexArrays(1)
            self._vbos = glGenBuffers(4)
            glBindVertexArray(self._vao)
            self.material.build_shader()
        else:
            glBindVertexArray(self._vao)


        vertices = np.array(self._vertices, dtype=np.float32)
        normals = np.array(self._normals, dtype=np.float32)
        texcoords = np.array(self._texcoords, dtype=np.float32)
        indices = np.array(self._indices, dtype=np.int32)
        for i in range(math.ceil(len(vertices) / 3)):
            d = pyrr.vector3.length(vertices[i*3:i*3+3])
            if d > self._bounding_radius:
                self._bounding_radius = d

        draw = GL_STATIC_DRAW
        if not self.static:
            draw = GL_DYNAMIC_DRAW

        # Buffer overflow, we need more space.
        if self._buffer_size < vertices.nbytes:
            global VERTEX_BYTES
            self._buffer_size = vertices.nbytes + (500 * VERTEX_BYTES)
            self._buffer_size_changed = True

        # Bind Vertices
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[0])
        glEnableVertexAttribArray(0) # shader layout location
        glVertexAttribPointer(0, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size,
                         vertices, draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, vertices.nbytes, vertices)

        # Bind Normals
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[1])
        glEnableVertexAttribArray(1) # shader layout location
        glVertexAttribPointer(1, 3, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, normals, draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, normals.nbytes, normals)

        # Bind TexCoords
        glBindBuffer(GL_ARRAY_BUFFER, self._vbos[2])
        glEnableVertexAttribArray(2) # shader layout location
        glVertexAttribPointer(2, 2, GL_FLOAT, False, 0, ctypes.c_void_p(0))
        if self._buffer_size_changed:
            glBufferData(GL_ARRAY_BUFFER, self._buffer_size, texcoords,
                         draw)
        else:
            glBufferSubData(GL_ARRAY_BUFFER, 0, texcoords.nbytes, texcoords)
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
            self._normals = []
            self._texcoords = []
            self._vertices = []
            self._indices = []

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
        cube2 = Cube(width=2.0, height=3.0, depth=5.0) # generates 2 x 3 x 5 Cube.

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

        self._vertices = [
            -width, -depth, height,
            width, -depth, height,
            -width, depth, height,
            width, depth, height,
             -width, depth, height,
             width, depth, height,
             -width, depth, -height,
             width, depth, -height,
             -width, depth, -height,
             width, depth, -height,
             -width, -depth, -height,
             width, -depth, -height,
             -width, -depth, -height,
             width, -depth, -height,
             -width, -depth, height,
             width, -depth, height,
             width, -depth, height,
             width, -depth, -height,
             width, depth, height,
             width, depth, height,
             width, depth, -height,
             -width, -depth, -height,
             -width, -depth, height,
             -width, depth, -height,
             -width, depth, -height,
             -width, -depth, height,
             -width, depth, height]

        self._normals = [
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 0.0, 1.0,
            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, -1.0,
            0.0, 0.0, -1.0,
            0.0, 0.0, -1.0,
            0.0, 0.0, -1.0,
            0.0, -1.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, -1.0, 0.0,
            0.0, -1.0, 0.0,
            1.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0.0,
            -1.0, 0.0, 0]

        self._texcoords = [
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 0.0,
            0.0, 1.0,
            1.0, 0.0,
            1.0, 0.0,
            1.0, 1.0,
            0.0, 1.0,
            1.0, 1.0,
            0.0, 0.0,
            0.0, 0.0,
            1.0, 1.0,
            1.0, 0.0]

        self._indices = [
            0, 1, 2,
            2, 1, 3,
            4, 5, 6,
            6, 5, 7,
            8, 9, 10,
            10, 9, 11,
            12, 13, 14,
            14, 13, 15,
            16, 17, 18, 19, 17, 20, 21, 22, 23, 24, 25, 26]

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

        for i in range(self.parallels + 1):
            for j in range(self.meridians + 1):
                x1 = r * math.sin(step_height * i) * math.cos(step_angle * j)
                y1 = r * math.sin(step_height * i) * math.sin(step_angle * j)
                z1 = r * math.cos(step_height * i)
                u1 = u_step * j
                v1 = 1.0 - (v_step * i)

                x2 = r * math.sin(step_height * (i + 1)) * math.cos(step_angle * j)
                y2 = r * math.sin(step_height * (i + 1)) * math.sin(step_angle * j)
                z2 = r * math.cos(step_height * (i + 1))
                u2 = u_step * j
                v2 = 1.0 - (v_step * (i + 1))

                x3 = r * math.sin(step_height * (i + 1)) * math.cos(step_angle * (j + 1))
                y3 = r * math.sin(step_height * (i + 1)) * math.sin(step_angle * (j + 1))
                z3 = r * math.cos(step_height * (i + 1))
                u3 = u_step * (j + 1)
                v3 = 1.0 - (v_step * (i + 1))

                x4 = r * math.sin(step_height * i) * math.cos(step_angle * (j + 1))
                y4 = r * math.sin(step_height * i) * math.sin(step_angle * (j + 1))
                z4 = r * math.cos(step_height * i)
                u4 = u_step * (j + 1)
                v4 = 1.0 - (v_step * i)


                normal = plane_normal([x1, y1, z1],
                                      [x2, y2, z2],
                                      [x3, y3, z3])
                self._vertices += [x1, y1, z1] # 0
                self._vertices += [x2, y2, z2] # i + 1
                self._vertices += [x3, y3, z3] # i + 2
                self._vertices += [x4, y4, z4] # i + 3
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
        self.static = False # Do not clear the vertices each time.
        self.material.display = WIREFRAME
        self.build_lines()

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
            self.build()

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
            # This is a dynamic object, destroying the object is not a good idea
            # so we just update the buffer here.
            self.build()
