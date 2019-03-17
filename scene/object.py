"""
Payton Object Module
"""
from OpenGL.GL import (glDeleteLists, glPushMatrix, glPopMatrix, glMultMatrixf,
                       glCallList, glGenLists, glNewList, glBegin, glEnd,
                       glNormal3f, glTexCoord2f, glVertex3f, glEndList,
                       GL_COMPILE, GL_QUADS, GL_LINE_STRIP)

from payton.scene.material import Material, SOLID


class Object(object):
    """
    Main Payton Object.

    This is an abstract class to define common properties between
    Mesh / Particle / Virtual objects.

    Objects are not actually built as a 3D object unless they are
    being rendered. Render function calls `build` function
    which then creates the opengl display list of the object.
    Display list is a static data so, once the object is built,
    changing vertices or indices won't help with the geometry
    of the object.
    
    To change the geometry, you need to call `build` function once
    more. (Also, display mode changes need a rebuild)
    """
    def __init__(self):
        """
        Initialize the basic object properties here.
        This is important as to keep track of all self object
        properties and avoid any assumptions on whether an object
        property is set or not.

        So if anyhow, you are adding an object property, please
        do not forget to define its default here.

        Each object can have several children.
        And each child can have its own children as well.
        So we have an object tree, which is suitable for complex
        systems like a solar system.
        A Star has planets and each planet can have moons or satellites.
        Each child in the list has their own local object coordinate system.
        To get the absolute coordinates of a local coordinate in the universe
        you can use to_absolute function.
        """
        self.children = []
        self.material = Material()
        self.static = True
        self.matrix = [1.0, 0.0, 0.0, 0.0,
                       0.0, 1.0, 0.0, 0.0,
                       0.0, 0.0, 1.0, 0.0,
                       0.0, 0.0, 0.0, 1.0]
        self._vertices = []
        self._normals = []
        self._texcoords = []
        self._indices = []

        self._list = None

    def destroy(self):
        """
        Destroy objects self
        """
        glDeleteLists(self._list, 1)

    def render(self):
        """
        Virtual function for rendering the object.
        """

        if not self._list:
            self.build()

        glPushMatrix()
        glMultMatrixf(self.matrix)
        self.material.begin_render()
        glCallList(self._list)
        self.material.end_render()
        for k in self.children:
            k.render()
        glPopMatrix()

    def set_position(self, x, y, z):
        """
        Shortcut function for explicitly modifying matrix indices.

        Basically just sets 12, 13, 14 = x, y, z
        """
        self.matrix[12], self.matrix[13], self.matrix[14] = x, y, z
        
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

    def build(self):
        """
        Build OpenGL Display List
        This function gets automatically called if display list does not
        exists in the first render cycle. Once the display list is built,
        geometry changes or material display mode changes will not be
        automatically effected. So, in every geometry or display mode
        change, a build call is necessary.

        if self.static is True, then system assumes that another build
        call is not expected, thus frees `_normals', `_textcoords`,
        `_vertices` and `_indices` lists to free memory.
        So in this case, calling `build` function twice will result with
        an invisible object (will not be drawn)

        """
        self._list = glGenLists(1)
        glNewList(self._list, GL_COMPILE)
        # Draw things here
        for quad in self._indices:
            if self.material.display == SOLID:
                glBegin(GL_QUADS)
            else:
                glBegin(GL_LINE_STRIP)

            for index in quad:
                vertex = index[0]
                texcoord = index[1]
                normal = index[2]
                if normal > -1:
                    glNormal3f(self._normals[normal][0],
                               self._normals[normal][1],
                               self._normals[normal][2])

                if texcoord > -1:
                    glTexCoord2f(self._texcoords[texcoord][0],
                                 self._texcoords[texcoord][1])

                glVertex3f(self._vertices[vertex][0],
                           self._vertices[vertex][1],
                           self._vertices[vertex][2])
            glEnd()
        glEndList()

        if self.static:
            # we can clear this data to free some more memory
            self._normals = []
            self._texcoords = []
            self._vertices = []
            self._indices = []

        return True
