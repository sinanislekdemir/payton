"""
Grid module

Grid is a basic layout *(virtual ground)* for the entire scene which centers
the origin of the scene (0, 0, 0) and can not be moved.

Grid size can be adjusted. Grid is a perfect way to visually see the movement
and positions of objects in space.
"""
from OpenGL.GL import (glDisable, glColor3f, glBegin, glEnd, glVertex3f,
                       glColor, GL_LIGHTING, GL_LINES, glEnable)
from payton.scene.object import Object


class Grid(Object):
    """
    Properties of Grid:

    - `grid_size`: Default `10`
    - `grid_spacing`: Default `1.0`
    - `visible`: Default `True`

    Example usage:

        from payton.scene.scene import Scene


        my_scene = Scene()
        my_scene.grid.grid_size = 20 #  we need more space
        my_scene.run()
    """
    def __init__(self, **args):
        """
        Initialize the grid.  
        """
        super().__init__()
        self.grid_size = args.get('grid_size', 10)
        self.grid_spacing = args.get('grid_spacing', 1.0)
        self.visible = args.get('visible', True)
        self._grid_start = -(self.grid_size / 2.0)

    def build(self):
        self._grid_start = ((self.grid_size * self.grid_spacing) / 2.0) * -1.0
        return 0

    def render(self):
        if not self.visible:
            return False
        glDisable(GL_LIGHTING)
        for i in range(self.grid_size + 1):
            if i == self.grid_size / 2:
                continue
            glColor3f(0.4, 0.4, 0.4)
            glBegin(GL_LINES)
            glVertex3f(self._grid_start + (i * self.grid_spacing),
                       self._grid_start, 0)
            glVertex3f(self._grid_start + (i * self.grid_spacing),
                       self._grid_start * -1.0, 0)
            glEnd()

            glBegin(GL_LINES)
            glVertex3f(self._grid_start,
                       self._grid_start + (i * self.grid_spacing), 0)
            glVertex3f(self._grid_start * -1.0,
                       self._grid_start + (i * self.grid_spacing), 0)
            glEnd()

        glColor(1.0, 0.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(self._grid_start, 0, 0)
        glVertex3f(self._grid_start * -1.0, 0, 0)
        glEnd()

        glColor(0.0, 1.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0, self._grid_start, 0)
        glVertex3f(0, self._grid_start * -1, 0)
        glEnd()

        glColor(0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, self._grid_start)
        glVertex3f(0, 0, self._grid_start * -1)
        glEnd()

        glEnable(GL_LIGHTING)
        return 0
