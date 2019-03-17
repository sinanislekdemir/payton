import math
from payton.math.vector import sub_vector
from OpenGL import GL, GLU

BUTTON_LEFT = 1
BUTTON_RIGHT = 2


class Observer(object):
    """
    Observers are basically cameras in the scene.
    It uses gluLookAt function for Perspective cameras.
    """
    def __init__(self):
        """
        Initialize the defaults of an Observer.
        Default observer is a perspective mode camera
        standing at x:10 y:10 z:10 looking at the origin
        and up direction is x:0 y:1 z:0
        """
        self.position = [10.0, 10.0, 10.0]
        self.target = [0.0, 0.0, 0.0]
        self.up = [0.0, 0.0, 1.0]
        self.target_object = None
        self.fov = 45.0
        self.aspect_ratio = 800.0 / 600.0
        self.near = 0.1
        self.far = 100.0
        self.perspective = True
        # zoom factor for Orthographic projection
        self.zoom = 10

    def distance(self):
        """
        Calculate distance to target
        """
        xdiff = self.position[0] - self.target[0]
        ydiff = self.position[1] - self.target[1]
        zdiff = self.position[2] - self.target[2]
        return math.sqrt(xdiff * xdiff + ydiff * ydiff + zdiff * zdiff)

    def rotate_around_target(self, phi, theta):
        """
        Theta: horizontal
        Phi: vertical
        """
        diff = sub_vector(self.position, self.target)

        r = self.distance()
        _theta = math.acos(diff[2] / r)
        _phi = math.atan2(diff[1], diff[0])

        _theta = (math.degrees(_theta) + theta) % 360
        _phi = (math.degrees(_phi) + phi) % 360

        _theta = math.radians(_theta)
        _phi = math.radians(_phi)
        x = r * math.sin(_theta) * math.cos(_phi)
        y = r * math.sin(_theta) * math.sin(_phi)
        z = r * math.cos(_theta)

        self.position[0] = x + self.target[0]
        self.position[1] = y + self.target[1]
        self.position[2] = z + self.target[2]

    def mouse(self, button, shift, ctrl, x, y, xrel, yrel):
        if shift:
            if button == BUTTON_LEFT:
                self.rotate_around_target(xrel, -yrel)

        if ctrl:
            if button == BUTTON_LEFT:
                self.distance_to_target(self.distance() + yrel)

    def distance_to_target(self, distance):
        diff = sub_vector(self.position, self.target)
        _theta = math.acos(diff[2] / self.distance())
        _phi = math.atan2(diff[1], diff[0])

        self.position[0] = (self.target[0] + distance * math.sin(_theta) *
                            math.cos(_phi))
        self.position[1] = (self.target[1] + distance * math.sin(_theta) *
                            math.sin(_phi))
        self.position[2] = self.target[2] + distance * math.cos(_theta)

    def render(self):
        # Draw in 3D
        # Enable Depty Test
        GL.glMatrixMode(GL.GL_PROJECTION)
        GL.glPopMatrix()
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

        if self.perspective:
            # Set Perspective
            GLU.gluPerspective(self.fov,
                               self.aspect_ratio,
                               self.near,
                               self.far)
        else:
            GL.glOrtho(-self.zoom * self.aspect_ratio / 2,
                       self.zoom * self.aspect_ratio / 2,
                       -self.zoom * self.aspect_ratio / 2,
                       self.zoom * self.aspect_ratio / 2,
                       self.near, self.far)

        # Are we following an object?
        if self.target_object:
            self.target = self.target_object.matrix[12:15]

        # Y-Up
        GLU.gluLookAt(self.position[0],
                      self.position[1],
                      self.position[2],
                      self.target[0],
                      self.target[1],
                      self.target[2],
                      self.up[0],
                      self.up[1],
                      self.up[2])
        GL.glEnable(GL.GL_DEPTH_TEST)
