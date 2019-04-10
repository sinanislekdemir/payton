import math
import pyrr
import numpy as np

from payton.math.vector import sub_vector

BUTTON_LEFT = 1
BUTTON_RIGHT = 2


class Observer(object):
    """
    Observers are basically cameras in the scene.
    """
    def __init__(self, **args):
        """
        Initialize the defaults of an Observer.
        Default observer is a perspective mode camera
        standing at x:10 y:10 z:10 looking at the origin
        and up direction is x:0 y:1 z:0

        Args:
          position: Position of the camera in absolute coordinates.
                    [10., 10., 10.]
          target: Target of the camera in absolute coordinates.
                  [0., 0., 0.]
          up: Up vector of the camera. [0., 0., 1.] Z-Up
          target_object: Instead of a fixed coordinate, track an object. (None)
          fov: Field of view of the camera [45.]
          aspect_ratio: Aspect ratio of the camera. [4/3]
          near: Near clipping plane, can't be negative. [0.1]
          far: Far plane. Further objects will be invisible. [100.]
          active: Is this the active camera in the scene? [False]
        """
        self.position = args.get('position', [10.0, 10.0, 5.0])
        self.target = args.get('target', [0.0, 0.0, 0.0])
        self.up =  args.get('up', [0.0, 0.0, 1.0])

        self.target_object = args.get('target_object', None)
        self.fov = args.get('fov', 45.0)
        self.aspect_ratio = args.get('aspect_ratio', 800.0 / 600.0)
        self.near = args.get('near', 0.1)
        self.far = args.get('far', 100.0)

        # self.perspective = args.get('perspective', True)
        # zoom factor for Orthographic projection
        self.zoom = args.get('zoom', 10)
        self.active = args.get('active', False)
        self.perspective = args.get('perspective', True)

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
                if self.perspective:
                    self.distance_to_target(self.distance() + yrel)
                else:
                    self.distance_to_target(self.zoom + yrel)

    def distance_to_target(self, distance):
        if not self.perspective:
            self.zoom = distance
            return
        diff = sub_vector(self.position, self.target)
        _theta = math.acos(diff[2] / self.distance())
        _phi = math.atan2(diff[1], diff[0])

        self.position[0] = (self.target[0] + distance * math.sin(_theta) *
                            math.cos(_phi))
        self.position[1] = (self.target[1] + distance * math.sin(_theta) *
                            math.sin(_phi))
        self.position[2] = self.target[2] + distance * math.cos(_theta)

    def render(self):
        """Render camera into two matrices.

        Return:
          (projection_matrix, view_matrix)
        """
        if self.perspective:
            projection_matrix = pyrr.matrix44.create_perspective_projection(
                self.fov, self.aspect_ratio, self.near, self.far,
                dtype=np.float32)
        else:
            x = 100 * (self.aspect_ratio)
            y = 100
            if self.zoom == 0:
                self.zoom = 0.01
            projection_matrix = pyrr.matrix44.create_orthogonal_projection_matrix(
                left=(-x / self.zoom),
                right=(x / self.zoom),
                top=(y / self.zoom),
                bottom=(-y /self.zoom),
                near=self.near, far=self.far, dtype=np.float32)

        eye = np.array(self.position, dtype=np.float32)
        if self.target_object:
            self.target = self.target_object.get_position()

        target = np.array(self.target, dtype=np.float32)
        up = np.array(self.up, dtype=np.float32)
        view_matrix = pyrr.matrix44.create_look_at(eye, target, up)
        return projection_matrix, view_matrix
