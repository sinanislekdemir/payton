"""
Payton main geometry module

Geometry module holds the basic geometry shapes. They are all inherited
from `payton.scene.Object` class. They are as simple as possible.

Their face informations are generated at the initialization but display
lists are not generated until rendering.
"""


import math
from payton.math.vector import plane_normal
from payton.scene import Object
from payton.scene.shader import lightless_fragment_shader


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

        self._uvs = [
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
        super().__init__()
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

        for i in range(self.parallels + 1):
            for j in range(self.meridians + 1):
                x1 = r * math.sin(step_height * i) * math.cos(step_angle * j)
                y1 = r * math.sin(step_height * i) * math.sin(step_angle * j)
                z1 = r * math.cos(step_height * i)

                x2 = r * math.sin(step_height * (i + 1)) * math.cos(step_angle * j)
                y2 = r * math.sin(step_height * (i + 1)) * math.sin(step_angle * j)
                z2 = r * math.cos(step_height * (i + 1))

                x3 = r * math.sin(step_height * (i + 1)) * math.cos(step_angle * (j + 1))
                y3 = r * math.sin(step_height * (i + 1)) * math.sin(step_angle * (j + 1))
                z3 = r * math.cos(step_height * (i + 1))

                x4 = r * math.sin(step_height * i) * math.cos(step_angle * (j + 1))
                y4 = r * math.sin(step_height * i) * math.sin(step_angle * (j + 1))
                z4 = r * math.cos(step_height * i)
                normal = plane_normal([x1, y1, z1],
                                      [x2, y2, z2],
                                      [x3, y3, z3])
                self._vertices += [x1, y1, z1] # 0
                self._vertices += [x2, y2, z2] # i + 1
                self._vertices += [x3, y3, z3] # i + 2
                self._vertices += [x4, y4, z4] # i + 3
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._normals += [normal[0], normal[1], normal[2]]
                self._indices += [indices, indices+1, indices+2]
                self._indices += [indices, indices+2, indices+3]
                indices += 4
        return True
