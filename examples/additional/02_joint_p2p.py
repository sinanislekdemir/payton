from payton.scene import Scene
from payton.scene.geometry import Cube, Plane, Sphere

s = Scene()
ground = Plane(width=10, height=10)

cube = Cube(mass=1.0)
cube.position = [0, 0, 2]

sphere_1 = Sphere(mass=1.0)
sphere_2 = Sphere(mass=1.0)
sphere_1.position = [0, 0, 4]
sphere_2.position = [1, 0, 4]
sphere_1.constraint_point(sphere_2, [0.5, 0, 0.5], [-0.5, 0, 0.5])

s.add_object("ground", ground)
s.add_object("cube", cube)
s.add_object("sphere_1", sphere_1)
s.add_object("sphere_2", sphere_2)

s.run()
