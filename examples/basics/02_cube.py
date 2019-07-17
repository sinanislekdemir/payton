from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube = Cube()

cube_by_corners = Cube(from_corner=[-3, -3, 1], to_corner=[-1, -1, 3])

scene.add_object("cube", cube)
scene.add_object("cube_by_corners", cube_by_corners)

scene.run()
