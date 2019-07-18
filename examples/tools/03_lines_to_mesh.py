from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import lines_to_mesh

line1 = Line(vertices=[[0, 0, 0], [5, 0, 0], [5, 0, 4], [0, 0, 4], [0, 0, 0]])

line2 = Line(vertices=[[2, 0, 2], [3, 0, 2], [3, 0, 3], [2, 0, 3], [2, 0, 2]])

mesh = lines_to_mesh([line1, line2])

scene = Scene()
scene.add_object("line1", line1)
scene.add_object("line2", line2)
scene.add_object("mesh", mesh)

scene.run()
