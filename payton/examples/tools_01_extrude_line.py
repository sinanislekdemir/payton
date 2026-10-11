from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import extrude_line

line = Line(vertices=[[0, 0, 0], [2, 0, 1], [2, 2, 2], [0, 2, 3], [0, 0, 0]])

mesh = extrude_line(line, [0.0, 0.0, 1.0], 3)

scene = Scene()
scene.add_object("line", line)
scene.add_object("mesh", mesh)
scene.run()
