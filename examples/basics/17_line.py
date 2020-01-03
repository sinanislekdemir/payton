from payton.scene import Scene
from payton.scene.geometry import Line

scene = Scene()

line = Line(
    vertices=[[0, 0, 0], [0, 0, 1], [0.5, 0, 1.5], [1, 0, 1], [0, 0, 1], [1, 0, 0], [0, 0, 0], [1, 0, 1], [1, 0, 0]]
)

scene.add_object("line", line)
scene.run()
