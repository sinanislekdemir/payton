import math
import os
from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import rotate_line


line = Line(
    vertices=[
        [0, 0, 0],
        [0.5, 0, 0],
        [0.6, 0, 0.25],
        [0.8, 0, 0.9],
        [0.9, 0, 1.5],
        [0.75, 0, 1.8],
        [0.3, 0, 2.1],
        [0.6, 0, 2.3],
    ]
)

mesh = rotate_line(line, [0, 0, 1], math.radians(270))

texture_file = os.path.join(os.path.dirname(__file__), "green.jpg")
mesh.material.texture = texture_file

scene = Scene()
scene.add_object("line", line)
scene.add_object("mesh", mesh)
scene.run()
