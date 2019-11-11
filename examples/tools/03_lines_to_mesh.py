import os

from payton.scene import Scene
from payton.scene.geometry import Line
from payton.tools.mesh import lines_to_mesh

wall_line1 = Line(
    vertices=[[0, 0, 0], [5, 0, 0], [5, 0, 4], [0, 0, 4], [0, 0, 0]]
)
wall_line2 = Line(
    vertices=[[2, 0, 2], [3, 0, 2], [3, 0, 3], [2, 0, 3], [2, 0, 2]]
)

wall = lines_to_mesh([wall_line1, wall_line2])

texture_file = os.path.join(os.path.dirname(__file__), "wall.jpg")
wall.material.texture = texture_file

scene = Scene()
scene.add_object("wall_line1", wall_line1)
scene.add_object("wall_line2", wall_line2)
scene.add_object("wall", wall)

roof_base = Line(
    vertices=[[-1, -1, 4], [6, -1, 4], [6, 3, 4], [-1, 3, 4], [-1, -1, 4]]
)

roof_top = Line(
    vertices=[
        [2, 0.999, 7],
        [3, 0.999, 7],
        [3, 1.001, 7],
        [2, 1.001, 7],
        [2, 0.999, 7],
    ]
)

roof = lines_to_mesh([roof_base, roof_top])
roof.material.texture = texture_file
roof.fix_texcoords(2, 2)  # Repeat the texture twice, make bricks at half size.

scene.add_object("roof_base_line", roof_base)
scene.add_object("roof_top_line", roof_top)
scene.add_object("roof", roof)

scene.run()
