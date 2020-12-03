import os

from payton.scene import Scene
from payton.scene.geometry import Mesh

scene = Scene()
mesh = Mesh()
mesh.add_triangle(
    [[0, 0, 0], [2, 0, 0], [2, 2, 0]],
    texcoords=[[0, 0], [1, 0], [1, 1]],
)
mesh.add_triangle(
    [[0, 0, 0], [2, 2, 0], [0, 2, 0]],
    texcoords=[[0, 0], [1, 1], [0, 1]],
)
texture_file = os.path.join(os.path.dirname(__file__), "cube.png")
mesh.material.texture = texture_file
scene.add_object("mesh", mesh)
scene.run()
