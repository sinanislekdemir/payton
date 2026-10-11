from payton.scene import Scene
from payton.scene.geometry import Mesh

scene = Scene()
mesh = Mesh()
mesh.add_triangle(
    [[-2, 0, 0], [2, 0, 0], [0, 2, 0]],
    texcoords=[[0, 0], [1, 0], [1, 1]],
    colors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
)

scene.add_object("mesh", mesh)
scene.run()
