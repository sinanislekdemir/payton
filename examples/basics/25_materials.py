import os

from payton.scene import Scene
from payton.scene.geometry import Mesh
from payton.scene.material import DEFAULT, Material

scene = Scene()
mesh = Mesh()

yellow_material = Material(lights=False, color=[1.0, 1.0, 0.0, 1.0])
mesh.add_material("yellow", yellow_material)

# Regular add_triangle will add it with DEFAULT material
mesh.add_triangle(
    [[0, 0, 0], [2, 0, 0], [2, 2, 0]], texcoords=[[0, 0], [1, 0], [1, 1]]
)

# Explicit material definition
mesh.add_triangle(
    [[0, 0, 0], [2, 2, 0], [0, 2, 0]],
    texcoords=[[0, 0], [1, 1], [0, 1]],
    material="yellow",
)

texture_file = os.path.join(os.path.dirname(__file__), "cube.png")
mesh.materials[DEFAULT].texture = texture_file
# mesh.material.texture = texture_file  # implicit declaration
scene.add_object("mesh", mesh)
scene.run()
