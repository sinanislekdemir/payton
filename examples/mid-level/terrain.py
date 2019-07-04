import os
from PIL import Image
from payton.scene import Scene
from payton.scene.geometry import Mesh

scene = Scene()
mesh = Mesh()

texture = os.path.join(os.path.dirname(__file__), "engraved.jpg")

img = Image.open(texture)
pix = img.load()

aspect = img.size[0] / img.size[1]

pixel_size_x = 5 / img.size[0]
pixel_size_y = 5 / (img.size[1] * aspect)

print("Building the engraving")
x, y = img.size
for j in range(y-1):
    for i in range(x - 1):
        v_1 = pix[i, j][0] / 255
        v_2 = pix[i+1, j][0] / 255
        v_3 = pix[i+1, j+1][0] / 255
        v_4 = pix[i, j+1][0] / 255

        mesh.add_triangle(
            [
                [i*pixel_size_x, j*pixel_size_y, v_1],
                [(i+1)*pixel_size_x, j*pixel_size_y, v_2],
                [(i+1)*pixel_size_x, (j+1)*pixel_size_y, v_3]
            ],
            texcoords=[
                [i/img.size[0], j/img.size[1]],
                [(i+1)/img.size[0], j/img.size[1]],
                [i/img.size[0], (j+1)/img.size[1]]
            ]
        )

        mesh.add_triangle(
            [
                [i*pixel_size_x, j*pixel_size_y, v_1],
                [(i+1)*pixel_size_x, (j+1)*pixel_size_y, v_3],
                [i*pixel_size_x, (j+1)*pixel_size_y, v_4]
            ],
            texcoords=[
                [i/img.size[0], j/img.size[1]],
                [(i+1)/img.size[0], (j+1)/img.size[1]],
                [i/img.size[0], (j+1)/img.size[1]]
            ]
        )

print("Done, now loading the scene")
mesh.material.texture = texture
mesh.position = [-2.5, -3, 0]
scene.add_object("mesh", mesh)
scene.run()
