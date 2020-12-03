import os

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import info_box
from payton.scene.camera import Camera

scene = Scene()
scene.background.top_color = [0, 0, 0, 1]
scene.background.bottom_color = [0, 0, 0, 1]

texture_file = os.path.join(os.path.dirname(__file__), "cube.png")

cube = Cube(width=5.0, height=5.0, depth=5.0)
cube.position = [0, 0, 2.5]
cube.material.texture = texture_file

scene.add_object("cube", cube)

inside_box = Camera(
    position=[-1.7898840267533351, 2.210322695165203, 1.400984730396208],
    target=[0, 0, 1],
    fov=110,
)

scene.add_camera(inside_box)

scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        label="Hit F2/F3 to switch between cameras",
    ),
)

scene.run()
