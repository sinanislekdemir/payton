import os
from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.observer import Observer
from payton.scene.gui import Hud, Text

scene = Scene()

texture_file = os.path.join(os.path.dirname(__file__), "cube.png")

cube = Cube(width=5.0, height=5.0, depth=5.0)
cube.position = [0, 0, 2.5]
cube.material.texture = texture_file

scene.add_object("cube", cube)

inside_box = Observer(
    position=[-1.7898840267533351, 2.210322695165203, 1.400984730396208],
    target=[0, 0, 1],
    fov=110,
)

scene.add_observer(inside_box)

hud = Hud()
font_file = os.path.join(
    os.path.dirname(__file__), "../static/arial_narrow_7.ttf"
)
hud.set_font(font_file, 15)

info_text = "Cycle through cameras using F2 and F3"

info = Text(
    label=info_text, position=(550, 0), color=(1, 1, 1), size=(300, 200)
)

hud.add_child("info", info)

scene.add_object("hud", hud)
scene.run()
