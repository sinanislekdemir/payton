import os
import math
from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import Hud, Rectangle, Text


def rotate(name, scene, period, total):
    y = math.radians(period * 100)
    y = -y if int(total) % 2 == 0 else y

    scene.objects["cube"].rotate_around_x(math.radians(period * 50))
    scene.objects["cube"].rotate_around_y(y)
    scene.objects["cube"].rotate_around_z(math.radians(period * 150))


def print_fps(name, scene, period, total):
    scene.huds["hud"].children["fps"].label = f"FPS: {scene.fps}"


texture_file = os.path.join(os.path.dirname(__file__), "cube.png")

scene = Scene()
scene.observers[0].distance_to_target(3)

cube = Cube()
cube.material.texture = texture_file
scene.add_object("cube", cube)

hud = Hud()
font_file = os.path.join(os.path.dirname(__file__), "../static/arial_narrow_7.ttf")
hud.set_font(font_file, 15)

scene.add_object("hud", hud)

texture_rect = Rectangle(position=(700, 500), size=(90, 90))
rectangle = Rectangle(position=(10, 20, 0), size=(200, 100))

texture_rect.material.texture = texture_file

hud.add_child("rect", rectangle)
hud.add_child("text_rect", texture_rect)


def click_func():
    t = Text(label="Yay!", position=(10, 40))
    rectangle.add_child(f"l_a", t)


text = Text(
    label="Click Me!",
    bgcolor=(0, 0, 0, 0.5),
    position=(10, 10),
    size=(100, 30),
    color=(1, 1, 1),
    on_click=click_func,
)
text.position = (0, 0)

rectangle.add_child("label", text)

info_text = """Hit Space to start cube animation
Shift + Mouse Drag = Rotate
Ctrl + Mouse Drag = Zoom
Ctrl + Shift + Mouse Drag = Pan
W = Change Mode
C = Change Camera Mode
"""

info = Text(
    label=info_text, position=(550, 0), color=(1, 1, 1), size=(300, 200)
)

hud.add_child("info", info)


fps = Text(label="Hit Space:", position=(0, 0), color=(1, 1, 1))

hud.add_child("fps", fps)

scene.create_clock("rotate", 0.01, rotate)
scene.create_clock("fps", 0.01, print_fps)

scene.run()
