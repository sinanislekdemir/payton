import random

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import Button, Hud, Theme, Window, WindowAlignment

scene = Scene()
hud = Hud()
scene.add_object("interface", hud)


def new_cube():
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    z = random.randint(0, 5)
    obj_count = len(scene.objects)
    cube = Cube()
    cube.position = [x, y, z]
    scene.add_object("cube_{}".format(obj_count), cube)


def print_fps(period, total):
    scene.huds["interface"].children["main_window"].children["fps_text"].label = f"FPS: {scene.fps}"


theme = Theme()
theme.text_color = [1.0, 0.0, 0.0]


main_window = Window("GUI Example", width=220, height=600, align=WindowAlignment.LEFT, theme=theme,)
create_cube = Button("New Cube", width=200, height=30, left=10, top=40, on_click=new_cube)
fps_text = Button("Hit Space for FPS", width=200, height=30, left=10, top=80)
main_window.add_child("create_cube", create_cube)
main_window.add_child("fps_text", fps_text)

hud.add_child("main_window", main_window)

scene.create_clock("fps", 0.01, print_fps)


scene.run()
