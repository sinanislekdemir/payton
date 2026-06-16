import random

from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import Button, EditBox, Hud, Slider, Theme, Window, WindowAlignment

scene = Scene()
hud = Hud()
scene.add_object("interface", hud)

w, d, h = 1.0, 1.0, 1.0
_selected: str = ""
_prev_selected: str = ""


def new_cube():
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    z = random.randint(0, 5)
    obj_count = len(scene.objects)
    name = "cube_{}".format(obj_count)
    cube = Cube(width=w, depth=d, height=h)
    cube.position = [x, y, z]
    scene.add_object(name, cube)


def on_select(objects):
    global _selected, _prev_selected
    if not objects:
        return
    obj = objects[0]
    name = None
    for k, v in scene.objects.items():
        if v is obj:
            name = k
            break
    if name is None:
        return
    if _prev_selected and _prev_selected in scene.objects:
        scene.objects[_prev_selected].toggle_wireframe()
    _selected = name
    _prev_selected = name
    obj.toggle_wireframe()
    _update_sliders_from_selected()


scene.on_select = on_select


def _update_sliders_from_selected():
    main = scene.huds["interface"].children["main_window"]
    if _selected and _selected in scene.objects:
        pos = scene.objects[_selected].position
        main.children["slider_x"].value = pos[0]
        main.children["slider_y"].value = pos[1]
        main.children["slider_z"].value = pos[2]


def _set_pos(axis: int, val: float):
    if _selected and _selected in scene.objects:
        obj = scene.objects[_selected]
        pos = list(obj.position)
        pos[axis] = val
        obj.position = pos


def set_pos_x(val: float):
    _set_pos(0, val)


def set_pos_y(val: float):
    _set_pos(1, val)


def set_pos_z(val: float):
    _set_pos(2, val)


def print_fps(period, total):
    scene.huds["interface"].children["main_window"].children[
        "fps_text"
    ].label = f"FPS: {scene.fps}"


def set_size(val: str):
    global w, d, h
    parts = val.split(",")
    if len(parts) != 3:
        print("Invalid format given")
    try:
        w = float(parts[0])
        d = float(parts[1])
        h = float(parts[2])
    except ValueError:
        print("Not a valid float given")


theme = Theme()
theme.text_color = [1.0, 0.0, 0.0]


main_window = Window(
    "GUI Example",
    width=220,
    height=600,
    align=WindowAlignment.LEFT,
    theme=theme,
)

create_cube = Button(
    "Create New Cube", width=200, height=30, left=10, top=40, on_click=new_cube
)
cube_size = EditBox(
    "1, 1, 1", width=200, height=30, left=10, top=80, on_change=set_size
)
fps_text = Button("Hit Space for FPS", width=200, height=30, left=10, top=120)

slider_x = Slider(
    "X", width=200, height=30, left=10, top=160,
    min_value=-10.0, max_value=10.0, on_change=set_pos_x,
)
slider_y = Slider(
    "Y", width=200, height=30, left=10, top=200,
    min_value=-10.0, max_value=10.0, on_change=set_pos_y,
)
slider_z = Slider(
    "Z", width=200, height=30, left=10, top=240,
    min_value=0.0, max_value=20.0, on_change=set_pos_z,
)

free_text = EditBox(
    "The nine-to-five is one of the greatest atrocities sprung upon mankind. You give your life away to a function that doesn't interest you.",
    width=200,
    height=200,
    left=10,
    top=280,
    multiline=True,
)

main_window.add_child("create_cube", create_cube)
main_window.add_child("cube_size", cube_size)
main_window.add_child("fps_text", fps_text)
main_window.add_child("slider_x", slider_x)
main_window.add_child("slider_y", slider_y)
main_window.add_child("slider_z", slider_z)
main_window.add_child("free_text", free_text)

hud.add_child("main_window", main_window)

scene.create_clock("fps", 0.01, print_fps)


scene.run()
