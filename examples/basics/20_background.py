import os
from payton.scene import Scene
from payton.scene.gui import Hud, Text

target = [124 / 255, 185 / 255, 232 / 255]
initial = [0, 0.1, 0.2, 1.0]
rise = True


def change_background(name, scene, period, total):
    global rise
    c = scene._background.top_color
    if rise:
        if c[0] <= target[0]:
            c[0] += 0.01
        if c[1] <= target[1]:
            c[1] += 0.01
        if c[2] <= target[2]:
            c[2] += 0.01
        if sum(c) >= sum(target):
            rise = False
    else:
        if c[0] >= initial[0]:
            c[0] -= 0.01
        if c[1] >= initial[1]:
            c[1] -= 0.01
        if c[2] >= initial[2]:
            c[2] -= 0.01

        if sum(c) <= sum(initial) + 0.01:
            rise = True
    scene._background.top_color = c


scene = Scene()
scene.create_clock("sun", 0.1, change_background)

hud = Hud()
font_file = os.path.join(
    os.path.dirname(__file__), "../static/arial_narrow_7.ttf"
)
hud.set_font(font_file, 15)

info_text = "Hit space to start background animation"

info = Text(
    label=info_text, position=(550, 0), color=(1, 1, 1), size=(300, 200)
)

hud.add_child("info", info)
scene.add_object("hud", hud)

scene.run()
