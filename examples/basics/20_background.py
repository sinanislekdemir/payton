from payton.scene import Scene
from payton.scene.gui import info_box

target = [124 / 255, 185 / 255, 232 / 255]
initial = [0, 0.1, 0.2, 1.0]
rise = True


def change_background(period, total):
    global scene, rise
    c = scene.background.top_color
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
    scene.background.top_color = c


scene = Scene()
scene.create_clock("sun", 0.1, change_background)

scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        width=220,
        height=100,
        label="Hit SPACE\nto start animation",
    ),
)

scene.run()
