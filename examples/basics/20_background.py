from payton.scene import Scene
from payton.scene.gui import info_box

minutes = 0


def change_background(period, total):
    global minutes
    h = int(minutes / 60) % 24
    m = minutes % 60
    scene.background.set_time(h, m)
    minutes += 10


scene = Scene()
scene.create_clock("sun", 0.1, change_background)

scene.add_object(
    "info", info_box(left=10, top=10, label="Hit SPACE to start animation",),
)

scene.run()
