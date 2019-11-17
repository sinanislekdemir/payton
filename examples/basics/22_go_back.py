from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import info_box

scene = Scene()


def move_forward(period, total):
    scene.objects["cube"].forward(1)
    if scene.objects["cube"].position[1] > 10:
        scene.clocks["forward"].pause()
        scene.clocks["back"]._pause = False
    else:
        scene.clocks["back"]._pause = True


def step_back_history(period, total):
    success = scene.objects["cube"].step_back()
    if not success:
        scene.clocks["back"].pause()


cube = Cube()
cube.track_motion = True
scene.add_object("cube", cube)
scene.create_clock("forward", 0.2, move_forward)
scene.create_clock("back", 0.2, step_back_history)

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
