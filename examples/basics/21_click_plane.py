from payton.scene import Scene
from payton.scene.geometry import Line, Plane
from payton.scene.gui import info_box


def draw(hit):
    global scene
    scene.objects["line"].append([hit])


scene = Scene()
scene.add_click_plane([0, 0, 0.5], [0, 0, 1], draw)
scene.add_object("line", Line())
scene.objects["line"].material.color = [1.0, 0.0, 0.0]

ground = Plane(30, 30)
ground.position = [0, 0, 0.48]
scene.add_object("ground", ground)
scene.add_object(
    "info", info_box(left=10, top=10, width=220, height=100, label="Start Clicking \nto draw lines",),
)

scene.run()
