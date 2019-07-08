from payton.scene import Scene
from payton.scene.geometry import Line


def draw(hit):
    global scene
    scene.objects["line"].append([hit])


scene = Scene()
scene.add_click_plane([0, 0, 0.5], [0, 0, 1], draw)
scene.add_object("line", Line())

scene.run()
