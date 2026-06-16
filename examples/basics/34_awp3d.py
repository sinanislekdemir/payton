import os

from payton.scene import Scene
from payton.scene.geometry import Plane
from payton.scene.geometry.awp3d import AWP3D
from payton.scene.gui import Hud, ProgressBar, Window, WindowAlignment

object_file = os.path.join(os.path.dirname(__file__), "AWP3D", "fall-to-roll.awp3d")

scene = Scene()
hud = Hud()
scene.add_object("interface", hud)

loading_window = Window(
    "Loading",
    width=320,
    height=100,
    left=10,
    top=10,
)
loading_bar = ProgressBar(
    "Loading AWP3D",
    width=300,
    height=30,
    left=10,
    top=40,
    min_value=0.0,
    max_value=1.0,
)
loading_window.add_child("bar", loading_bar)
hud.add_child("loading", loading_window)

anim = AWP3D()
total = anim.begin_incremental_load(object_file)
loading_bar.max_value = float(total)
_done = False

def _load_tick(period, total_elapsed):
    global _done
    bar = scene.huds["interface"].children["loading"].children["bar"]
    if not _done:
        more = anim.load_next_frame()
        bar.value = anim._load_current - anim._load_min
        if not more:
            anim.finish_incremental_load()
            _done = True
    else:
        scene.huds["interface"].children["loading"].hide()
        scene.add_object("arissa", anim)
        ground = Plane(10, 10)
        scene.add_object("ground", ground)
        scene.clocks["_loader"].kill()

scene.create_clock("_loader", 0.0, _load_tick)

scene.run(start_clocks=True)
