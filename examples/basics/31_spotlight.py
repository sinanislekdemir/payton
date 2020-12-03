import math
import os

from payton.scene import Scene
from payton.scene.geometry import Wavefront

scene = Scene()
scene.background.top_color = [0, 0, 0, 1]
scene.background.bottom_color = [0, 0, 0, 1]

amount = 0.5
total_angles = -30


def swing(period, total):
    global total_angles, amount
    scene.objects["lamp"].rotate_around_x(math.radians(amount))
    total_angles += amount
    if total_angles >= 30:
        amount = -0.5
    if total_angles <= -60:
        amount = 0.5
    light_pos = scene.objects["lamp"].to_absolute([0, 0, -3.4])
    scene.lights[0].position = light_pos


table_file = os.path.join(os.path.dirname(__file__), "scene", "table.obj")
lamp_file = os.path.join(os.path.dirname(__file__), "scene", "lamp.obj")

table = Wavefront(table_file)
lamp = Wavefront(lamp_file)
lamp.fix_normals(reverse=True)
lamp.position = [0, 0, 12]

scene.create_clock("swing", 0.01, swing)
scene.active_camera.position = [
    8.261520800759284,
    8.259030103723475,
    17.54799562339614,
]
scene.lights[0].position = [0, 0, 8.6]
scene.add_object("table", table)
scene.add_object("lamp", lamp)
scene.run()
