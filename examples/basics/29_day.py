import math
import os

from payton.scene import SHADOW_HIGH, Scene
from payton.scene.geometry import Wavefront

scene = Scene(1600, 900)
angle = 0


def change_date(period, total):
    global angle

    angle += 1
    if angle == 180:
        angle = 0

    if angle < 30 or angle > 150:
        scene.lights[0].color = [1, 0.7, 0]
    else:
        scene.lights[0].color = [1, 1, 1]

    total_minutes = angle * 4
    minute = total_minutes % 60
    hour = math.floor(total_minutes / 60) + 7

    scene.background.set_time(hour, minute)
    x = math.cos(math.radians(angle)) * 10
    y = math.sin(math.radians(angle)) * 10
    scene.lights[0].position = [x, x, y]


obj_file = os.path.join(os.path.dirname(__file__), "scene", "scene.obj")

obj = Wavefront(obj_file)
# scene.active_observer.perspective = False

scene.add_object("scene", obj)
scene.create_clock("mover", 0.05, change_date)
scene.active_observer.position = [
    3.658917285721423,
    5.527119165715093,
    2.5024855760753986,
]
scene.active_observer.target = [
    -0.8135842084884644,
    0.7309485077857971,
    0.05432761460542679,
]
scene.grid.visible = False
scene.shadow_quality = SHADOW_HIGH
scene.shadow_samples = 20
scene.run()
