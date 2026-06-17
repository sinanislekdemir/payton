import math
import os

from payton.scene import SHADOW_HIGH, Scene
from payton.scene.geometry import Wavefront
from payton.scene.gui import Button, Hud, Slider, Window, WindowAlignment
from payton.scene.light import Light

scene = Scene(1600, 900)
angle = 0
paused = False
_slider_from_clock = False

night_light_1 = Light(position=[-0.7, 2.3, 1.2], color=[1.0, 0.6, 0.1])
night_light_2 = Light(position=[0.7, 2.3, 1.2], color=[1.0, 0.6, 0.1])
scene.lights.extend([night_light_1, night_light_2])

sun = scene.lights[0]


def update_daylight():
    global angle
    night = 180 <= angle < 360
    if night:
        sun.color = [0, 0, 0]
        night_light_1.color = [1.0, 0.6, 0.1]
        night_light_2.color = [1.0, 0.6, 0.1]
    else:
        if angle < 30 or angle > 150:
            sun.color = [1, 0.7, 0]
        else:
            sun.color = [1, 1, 1]
        night_light_1.color = [0, 0, 0]
        night_light_2.color = [0, 0, 0]

    total_minutes = angle * 4
    minute = int(total_minutes) % 60
    hour = int(math.floor(total_minutes / 60)) + 7
    scene.background.set_time(hour, minute)
    x = math.cos(math.radians(angle)) * 10
    y = math.sin(math.radians(angle)) * 10
    sun.position = [x, x, y]


def change_date(period, total):
    global angle, _slider_from_clock
    if paused:
        return
    angle += 1
    if angle >= 360:
        angle = 0
    update_daylight()
    _slider_from_clock = True
    time_slider.value = angle
    _slider_from_clock = False


def on_pause_click():
    global paused
    paused = not paused
    btn_pause.label = "Play" if paused else "Pause"


def on_time_change(val):
    global angle, paused, _slider_from_clock
    if _slider_from_clock:
        return
    paused = True
    btn_pause.label = "Play"
    angle = val
    update_daylight()


hud = Hud()
scene.add_object("interface", hud)

main_window = Window("Daylight Control", width=240, height=130, align=WindowAlignment.LEFT)

btn_pause = Button("Pause", width=200, height=30, left=10, top=30, on_click=on_pause_click)
time_slider = Slider(
    "Time",
    width=200,
    height=30,
    left=10,
    top=70,
    min_value=0.0,
    max_value=359.0,
    value=0.0,
    on_change=on_time_change,
)

main_window.add_child("btn_pause", btn_pause)
main_window.add_child("time_slider", time_slider)
hud.add_child("main_window", main_window)

obj_file = os.path.join(os.path.dirname(__file__), "scene", "scene.obj")

obj = Wavefront(obj_file)
for m in obj.materials.values():
    m.particle_size = 0.016
# scene.active_camera.perspective = False

scene.add_object("scene", obj)
scene.create_clock("mover", 0.05, change_date)
scene.active_camera.position = [
    3.658917285721423,
    5.527119165715093,
    2.5024855760753986,
]
scene.active_camera.target = [
    -0.8135842084884644,
    0.7309485077857971,
    0.05432761460542679,
]
scene.grid.visible = False
scene.shadow_quality = SHADOW_HIGH
scene.shadow_samples = 20
scene.run(start_clocks=True)
