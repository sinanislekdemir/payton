import math

from payton.scene import SHADOW_HIGH, Scene
from payton.scene.geometry import Cube
from payton.scene.geometry import Plane as Ground
from payton.scene.light import Light

scene = Scene()
scene.shadow_quality = SHADOW_HIGH
scene.shadow_samples = 4

ground = Ground(width=30, height=30)
scene.add_object("ground", ground)

cube = Cube(width=2, height=4, depth=2)
cube.position = [0, 0, 2]
scene.add_object("cube", cube)

scene.grid.visible = False

# Default light is already shadow_enabled=True — make it red
scene.lights[0].position = [7.0, 7.0, 6.0]
scene.lights[0].color = [1.0, 0.3, 0.3]

# Green and blue shadow-casting lights
green = Light(position=[-7.0, 7.0, 6.0], color=[0.3, 1.0, 0.3], shadow_enabled=True)
scene.lights.append(green)
blue = Light(position=[0.0, -7.0, 6.0], color=[0.3, 0.3, 1.0], shadow_enabled=True)
scene.lights.append(blue)


def move_lights(period, total):
    angle = total * 30
    r = 7.0
    scene.lights[0].position = [
        math.cos(math.radians(angle)) * r,
        math.sin(math.radians(angle)) * r,
        6.0,
    ]
    scene.lights[1].position = [
        math.cos(math.radians(angle + 120)) * r,
        math.sin(math.radians(angle + 120)) * r,
        6.0,
    ]
    scene.lights[2].position = [
        math.cos(math.radians(angle + 240)) * r,
        math.sin(math.radians(angle + 240)) * r,
        6.0,
    ]


scene.create_clock("orbit", 0.016, move_lights)


def screenshot_cb(period, total):
    if total > 3.0 and not hasattr(scene, "_done"):
        scene._done = True
        name = scene.screenshot()
        print(f"Screenshot saved: {name}")


scene.create_clock("ss", 0.5, screenshot_cb)

scene.run(start_clocks=True)
