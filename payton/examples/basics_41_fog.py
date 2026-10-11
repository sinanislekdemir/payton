"""Fog demo — shows linear atmospheric fog.

Objects are placed at increasing distances along the Y axis so you can
clearly see the fog thickening as they recede.  Press F1 for help.

Fog parameters in use:
    fog_mode    = 0   (linear)
    fog_near    = 10  metres  – fog starts here
    fog_far     = 60  metres  – fully opaque at this distance
    fog_color   = soft sky-blue, matching the background tones
"""

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane
from payton.scene.theme import THEME_GAMEENGINE

scene = Scene(theme=THEME_GAMEENGINE)
scene.lights[0].position = [20, -5, 20]

# Ground plane
ground = Plane(200, 200)
ground.material.color = [0.35, 0.50, 0.35]
scene.add_object("ground", ground)

# Row of cubes at increasing distances (Y axis goes "into" the scene)
colors = [
    [0.9, 0.2, 0.2],
    [0.9, 0.6, 0.1],
    [0.8, 0.8, 0.1],
    [0.2, 0.8, 0.2],
    [0.1, 0.6, 0.9],
    [0.5, 0.2, 0.9],
    [0.9, 0.2, 0.7],
    [0.8, 0.8, 0.8],
]

for i, color in enumerate(colors):
    distance = 5 + i * 8  # 5 m, 13 m, 21 m … 61 m
    cube = Cube()
    cube.material.color = color
    cube.position = [0.0, float(distance), 0.5]
    scene.add_object(f"cube_{i}", cube)

# Enable linear fog that matches the game-engine sky
scene.enable_fog()
scene.fog_mode = 0               # linear
scene.fog_color = [0.38, 0.55, 0.78]  # matches THEME_GAMEENGINE sky
scene.fog_near = 10.0            # metres
scene.fog_far = 60.0             # metres

scene.run()
