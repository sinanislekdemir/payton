"""3-D spatial audio demo.

Moving cubes play looping sounds at their positions.
Click anywhere to trigger a one-shot sound at that location.

Requires: miniaudio (``pip install miniaudio``)
"""

import math
import struct
import wave
from tempfile import NamedTemporaryFile
from typing import Any

from payton.scene import Scene
from payton.scene.geometry import Cube, Sphere

SAMPLE_RATE = 44100


def _make_wav(freq: float, duration: float, filename: str) -> None:
    num_samples = int(SAMPLE_RATE * duration)
    samples = []
    for i in range(num_samples):
        t = i / SAMPLE_RATE
        envelope = max(0.0, 1.0 - t / duration)
        val = int(16000 * envelope * math.sin(2.0 * math.pi * freq * t))
        samples.append(struct.pack("<h", val))
    with wave.open(filename, "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(b"".join(samples))


beep_file = NamedTemporaryFile(suffix=".wav", delete=False)
_make_wav(440, 0.3, beep_file.name)
beep_file.close()

hum_file = NamedTemporaryFile(suffix=".wav", delete=False)
_make_wav(220, 1.0, hum_file.name)
hum_file.close()

tone_file = NamedTemporaryFile(suffix=".wav", delete=False)
_make_wav(880, 0.2, tone_file.name)
tone_file.close()


def play_at_click(hit: Any) -> None:
    scene.playSound(hit[:3], beep_file.name)


scene = Scene()
scene.add_click_plane([0, 0, 0], [0, 0, 1], play_at_click)

cube = Cube()
cube.position = [0, 0, 0.5]
scene.add_object("cube", cube)

sphere = Sphere()
sphere.position = [-3, 2, 0.5]
scene.add_object("sphere", sphere)


def move(period: float, total: float) -> None:
    angle = math.radians(total * 45 % 360)
    cube.position = [math.cos(angle) * 5, math.sin(angle) * 5, 0.5]


def move_sphere(period: float, total: float) -> None:
    angle = math.radians(total * 30 % 360)
    sphere.position = [-3 + math.cos(angle) * 3, 2 + math.sin(angle) * 3, 0.5]


scene.create_clock("move_cube", 0.01, move)
scene.create_clock("move_sphere", 0.01, move_sphere)

cube.playSound(hum_file.name, loop=True)
sphere.playSound(tone_file.name, loop=True)

print("Click on the grid to trigger sounds at that position.")
print("The cube and sphere have looping sounds that follow them.")
scene.run(start_clocks=True)
