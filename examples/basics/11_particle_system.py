import math
import random

from payton.scene import Scene
from payton.scene.geometry import ParticleSystem
from payton.scene.gui import Hud, Text, info_box

GRAVITY = 9.8


class ParticleDemo(Scene):
    def __init__(self, width=800, height=600, on_select=None, **kwargs):
        super().__init__(width=width, height=height, on_select=on_select, **kwargs)
        self.ps: ParticleSystem = ParticleSystem()
        self.ps.material.color = [1.0, 0.3, 0.0]
        self.add_object("ps", self.ps)

        self.add_object(
            "info",
            info_box(left=10, top=10, label="Hit SPACE to start animation\n10000 Particles"),
        )
        hud = Hud()
        text = Text(
            label="FPS:",
            position=(5, 205),
            size=(200, 35),
            color=(1, 1, 1),
        )

        hud.add_child("text", text)
        self.add_object("hud", hud)

        self.create_clock("animate", 0.01, self.animate)
        self.create_clock("fps", 0.5, self.show_fps)

    def generate_particles(self):
        for i in range(10000):
            angle = math.radians(random.randint(0, 9000) / 100.0)
            orientation = math.radians(random.randint(0, 3600) / 10.0)
            velocity = float(random.randint(100, 200) / 10.0)
            self.ps.add([0, 0, 0.2], angle=angle, velocity=velocity, orientation=orientation)

    def show_fps(self, period, total):
        self.huds["hud"].children["text"].label = f"FPS: {self.fps}"

    def animate(self, period, total):
        for i in range(10000):
            position = self.ps._vertices[i]
            if position[2] < 0:
                continue
            xs = math.cos(self.ps.meta[i]["orientation"])
            ys = math.sin(self.ps.meta[i]["orientation"])
            angle = self.ps.meta[i]["angle"]
            dist = -(self.ps.meta[i]["velocity"] * total * math.cos(angle))
            height = self.ps.meta[i]["velocity"] * total * math.sin(angle) - 0.5 * GRAVITY * (total ** 2)

            position[0] = dist * xs
            position[1] = dist * ys
            position[2] = height
            self.ps._vertices[i] = position
        self.ps.refresh()


scene = ParticleDemo()
scene.generate_particles()

scene.run()
