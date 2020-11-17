import random

from payton.math.vector import distance, normalize_vector, scale_vector
from payton.scene import Scene
from payton.scene.geometry import Line
from payton.scene.gui import info_box
from payton.scene.material import BLACK, RED


class App(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.line = Line()
        self.vertex_velocities = []
        self.line.material.color = RED
        self.grid.visible = False

        for i in range(20):
            x = random.randint(-1000, 1000) / 100
            y = random.randint(-1000, 1000) / 100
            z = random.randint(-1000, 1000) / 100
            self.line.append(vertices=[[x, y, z]])
            v = normalize_vector([x, y, z])
            if random.randint(0, 1) == 1:
                v = scale_vector(v, -1)
            self.vertex_velocities.append(v)

        self.add_object("line", self.line)
        self.create_clock("clock", 0.01, self.animate)
        self.background.top_color = BLACK
        self.background.bottom_color = BLACK
        self.add_object(
            "info",
            info_box(
                left=10,
                top=10,
                label="Hit SPACE to start animation",
            ),
        )

    def animate(self, period, total):
        for i, vertex in enumerate(self.line._vertices):
            if distance([0, 0, 0], vertex) > 30:
                self.vertex_velocities[i] = scale_vector(self.vertex_velocities[i], -1)
            vertex[0] += self.vertex_velocities[i][0]
            vertex[1] += self.vertex_velocities[i][1]
            vertex[2] += self.vertex_velocities[i][2]

        self.line.refresh()


scene = App()
scene.run()
