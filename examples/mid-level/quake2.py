import os
import random

from payton.math.vector import distance
from payton.scene import Scene
from payton.scene.geometry import MD2, Plane


class App(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        ground = Plane(20, 20)
        self.lights[0].position = [10.0, 10.0, 10]
        model_file = os.path.join(
            os.path.dirname(__file__), "forgottenone2", "tris.md2"
        )
        weapon_file = os.path.join(
            os.path.dirname(__file__), "forgottenone2", "weapon.md2"
        )

        model = MD2(model_file, "ForgottenOne.pcx")
        weapon = MD2(weapon_file, "weapon.pcx")
        model.add_child("weapon", weapon)
        print(model.animations)
        print(weapon.animations)

        self.add_object("warrior", model)
        self.add_object("ground", ground)
        self.create_clock("walk", 0.01, self.walk)
        self.target = [5, 0, 0]
        model.direct_to([5, 0, 0])

    def walk(self, period, total):
        if self.objects["warrior"].animation == "":
            self.objects["warrior"].animate('run', 0, 5)
        self.objects["warrior"].forward(0.03)
        if distance(self.target, self.objects["warrior"].position) < 0.5:
            self.target = [
                float(random.randint(-10, 10)),
                float(random.randint(-10, 10)),
                0,
            ]
            self.objects["warrior"].direct_to(self.target)
        if self.objects["warrior"].position[0] >= 10:
            self.objects["warrior"].position = [-10, 0, 0]


app = App(width=1600, height=900)
app.run()
