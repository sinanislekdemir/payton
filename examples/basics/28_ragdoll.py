import math
import random

from payton.math.functions import distance
from payton.scene import Scene
from payton.scene.geometry import Plane, RagDoll
from payton.scene.geometry.ragdoll import L_HIP, L_KNEE, L_SHOULDER, R_HIP, R_KNEE, R_SHOULDER
from payton.scene.gui import info_box


class RagdollApp(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.ragdoll = RagDoll()
        self.state = 1
        self.total_ang = 0
        self.add_object("ragdoll", self.ragdoll)
        self.create_clock("walk", 0.01, self.walk)
        self.add_object("ground", Plane(20, 20))

        self.target = [5, 0, 2]
        self.ragdoll.direct_to([5, 0, 2])

        self.ragdoll.joints[R_SHOULDER].rotate_around_y(math.radians(50))
        self.ragdoll.joints[L_SHOULDER].rotate_around_y(math.radians(-50))
        self.ragdoll.joints[R_KNEE].rotate_around_x(math.radians(-20))
        self.ragdoll.joints[L_KNEE].rotate_around_x(math.radians(-20))
        self.add_object(
            "info",
            info_box(
                left=10,
                top=10,
                label="Hit SPACE to start animation",
            ),
        )

    def walk(self, period, total):
        self.total_ang += self.state
        ang = self.state

        self.ragdoll.joints[R_HIP].rotate_around_x(math.radians(ang))
        self.ragdoll.joints[L_HIP].rotate_around_x(math.radians(-ang))
        self.ragdoll.joints[R_SHOULDER].rotate_around_x(math.radians(-ang))
        self.ragdoll.joints[L_SHOULDER].rotate_around_x(math.radians(ang))
        if self.total_ang > 40:
            self.state = -1
        if self.total_ang < -40:
            self.state = 1
        self.ragdoll.forward(0.025)
        if distance(self.target, self.ragdoll.position) < 0.5:
            self.target = [
                float(random.randint(-10, 10)),
                float(random.randint(-10, 10)),
                2,
            ]
            self.ragdoll.direct_to(self.target)
        if self.ragdoll.position[0] >= 10:
            self.ragdoll.position = [-10, 0, 2]


app = RagdollApp()
app.run()
