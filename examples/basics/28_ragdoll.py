import math

from payton.scene import Scene
from payton.scene.geometry import RagDoll
from payton.scene.geometry.ragdoll import (
    L_HIP,
    L_KNEE,
    L_SHOULDER,
    R_HIP,
    R_KNEE,
    R_SHOULDER,
)

scene = Scene()
ragdoll = RagDoll()
right_leg = True
total_ang = 0


def walk(period, total):
    global right_leg, total_ang
    if right_leg:
        total_ang += 1
        ang = 1
    else:
        total_ang -= 1
        ang = -1

    ragdoll.joints[R_HIP].rotate_around_x(math.radians(ang))
    ragdoll.joints[L_HIP].rotate_around_x(math.radians(-ang))
    ragdoll.joints[R_SHOULDER].rotate_around_x(math.radians(-ang))
    ragdoll.joints[L_SHOULDER].rotate_around_x(math.radians(ang))
    if total_ang > 40:
        right_leg = False
    if total_ang < -40:
        right_leg = True


scene.add_object("ragdoll", ragdoll)
scene.create_clock("walk", 0.01, walk)

ragdoll.joints[R_SHOULDER].rotate_around_y(math.radians(50))
ragdoll.joints[L_SHOULDER].rotate_around_y(math.radians(-50))
ragdoll.joints[R_KNEE].rotate_around_x(math.radians(-20))
ragdoll.joints[L_KNEE].rotate_around_x(math.radians(-20))
scene.run()
