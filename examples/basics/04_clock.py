import logging
import math

from payton.scene import SHADOW_HIGH, Scene
from payton.scene.geometry import Plane, Sphere
from payton.scene.gui import info_box

logging.basicConfig(level=logging.DEBUG)


LAUNCH_ANGLE = math.radians(30)  # 30 Degrees
GRAVITY = 9.8
INITIAL_VELOCITY = 20  # meters/seconds


def projectile_motion(period, total):
    # projectile motion.
    # y = v0 * t * cos(a)
    # z = v0 * t * sin(a) - 1/2 * g * t^2
    global scene
    position = scene.objects["ball"].position
    if position[2] < 0:
        # Do not continue simulation if we hit the ground.
        scene.clocks["motion"].kill()  # We do not need this clock anymore
        return None

    # Go towards -Y direction.
    position[1] = -(INITIAL_VELOCITY * total * math.cos(LAUNCH_ANGLE))
    position[2] = INITIAL_VELOCITY * total * math.sin(LAUNCH_ANGLE) - 0.5 * GRAVITY * (total ** 2)
    scene.objects["ball"].position = position
    return None


def logger(period, total):
    global scene
    if scene.objects["ball"].matrix[3][2] < 0:
        # Do not continue simulation if we hit the ground.
        scene.clocks["logger"].kill()  # We do not need this clock anymore
        return None

    # Log ball location
    logging.debug(
        "Ball position: x:{} y:{} z:{} t={}".format(
            scene.objects["ball"].matrix[3][0],
            scene.objects["ball"].matrix[3][1],
            scene.objects["ball"].matrix[3][2],
            total,
        )
    )


#  Definitions
scene = Scene()
scene.shadow_quality = SHADOW_HIGH

ball = Sphere(radius=1, track_motion=True)

# Add ball to the scene
scene.add_object("ball", ball)
scene.active_observer.target_object = ball  # Track the ball

scene.grid.resize(30, 30, 2)
ground = Plane(80, 80)
scene.add_object("gr", ground)

scene.create_clock("motion", 0.005, projectile_motion)
scene.create_clock("logger", 0.05, logger)

scene.add_object(
    "info", info_box(left=10, top=10, label="Hit SPACE to start animation"),
)

scene.run()
