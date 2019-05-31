import logging
import math

from payton.scene import Scene
from payton.scene.geometry import Sphere

logging.basicConfig(level=logging.DEBUG)


LAUNCH_ANGLE = math.radians(30)  # 30 Degrees
GRAVITY = 9.8
INITIAL_VELOCITY = 20  # meters/seconds


def projectile_motion(name, scene, period, total):
    # projectile motion.
    # y = v0 * t * cos(a)
    # z = v0 * t * sin(a) - 1/2 * g * t^2
    global LAUNCH_ANGLE
    global GRAVITY
    global INITIAL_VELOCITY
    position = scene.objects["ball"].get_position()
    if position[2] < 0:
        # Do not continue simulation if we hit the ground.
        scene.clocks[name].kill()  # We do not need this clock anymore
        return None

    # Go towards -Y direction.
    position[1] = -(INITIAL_VELOCITY * total * math.cos(LAUNCH_ANGLE))
    position[2] = INITIAL_VELOCITY * total * math.sin(
        LAUNCH_ANGLE
    ) - 0.5 * GRAVITY * (total ** 2)
    scene.objects["ball"].set_position(position)
    return None


def logger(name, scene, period, total):
    if scene.objects["ball"].matrix[3][2] < 0:
        # Do not continue simulation if we hit the ground.
        scene.clocks[name].kill()  # We do not need this clock anymore
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
pm_scene = Scene()

ball = Sphere(radius=1, track_motion=True)

# Add ball to the scene
pm_scene.add_object("ball", ball)
pm_scene.observers[0].target_object = ball  # Track the ball

pm_scene.grid.resize(30, 30, 2)

pm_scene.create_clock("motion", 0.01, projectile_motion)
pm_scene.create_clock("logger", 0.05, logger)

pm_scene.run()
