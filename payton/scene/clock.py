"""
Clock is a special timer which runs in parallel to your scene and can do
many things.

Each clock has a period, scene and a callback function. Basically, for each
clock tick, the callback function gets called with `name`, `scene`, `period`
and `total` variables.

- `name`: name of the clock which calls the function.
- `scene`: an object reference of the scene to modify if needed
- `period`: period of clock. (time difference between each click in seconds)
- `total`: total time past since the beginning of the simulation.

A clock can be used for several occasions like moving objects in time,
logging data and etc.

Each clock is technically a safe thread.

For accurate data calculations, time stops until the execution of the
callback function ends.

*Note:* All clocks are initially paused. You should hit *P* to unpause them.
The main reason is, clock threads are created and started before SDL2 and
whole OpenGL scene is ready, so there is a chance that you can not see
the beginning of your time based simulations.

Example usage:

    import logging
    import math

    from payton.scene.scene import Scene
    from payton.scene.clock import Clock
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
        position = scene.objects['ball'].position
        if position[2] < 0:
            # Do not continue simulation if we hit the ground.
            scene.clocks[name].kill()  # We do not need this clock anymore
            return None

        # Go towards -Y direction.
        position[1] = -(INITIAL_VELOCITY * total *
                        math.cos(LAUNCH_ANGLE))
        position[2] = (INITIAL_VELOCITY * total *
                       math.sin(LAUNCH_ANGLE) -
                       0.5 * GRAVITY * (total ** 2))
        scene.objects['ball'].position = position
        return None


    def logger(name, scene, period, total):
        if scene.objects['ball'].matrix[3][2] < 0:
            # Do not continue simulation if we hit the ground.
            scene.clocks[name].kill()  # We do not need this clock anymore
            return None

        # Log ball location
        logging.debug('Ball position: x:{} y:{} z:{} t={}'.format(
            scene.objects['ball'].matrix[3][0],
            scene.objects['ball'].matrix[3][1],
            scene.objects['ball'].matrix[3][2], total))


    #  Definitions
    pm_scene = Scene()
    ball = Sphere(radius=1, track_motion=True)

    # Add ball to the scene
    pm_scene.add_object('ball', ball)
    pm_scene.observers[0].target_object = ball #  Track the ball
    pm_scene.grid.resize(30, 30, 2)
    pm_scene.create_clock("motion", 0.01, projectile_motion)
    pm_scene.create_clock("logger", 0.01, logger)
    pm_scene.run()

"""
import threading
import time
from typing import Callable, Any, Type
from payton.scene.receiver import Receiver

SAFE_ASSUMPTION = 0.01


class Clock(threading.Thread):
    """
    Each clock in Scene is actually a thread which has a
    reference of the scene.
    Timer function carries a callback function to be called.
    """

    def __init__(
        self,
        name: str,
        period: float,
        scene: Type[Receiver],
        callback: Callable[[str, Type[Receiver], float, float], None],
    ):
        """
        name of the thread is a freetext for logging purposes
        period is the waiting period between each callback
        so execution time is omitted.
        scene is the reference to the actual scene object.
        """
        threading.Thread.__init__(self)
        self.name = name
        self.scene = scene
        self.period = period
        self._total_time = 0.0
        self.callback = callback
        self._kill = False
        self._pause = True

    def kill(self) -> None:
        """
        Send a kill signal to existing clock.
        """
        self._kill = True

    def pause(self) -> None:
        """
        Send a pause signal to existing clock. Re-sending the same signal
        will let it continue.
        """
        self._pause = not self._pause

    def run(self) -> Any:
        """
        Callback function must have four arguments
        name of the clock for logging purposes
        scene reference to the actual scene object
        period of the clock
        total time difference from the initial run of the clock
        """
        while True:
            if self._kill:
                return False

            if self._pause:
                # sleep a safe time to avoid excessive cpu usage
                time.sleep(SAFE_ASSUMPTION)
                continue
            self.callback(self.name, self.scene, self.period, self._total_time)

            time.sleep(self.period)
            self._total_time += self.period
        return True
