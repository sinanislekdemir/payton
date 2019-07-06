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
logging data, etc.

Each clock is technically a safe thread.

For accurate data calculations, time stops until the execution of the
callback function ends.

*Note:* All clocks are initially paused. You should hit *Space* to unpause
them. The main reason is, clock threads are created and started before
SDL2 and whole OpenGL scene is ready, so there is a chance that you can not
see the beginning of your time based simulations.

Example usage:

    .. include:: ../../examples/basics/04_clock.py
"""

import threading
import time
from typing import Callable, Any, Type
from payton.scene.receiver import Receiver

# SAFE_ASSUMPTION constant is a wait time between each clock cycle.
SAFE_ASSUMPTION = 0.01


class Clock(threading.Thread):
    """
    Each clock in Scene is actually a thread which has a
    reference of the scene. We carry this back reference as a Receiver
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
        Args:
          name: Name of the clock (freetext for logging purposes)
          period: Clock cycle period of time between callbacks (seconds between each tick)
          scene: Scene to be called back
          callback: Callback function to call when clock ticks.
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

        Return:
          bool
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
