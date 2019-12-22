"""
Clock is a special timer which runs in parallel to your scene and can do
many things.

Each clock has a period, scene and a callback function. Basically, for each
clock tick, the callback function gets called with `period` and `total`
variables.

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
from typing import Any, Callable

# SAFE_ASSUMPTION constant is a wait time between each clock cycle.
SAFE_ASSUMPTION = 0.01


class ClockException(Exception):
    pass


class Clock(threading.Thread):
    """
    Each clock in Scene is actually a thread which has a
    reference of the scene. We carry this back reference as a Receiver
    Timer function carries a callback function to be called.
    """

    def __init__(
        self, period: float, callback: Callable[[float, float], None]
    ):
        """
        Args:
          name: Name of the clock (freetext for logging purposes)
          period: Clock cycle period of time between callbacks (seconds between
                  each tick)
          scene: Scene to be called back
          callback: Callback function to call when clock ticks.
        """
        threading.Thread.__init__(self)
        self.period = period
        self._total_time = 0.0
        self.callback = callback
        self._kill = False
        self._pause = True
        self._hold = False

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
        if self._kill:
            raise ClockException("This clock is already killed")
        while True:
            if self._kill:
                return False

            if self._pause:
                # sleep a safe time to avoid excessive cpu usage
                time.sleep(SAFE_ASSUMPTION)
                continue

            self.callback(self.period, self._total_time)

            time.sleep(self.period)
            self._total_time += self.period
        return True
