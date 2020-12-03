import threading
import time
from typing import Any, Callable

# SAFE_ASSUMPTION constant is a wait time between each clock cycle.
SAFE_ASSUMPTION = 0.01


class ClockException(Exception):
    pass


class Clock(threading.Thread):
    """Clock thread

    Clock is a time-based thread which triggers the callback function
    in given intervals.

    Some use cases:
    * Get the object states every n microseconds and log them
    * Animate an object (move the object every 1/24 seconds)
    * Create an AI that scans the entire surroumdings regularly

    Best use case is to use it as a global timer in the whole scene
    with a single Clock.

    Due to GIL in Python Interpreter, try to keep the number of clocks as
    low as possible to prevent each clock to get in the way of main
    render cycle. Many clocks would probably create a lag in the scene.

    For example, you can have 10 AI bots in the scene.
    Create a single clock that loops and makes a decision for each bot
    instead of creating separate clocks for each bot.

    """

    def __init__(self, period: float, callback: Callable[[float, float], None]):
        """Initialize the clock

        Keyword arguments:
        period -- Call period of the clock in seconds as FLOAT
        callback -- Callback method to call.
        """
        threading.Thread.__init__(self)
        self.period = period
        self._total_time = 0.0
        self.callback = callback
        self._kill = False
        self._pause = True
        self._hold = False

    def kill(self) -> None:
        """Kill the clock, do not run anymore. (Kill the thread)"""
        self._kill = True

    def pause(self) -> None:
        """Pause the clock temporarily"""
        self._pause = not self._pause

    def run(self) -> Any:
        """Main clock cycle"""
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
