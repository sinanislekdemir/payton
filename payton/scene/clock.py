import threading
import time
from typing import Any, Callable

# SAFE_ASSUMPTION constant is a wait time between each clock cycle.
SAFE_ASSUMPTION = 0.01


class ClockException(Exception):
    pass


class Clock(threading.Thread):
    def __init__(self, period: float, callback: Callable[[float, float], None]):
        threading.Thread.__init__(self)
        self.period = period
        self._total_time = 0.0
        self.callback = callback
        self._kill = False
        self._pause = True
        self._hold = False

    def kill(self) -> None:
        self._kill = True

    def pause(self) -> None:
        self._pause = not self._pause

    def run(self) -> Any:
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
