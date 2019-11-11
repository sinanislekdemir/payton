"""Object Oriented aproach to Payton

As you have seen in other examples, mainly they are crafted with simplicty and
easy reading in mind. I always tried to keep things readable and understandable
by new starters. But as we also know, one of the bad practices in Python is
reaching to objects in Global. Reaching and modifying things in Global is
expensive and reduces the performance.

This example will show you simply how to deal with Payton in an object oriented
way.
"""
import math

from payton.scene import Scene
from payton.scene.geometry import Cube


class Application(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        cube = Cube()
        self.add_object("cube", cube)
        self.create_clock("rotator", 0.01, self.timer)

    def timer(self, period, total):
        self.objects["cube"].rotate_around_z(math.radians(1))


if __name__ == "__main__":
    app = Application()
    app.run()
