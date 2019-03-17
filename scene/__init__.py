"""
Payton Scene.

Python scene can be thought as a real movie scene, where you have your
actors, camera and lights. So, in a simulation, there can be
only one Scene running the show. *(Multiple scenes can be achieved by
doing things in a parallel programming way but due to Python limitations
like Global Interpreter Lock, there will not be much benefit to that)*

Scene has many items elements:

- Objects (`self.objects = []`)
- Observers (`self.observers = []`)
- Clocks (`self.clocks = {}`)
- Grid (`self.grid = Grid()`)
- window (SDL)

**Objects** hold the main scene actors.

**Grid** is the default grid in 3D Scene. It is easier to follow the
motion of objects when there is a grid.

**Clocks** give you actual clocks-timers you can program. If you want to
move your objects one step at a time, you need clocks. A motion can
not happen without `time` and as Albert Einstein said, *"time is what
clock measures"*

"""
