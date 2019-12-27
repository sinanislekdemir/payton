import random
from copy import deepcopy
from itertools import product

from payton.scene import Scene
from payton.scene.geometry import MatrixPlane
from payton.scene.gui import info_box
from payton.scene.material import LIGHT_STEEL_BLUE, POINTS


class RippleApp(Scene):
    def __init__(self, water_size=60, damp=10, **kwargs):
        super().__init__(**kwargs)
        self.water_size = water_size
        self.damp = damp
        self.background.top_color = [0, 0, 0, 1]
        self.background.bottom_color = [0, 0, 0, 1]

        self.add_click_plane([0, 0, 0], [0, 0, 1], self.ripple_pos)
        self.grid.visible = False

        self.plane = MatrixPlane(
            width=20, height=20, x=water_size, y=water_size
        )
        self.water = deepcopy(self.plane.grid)

        self.plane.material.display = POINTS
        self.plane.material.color = LIGHT_STEEL_BLUE

        self.create_clock("ripple", 0.025, self.calc_water)
        self.create_clock("drop", 5, self.drop)
        self.add_object("plane", self.plane)
        self.active_observer.distance_to_target(20)
        self.add_object(
            "info",
            info_box(
                left=10,
                top=10,
                width=220,
                height=100,
                label="Hit SPACE\nto start animation",
            ),
        )

    def drop(self, period, total):
        self.water[random.randint(0, self.water_size - 1)][
            random.randint(0, self.water_size - 1)
        ] = random.randint(-5, 15)

    def ripple_pos(self, hit):
        f = self.water_size / 20
        i = int((hit[0] + 10) * f)
        j = int((hit[1] + 10) * f)
        if not (0 <= i < self.water_size):
            return
        if not (0 <= j < self.water_size):
            return
        self.water[i][j] = 5

    def calc_water(self, period, total):
        grid = self.plane.grid
        water_size = self.water_size
        damp = self.damp

        for j, i in product(
            range(1, self.water_size - 1), range(1, water_size - 1)
        ):
            n = (
                self.water[i - 1][j]
                + self.water[i + 1][j]
                + self.water[i][j - 1]
                + self.water[i][j + 1]
            ) / 2.0
            n -= grid[i][j]
            n = n - (n / damp)
            grid[i][j] = n

        j = 0
        for i in range(1, water_size - 1):
            n = (
                self.water[i - 1][j]
                + self.water[i + 1][j]
                + self.water[i][j + 1]
            ) / 2.0
            n -= grid[i][j]
            n -= n / damp
            grid[i][j] = n

        i = 0
        for j in range(1, water_size - 1):
            n = (
                self.water[i + 1][j]
                + self.water[i][j - 1]
                + self.water[i][j + 1]
            ) / 2.0
            n -= grid[i][j]
            n -= n / damp
            grid[i][j] = n

        i = water_size - 1
        for j in range(1, water_size - 1):
            n = (
                self.water[i - 1][j]
                + self.water[i][j - 1]
                + self.water[i][j + 1]
            ) / 2.0
            n -= grid[i][j]
            n -= n / damp
            grid[i][j] = n

        j = water_size - 1
        for i in range(1, water_size - 1):
            n = (
                self.water[i - 1][j]
                + self.water[i + 1][j]
                + self.water[i][j - 1]
            ) / 2.0
            n -= grid[i][j]
            n -= n / damp
            grid[i][j] = n

        self.plane.update_grid()
        self.water, self.plane.grid = self.plane.grid, self.water


app = RippleApp(water_size=30, damp=20)
app.run()
