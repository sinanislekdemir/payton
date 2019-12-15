import random
from copy import deepcopy
from itertools import product

from payton.scene import Scene
from payton.scene.geometry import MatrixPlane
from payton.scene.gui import info_box
from payton.scene.material import LIGHT_STEEL_BLUE, POINTS

water_size = 120
damp = 20


def calc_water(period, total):
    global plane
    global water
    global damp
    grid = plane.grid

    for j, i in product(range(1, water_size - 1), range(1, water_size - 1)):
        n = (
            water[i - 1][j]
            + water[i + 1][j]
            + water[i][j - 1]
            + water[i][j + 1]
        ) / 2.0
        n -= grid[i][j]
        n = n - (n / damp)
        grid[i][j] = n

    j = 0
    for i in range(1, water_size - 1):
        n = (water[i - 1][j] + water[i + 1][j] + water[i][j + 1]) / 2.0
        n -= grid[i][j]
        n -= n / damp
        grid[i][j] = n

    i = 0
    for j in range(1, water_size - 1):
        n = (water[i + 1][j] + water[i][j - 1] + water[i][j + 1]) / 2.0
        n -= grid[i][j]
        n -= n / damp
        grid[i][j] = n

    i = water_size - 1
    for j in range(1, water_size - 1):
        n = (water[i - 1][j] + water[i][j - 1] + water[i][j + 1]) / 2.0
        n -= grid[i][j]
        n -= n / damp
        grid[i][j] = n

    j = water_size - 1
    for i in range(1, water_size - 1):
        n = (water[i - 1][j] + water[i + 1][j] + water[i][j - 1]) / 2.0
        n -= grid[i][j]
        n -= n / damp
        grid[i][j] = n

    plane.update_grid()
    water, plane.grid = plane.grid, water


def drop(period, total):
    global water
    water[random.randint(0, water_size - 1)][
        random.randint(0, water_size - 1)
    ] = random.randint(-5, 15)


def ripple_pos(hit):
    global water
    i = int((hit[0] + 10) * 3)
    j = int((hit[1] + 10) * 3)
    if not (0 <= i < water_size):
        return
    if not (0 <= j < water_size):
        return
    water[i][j] = 5


scene = Scene()
scene.background.top_color = [0, 0, 0, 1]
scene.background.bottom_color = [0, 0, 0, 1]

scene.add_click_plane([0, 0, 0], [0, 0, 1], ripple_pos)
scene.grid.visible = False

plane = MatrixPlane(width=20, height=20, x=water_size, y=water_size)
water = deepcopy(plane.grid)

plane.material.display = POINTS
plane.material.color = LIGHT_STEEL_BLUE

scene.create_clock("ripple", 0.025, calc_water)
scene.create_clock("drop", 5, drop)
scene.add_object("plane", plane)
scene.active_observer.distance_to_target(20)
scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        width=220,
        height=100,
        label="Hit SPACE\nto start animation",
    ),
)

scene.run()
