import math
import os
import random

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane

COLS = 16
ROWS = 16
CELL = 2.0
WALL_H = 3.0
THICK = 0.3
EYE_H = 1.5
SPEED = 0.45

scene = Scene()
scene.theme_gameengine()

brick_tex = os.path.join(
    os.path.dirname(__file__), '..', 'static', 'Bricks097_1K-JPG_Color.jpg'
)

floor = Plane(width=COLS * CELL, height=ROWS * CELL)
floor.position = [COLS * CELL / 2, ROWS * CELL / 2, -0.01]
scene.add_object('floor', floor)

progress = 0.0
waypoints: list[list[float]] = []
wall_names: list[str] = []


def build_maze() -> None:
    global waypoints, progress, wall_names

    # Remove previous walls (GL cleanup deferred to render thread)
    for name in wall_names:
        scene.remove_object(name)
    wall_names = []

    maze = [[0] * COLS for _ in range(ROWS)]
    parent: dict[tuple[int, int], tuple[int, int]] = {}

    def carve(x: int, y: int) -> None:
        dirs = [(0, -1, 1, 4), (1, 0, 2, 8), (0, 1, 4, 1), (-1, 0, 8, 2)]
        random.shuffle(dirs)
        for dx, dy, bit, opp in dirs:
            nx, ny = x + dx, y + dy
            if 0 <= nx < COLS and 0 <= ny < ROWS and maze[ny][nx] == 0:
                maze[y][x] |= bit
                maze[ny][nx] |= opp
                parent[(nx, ny)] = (x, y)
                carve(nx, ny)

    carve(0, 0)
    maze[0][0] |= 8
    maze[ROWS - 1][COLS - 1] |= 2

    path: list[tuple[int, int]] = []
    cell = (COLS - 1, ROWS - 1)
    while True:
        path.append(cell)
        if cell == (0, 0):
            break
        cell = parent[cell]
    path.reverse()

    waypoints = [[-0.5 * CELL, 0.5 * CELL, EYE_H]]
    for x, y in path:
        waypoints.append([x * CELL + CELL / 2, y * CELL + CELL / 2, EYE_H])
    waypoints.append([COLS * CELL + 0.5 * CELL, (ROWS - 0.5) * CELL, EYE_H])

    segs: list[tuple[float, float, float, float]] = []
    for y in range(ROWS):
        for x in range(COLS):
            if not (maze[y][x] & 2):
                segs.append(((x + 1) * CELL, y * CELL, (x + 1) * CELL, (y + 1) * CELL))
            if not (maze[y][x] & 4):
                segs.append((x * CELL, (y + 1) * CELL, (x + 1) * CELL, (y + 1) * CELL))

    if ROWS > 1:
        segs.append((0, CELL, 0, ROWS * CELL))
        segs.append((COLS * CELL, 0, COLS * CELL, (ROWS - 1) * CELL))
    segs.append((0, 0, COLS * CELL, 0))
    segs.append((0, ROWS * CELL, COLS * CELL, ROWS * CELL))

    ht = THICK / 2.0
    for i, (x1, y1, x2, y2) in enumerate(segs):
        dx = x2 - x1
        dy = y2 - y1
        length = math.sqrt(dx * dx + dy * dy)
        if length == 0:
            continue
        nx = dy / length
        ny = -dx / length

        cx = [x1 - nx * ht, x1 + nx * ht, x2 - nx * ht, x2 + nx * ht]
        cy = [y1 - ny * ht, y1 + ny * ht, y2 - ny * ht, y2 + ny * ht]

        cube = Cube(from_corner=[min(cx), min(cy), 0],
                    to_corner=[max(cx), max(cy), WALL_H])
        cube.material.texture = brick_tex
        cube.fix_texcoords()
        name = f'w{i}'
        scene.add_object(name, cube)
        wall_names.append(name)

    cam = scene.active_camera
    cam.position = waypoints[0]
    cam.target = waypoints[1]
    cam._use_cache = False
    progress = 0.0


build_maze()


def move(period: float, total: float) -> None:
    global progress
    if progress >= len(waypoints) - 1:
        build_maze()
        return
    progress += period / SPEED
    pos_idx = min(progress, len(waypoints) - 1)
    i = int(pos_idx)
    f = pos_idx - i
    if i >= len(waypoints) - 1:
        return
    c = scene.active_camera
    p0 = waypoints[i]
    p1 = waypoints[i + 1]
    c.position = [
        p0[0] + (p1[0] - p0[0]) * f,
        p0[1] + (p1[1] - p0[1]) * f,
        p0[2] + (p1[2] - p0[2]) * f,
    ]
    look_ahead = min(pos_idx + 1.5, len(waypoints) - 1)
    li = int(look_ahead)
    lf = look_ahead - li
    if li >= len(waypoints) - 1:
        c.target = waypoints[-1]
    else:
        t0 = waypoints[li]
        t1 = waypoints[li + 1]
        c.target = [
            t0[0] + (t1[0] - t0[0]) * lf,
            t0[1] + (t1[1] - t0[1]) * lf,
            t0[2] + (t1[2] - t0[2]) * lf,
        ]
    c._use_cache = False


scene.create_clock('move', 0.01, move)
scene.run(start_clocks=True)
