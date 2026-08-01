import random

from payton.scene import Scene
from payton.scene.geometry import MatrixPlane, NavMesh, Sphere
from payton.scene.gui import info_box

GRID = 50
SIZE = 30

obstacles = [
    (4, 8, 2, 10),
    (4, 10, 12, 16),
    (8, 14, 20, 26),
    (12, 18, 2, 10),
    (12, 16, 12, 18),
    (16, 22, 22, 28),
    (18, 24, 4, 10),
    (20, 28, 12, 16),
    (22, 28, 18, 22),
    (24, 30, 28, 34),
    (26, 34, 2, 8),
    (28, 32, 10, 16),
    (30, 36, 20, 28),
    (30, 38, 30, 36),
    (32, 38, 38, 44),
    (34, 40, 8, 14),
    (36, 42, 16, 22),
    (38, 44, 36, 42),
    (40, 46, 2, 10),
    (40, 44, 12, 18),
    (42, 46, 22, 30),
    (44, 48, 32, 38),
    (2, 8, 30, 38),
    (2, 8, 40, 46),
    (10, 14, 30, 38),
    (10, 16, 40, 48),
    (18, 22, 32, 38),
    (18, 24, 38, 46),
    (26, 30, 44, 48),
    (32, 36, 42, 48),
    (44, 48, 2, 8),
    (44, 48, 44, 48),
]

scene = Scene()
scene.active_camera.distance_to_target(SIZE * 1.5)

terrain = MatrixPlane(width=SIZE, height=SIZE, x=GRID, y=GRID)

for i in range(GRID):
    for j in range(GRID):
        terrain.grid[i][j] = 0.0
        terrain.color_grid[i][j] = [0.25, 0.45, 0.25]

for x1, x2, y1, y2 in obstacles:
    for i in range(x1, x2):
        for j in range(y1, y2):
            if 0 <= i < GRID and 0 <= j < GRID:
                terrain.grid[i][j] = 3.0
                terrain.color_grid[i][j] = [0.55, 0.25, 0.25]

terrain.update_grid()
scene.add_object("terrain", terrain)

navmesh = NavMesh(max_slope=55, max_step=0.6)
navmesh.build_from_mesh(terrain)
scene.add_object("navmesh", navmesh)

ball = Sphere(radius=0.35)
ball.material.color = [1.0, 0.0, 0.0]
scene.add_object("ball", ball)

waypoints: list[list[float]] = []
current_index = 0


def random_ground_point():
    candidates = [t for t in navmesh.graph.triangles if t.centroid[2] < 1.0]
    if not candidates:
        candidates = navmesh.graph.triangles
    if not candidates:
        return [0.0, 0.0, 0.0]
    return list(random.choice(candidates).centroid)


def pick_new_target():
    global waypoints, current_index
    pos = ball.position
    for _ in range(20):
        target = random_ground_point()
        path = navmesh.graph.find_path([pos[0], pos[1], 0.0], target)
        if path and len(path) >= 2:
            waypoints = path
            current_index = 0
            navmesh.highlight_path(path)
            return
    waypoints = []
    current_index = 0
    navmesh.clear_path()
    print("Could not find reachable target after 20 attempts")


def move_ball(period, total):
    global current_index, waypoints

    if not waypoints or current_index >= len(waypoints):
        pick_new_target()
        return

    target = waypoints[current_index]
    pos = ball.position
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = (dx**2 + dy**2) ** 0.5

    speed = 5.0
    if dist < 0.03:
        current_index += 1
        return

    step = min(speed * period, dist)
    ball.position = [
        pos[0] + (dx / dist) * step,
        pos[1] + (dy / dist) * step,
        0.35,
    ]


start = random_ground_point()
ball.position = [start[0], start[1], 0.35]
pick_new_target()

scene.create_clock("move_ball", 0.016, move_ball)

scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        label="Red ball wanders the maze. Green wireframe = navmesh. Yellow line = current path.",
    ),
)

scene.run(start_clocks=True)
