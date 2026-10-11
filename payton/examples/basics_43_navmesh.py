from payton.scene import Scene
from payton.scene.geometry import NavMesh, MatrixPlane
from payton.scene.gui import info_box

start_point = None
end_point = None


def on_click(hit):
    global start_point, end_point, scene
    if start_point is None:
        start_point = hit
        print(f"Start: {hit}")
    elif end_point is None:
        end_point = hit
        print(f"End: {hit}")
        path = scene.objects["navmesh"].graph.find_path(start_point, end_point)
        if path:
            scene.objects["navmesh"].highlight_path(path)
            print(f"Path: {len(path)} waypoints")
        else:
            print("No path found!")
    else:
        scene.objects["navmesh"].clear_path()
        start_point = hit
        end_point = None
        print(f"New Start: {hit}")


scene = Scene()
scene.add_click_plane([0, 0, 0.01], [0, 0, 1], on_click)

GRID = 30
SIZE = 20
terrain = MatrixPlane(width=SIZE, height=SIZE, x=GRID, y=GRID)

for i in range(GRID):
    for j in range(GRID):
        terrain.grid[i][j] = 0
        terrain.color_grid[i][j] = [0.3, 0.5, 0.3]

obstacles = [
    (4, 12, 6, 14),
    (4, 16, 6, 22),
    (10, 12, 12, 20),
    (14, 14, 22, 16),
    (16, 8, 18, 16),
    (16, 18, 24, 20),
    (20, 12, 22, 14),
    (10, 4, 16, 6),
    (20, 4, 22, 10),
    (8, 24, 24, 26),
]

for col_start, col_end, row_start, row_end in obstacles:
    for i in range(col_start, col_end):
        for j in range(row_start, row_end):
            if 0 <= i < GRID and 0 <= j < GRID:
                terrain.grid[i][j] = 3.0
                terrain.color_grid[i][j] = [0.6, 0.3, 0.3]

terrain.update_grid()
scene.add_object("terrain", terrain)

navmesh = NavMesh(max_slope=55, max_step=0.6)
navmesh.build_from_mesh(terrain)
scene.add_object("navmesh", navmesh)

scene.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        label="Click Start, then End. Red = obstacle blocks (unwalkable). Green wireframe = navmesh.",
    ),
)

scene.run()
