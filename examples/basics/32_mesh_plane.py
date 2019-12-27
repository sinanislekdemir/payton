import math

from payton.scene import Scene
from payton.scene.geometry import MatrixPlane


class App(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.matrix_plane = MatrixPlane(
            width=20, height=20, x=60, y=60, static=False
        )
        self.add_object("matrix", self.matrix_plane)
        self.background.top_color = [0, 0, 0, 1]
        self.background.bottom_color = [0, 0, 0, 1]

        ranges = [
            [1, 1, 1],  # 0
            [1, 0, 0],  # 1
            [1, 1, 0],  # 2
            [0, 1, 1],  # 3
            [0, 0, 1],  # 4+
        ]
        self.active_observer.distance_to_target(40)
        for i in range(60):
            for j in range(60):
                self.matrix_plane.grid[i][j] = math.sin(
                    math.radians(i * 18)
                ) + math.cos(math.radians(j * 18))
                for t in range(5):
                    if t - 2 >= self.matrix_plane.grid[i][j]:
                        dist = self.matrix_plane.grid[i][j] - t
                        if dist == 0:
                            dist = 0.00001
                        xdist = (ranges[t][0] - ranges[t - 1][0]) / dist
                        ydist = (ranges[t][1] - ranges[t - 1][1]) / dist
                        zdist = (ranges[t][2] - ranges[t - 1][2]) / dist
                        self.matrix_plane.color_grid[i][j] = [
                            ranges[t - 1][0] + xdist,
                            ranges[t - 1][1] + ydist,
                            ranges[t - 1][2] + zdist,
                        ]
                        break

        self.matrix_plane.update_grid()


app = App()
app.run()
