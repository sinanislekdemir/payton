import random

from payton.scene import Scene
from payton.scene.geometry import Sphere
from payton.scene.gui import info_box
from payton.scene.material import RED, YELLOW

sphere_count = 0
score_board = 0
game = Scene()
base_sphere = Sphere()


def create_balloons():
    global sphere_count
    for i in range(4):
        x = random.randint(-5, 5)
        y = random.randint(-5, 5)
        z = random.randint(10, 20)
        proxy = base_sphere.clone()
        proxy.position = [x, y, z]
        game.add_object(f"sphere_{sphere_count}", proxy)
        sphere_count += 1


def move_balloons(period, total):
    for k in range(sphere_count):
        sphere_name = f"sphere_{k}"
        if sphere_name in game.objects:
            if not game.objects[sphere_name].visible:
                continue
            pos = game.objects[sphere_name].position
            pos[2] -= 0.05
            game.objects[sphere_name].position = pos
            if pos[2] < 0:
                game.add_object(
                    "game", info_box(left=10, top=300, label="Game Over!",),
                )
                game.clocks["move-balloons"].pause()
            if pos[2] < 6:
                game.objects[sphere_name].material.color = YELLOW
            if pos[2] < 3:
                game.objects[sphere_name].material.color = RED


def select(list):
    # Re-use the existing objects
    global score_board
    for obj in list:
        x = random.randint(-5, 5)
        y = random.randint(-5, 5)
        z = random.randint(10, 20)
        obj.position = [x, y, z]
        score_board += 1


game.create_clock("move-balloons", 0.02, move_balloons)
game.on_select = select
create_balloons()

game.add_object(
    "info", info_box(left=10, top=10, label="Hit SPACE to start popping!",),
)

game.run()
