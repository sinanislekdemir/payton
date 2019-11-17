import random

from payton.scene import Scene
from payton.scene.geometry import Sphere
from payton.scene.gui import info_box
from payton.scene.material import RED, YELLOW

sphere_count = 0
score_board = 0
game = Scene()


def create_balloon(period, total):
    global sphere_count, game
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    sphere = Sphere()
    sphere.material.color = [1, 1, 1]
    sphere.position = [x, y, 10]
    game.add_object(f"sphere_{sphere_count}", sphere)
    sphere_count += 1


def move_balloons(period, total):
    global sphere_count, game
    for k in range(sphere_count):
        sphere_name = f"sphere_{k}"
        if sphere_name in game.objects:
            if not game.objects[sphere_name].visible:
                continue
            pos = game.objects[sphere_name].position
            pos[2] -= 0.1
            game.objects[sphere_name].position = pos
            if pos[2] < 0:
                game.add_object(
                    "game",
                    info_box(
                        left=10,
                        top=300,
                        width=220,
                        height=100,
                        label="Game Over!",
                    ),
                )
                game.clocks["balloon-creator"].pause()
                game.clocks["move-balloons"].pause()
            if pos[2] < 6:
                game.objects[sphere_name].material.color = YELLOW
            if pos[2] < 3:
                game.objects[sphere_name].material.color = RED


def select(list):
    global game
    global score_board
    for obj in list:
        if not obj.visible:
            continue
        obj.hide()
        score_board += 1


game.create_clock("balloon-creator", 1, create_balloon)
game.create_clock("move-balloons", 0.05, move_balloons)
game.on_select = select

game.add_object(
    "info",
    info_box(
        left=10,
        top=10,
        width=220,
        height=100,
        label="Hit SPACE to\nstart popping!",
    ),
)

game.run()
