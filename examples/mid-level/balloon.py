import random
from payton.scene import Scene
from payton.scene.geometry import Sphere
from payton.scene.gui import Hud, Text
from payton.scene.material import YELLOW, RED

sphere_count = 0
score_board = 0
game = Scene()


def create_balloon(name, scene, period, total):
    global sphere_count
    x = random.randint(-5, 5)
    y = random.randint(-5, 5)
    sphere = Sphere()
    sphere.material.color = [1, 1, 1]
    sphere.position = [x, y, 10]
    scene.add_object(f"sphere_{sphere_count}", sphere)
    sphere_count += 1


def move_balloons(name, scene, period, total):
    global sphere_count
    for k in range(sphere_count):
        sphere_name = f"sphere_{k}"
        if sphere_name in scene.objects:
            if not scene.objects[sphere_name].visible:
                continue
            pos = scene.objects[sphere_name].position
            pos[2] -= 0.01
            scene.objects[sphere_name].position = pos
            if pos[2] < 0:
                status = Text(
                    label="GAME OVER!!!",
                    position=(200, 300),
                    color=(1, 1, 1),
                    bgcolor=(1, 0, 0, 1),
                )
                scene.clocks["balloon-creator"].pause()
                scene.clocks["move-balloons"].pause()
                scene.huds["hud"].add_child("status", status)
            if pos[2] < 6:
                scene.objects[sphere_name].material.color = YELLOW
            if pos[2] < 3:
                scene.objects[sphere_name].material.color = RED


def select(list):
    global game
    global score_board
    for obj in list:
        obj.hide()
        score_board += 1
        game.huds["hud"].children[
            "score"
        ].label = f"Number of balloons popped: {score_board}"


game.create_clock("balloon-creator", 1, create_balloon)
game.create_clock("move-balloons", 0.01, move_balloons)
game.on_select = select
hud = Hud()

text = Text(
    label="Hit Space to start popping the balloons!",
    position=(10, 10),
    color=(1, 1, 1),
    size=(300, 100),
)

hud.add_child("info", text)

score = Text(
    label="Number of balloons popped: 0",
    position=(10, 40),
    color=(1, 1, 1),
    size=(300, 100),
)

hud.add_child("score", score)

game.add_object("hud", hud)
game.run()
