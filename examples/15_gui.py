import math
from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui import Hud, Rectangle, Text


def rotate(name, scene, period, total):
    y = math.radians(period * 100)
    y = -y if int(total) % 2 == 0 else y

    scene.objects["cube"].rotate_around_x(math.radians(period * 50))
    scene.objects["cube"].rotate_around_y(y)
    scene.objects["cube"].rotate_around_z(math.radians(period * 150))


scene = Scene()
cube = Cube()

scene.observers[0].distance_to_target(3)

hud = Hud()
rectangle = Rectangle(position=(10, 20, 0), size=(300, 300))
rectangle.material.texture = "cube.png"
hud.add_child("rect", rectangle)

text = Text(label="Hello World!")
text.set_position((0, 0))
rectangle.add_child("label", text)
hud.set_font("/Library/Fonts/Arial.ttf")

scene.add_object("hud", hud)


cube.material.texture = "cube.png"
scene.add_object("cube", cube)

scene.create_clock("rotate", 0.01, rotate)

scene.run()
