from PIL import Image, ImageDraw

from payton.scene.gui.base import Hud
from payton.scene.gui.window import Button, Window, WindowAlignment


def help_win():
    text = f"""Payton Shortcuts and Mouse Controls:

MOUSE:
Zoom In-Out: Mouse Wheel
Rotate: Right Click and Drag
Pan: Middle Click and Drag
KEYBOARD:
ESC: Quit
H: Show/Hide Help
C: Change Camera mode (Perspective/Orthographic)
Space: Start/Pause Scene Animations/Clocks
G: Show/Hide Grid
W: Display mode (wireframe/solid)
F2: Previous Observer F3: Next Observer
    """
    help_window = Window(align=WindowAlignment.RIGHT, title="How to use Payton Scene", height=10000, width=520,)
    help_window.add_child("text", Button(width=500, height=400, left=10, top=40, label=text))
    return help_window


def info_box(left: int, top: int, label: str) -> Hud:
    hud = Hud()
    timg = Image.new("RGBA", (1, 1))
    d = ImageDraw.Draw(timg)
    res = d.textsize(label, font=hud.font)
    width = res[0] + 60
    height = res[1] + 60

    window = Window("Info", width=width, height=height + 60, left=left, top=top)

    window.add_child(
        "label", Button(label=label, width=width - 20, height=height - 40, top=40, left=10),
    )
    window.add_child(
        "close",
        Button(
            label="CLOSE",
            left=int(width / 2) - 10,
            top=height + 10,
            width=int(width / 2),
            height=30,
            on_click=window.hide,
        ),
    )
    hud.add_child("window", window)
    return hud
