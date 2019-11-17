from payton.scene.gui.base import Hud
from payton.scene.gui.window import Button, Window, WindowAlignment


def help_win():
    text = """Payton Shortcuts and Mouse Controls:

MOUSE:
Zoom In-Out: Left CTRL + Mouse Drag (Up - Down)
Rotate: Left SHIFT + Mouse Drag (Left - Right)
KEYBOARD:
ESC: Quit
H: Show/Hide Help
C: Change Camera mode (Perspective/Orthographic)
Space: Start/Pause Scene Animations/Clocks
G: Show/Hide Grid
W: Display mode (wireframe/solid)
F2: Previous Observer F3: Next Observer
    """
    help_window = Window(
        align=WindowAlignment.RIGHT,
        title="How to use Payton Scene",
        height=100,
        width=400,
    )
    help_window.add_child(
        "text", Button(width=400, height=400, left=10, top=40, label=text)
    )
    return help_window


def info_box(left: int, top: int, width: int, height: int, label: str) -> Hud:
    hud = Hud()
    window = Window(
        "Info", width=width, height=height + 60, left=left, top=top
    )

    window.add_child(
        "label",
        Button(
            label=label, width=width - 20, height=height - 40, top=40, left=10
        ),
    )
    window.add_child(
        "close",
        Button(
            label="CLOSE",
            left=int(width / 2) - 10,
            top=height + 10,
            width=int(width / 2),
            height=40,
            on_click=window.hide,
        ),
    )
    hud.add_child("window", window)
    return hud
