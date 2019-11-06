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
