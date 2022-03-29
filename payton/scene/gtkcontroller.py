import logging
from typing import Any

try:
    import gi

    gi.require_version('Gtk', '3.0')
    from gi.repository import Gdk

    _GTK_SUPPORTED = True
except ModuleNotFoundError:
    _GTK_SUPPORTED = False
from payton.scene.camera import BUTTON_LEFT, BUTTON_MIDDLE, BUTTON_RIGHT


class GTKController:
    """GTK controller for eventbox."""

    def __init__(self, scene: Any) -> None:
        """Initialize GTK controller."""
        self.shift_down = False
        self.ctrl_down = False
        self.scene = scene
        self.left_button_down = False
        self.right_button_down = False
        self.middle_button_down = False
        self.xrel = None
        self.yrel = None

    def keyboard_release(self, event: Any) -> None:
        """Release keyboard."""
        keyval = event.keyval
        if keyval == Gdk.KEY_Shift_L:
            self.shift_down = False
        if keyval == Gdk.KEY_Control_L:
            self.shift_down = False

    def button_press(self, event: Any) -> None:
        """Press mouse button."""
        if event.button == 1:
            self.left_button_down = True
            camera = self.scene.active_camera
            mx, my = event.x, event.y
            eye, ray_dir = camera.screen_to_world(mx, my, self.scene.window_width, self.scene.window_height)
            if callable(self.scene.on_select):
                list = []
                for obj in self.scene.objects:
                    hit = self.scene.objects[obj].select(eye, ray_dir)
                    if hit:
                        list.append(self.scene.objects[obj])
                if list:
                    self.scene.on_select(list)
            if not (self.shift_down or self.ctrl_down):
                self.scene._check_click_plane(eye, ray_dir)

        if event.button == 2:
            self.middle_button_down = True
        if event.button == 3:
            self.right_button_down = True

    def button_release(self, event: Any) -> None:
        """Relese the mouse button."""
        if event.button == 1:
            self.left_button_down = False
        if event.button == 2:
            self.middle_button_down = False
        if event.button == 3:
            self.right_button_down = False

    def scroll(self, event: Any) -> None:
        """Mouse scroll event."""
        self.scene.active_camera.mouse_wheel(event.delta_y)

    def mouse_move(self, event: Any) -> None:
        """Mouse move."""
        camera = self.scene.active_camera
        button = None
        if self.right_button_down:
            button = BUTTON_RIGHT
        if self.middle_button_down:
            button = BUTTON_MIDDLE
        if self.left_button_down:
            button = BUTTON_LEFT
        if self.xrel is None:
            self.xrel = event.x
        if self.yrel is None:
            self.yrel = event.y
        xrel = event.x - self.xrel
        yrel = event.y - self.yrel
        self.xrel = event.x
        self.yrel = event.y
        if not button:
            return

        camera.mouse_move(
            button,
            self.shift_down,
            self.ctrl_down,
            event.x,
            event.y,
            xrel,
            yrel,
            self.scene.window_width,
            self.scene.window_height,
        )

    def keyboard_press(self, event: Any) -> int:  # noqa: C901
        """Keyboard press."""
        keyval = event.keyval
        print(keyval == Gdk.KEY_h)
        if keyval == Gdk.KEY_Escape:
            self.scene.terminate()
            return 2

        if keyval == Gdk.KEY_Shift_L:
            self.shift_down = True

        if keyval == Gdk.KEY_Control_L:
            self.ctrl_down = True

        if keyval == Gdk.KEY_c:
            p = self.scene.active_camera.perspective
            self.scene.active_camera.perspective = not p

        if keyval == Gdk.KEY_space:
            for clock in self.scene.clocks:
                c = self.scene.clocks[clock]
                logging.debug(f"Pause clock [{clock}]")
                c.pause()

        if keyval == Gdk.KEY_h:
            self.scene.huds['_help']._visible = not self.scene.huds['_help'].visible

        if keyval == Gdk.KEY_w:
            for obj in self.scene.objects:
                self.scene.objects[obj].toggle_wireframe()

        if keyval in [Gdk.KEY_F2, Gdk.KEY_F3]:
            active = 0
            for i in range(len(self.scene.cameras)):
                if self.scene.cameras[i].active:
                    active = i
            if keyval == Gdk.KEY_F2:
                active -= 1
            if keyval == Gdk.KEY_F3:
                active += 1
            if active < 0:
                active = len(self.scene.cameras) - 1
            if active > len(self.scene.cameras) - 1:
                active = 0
            self.scene.active_camera = self.scene.cameras[active]
            for i in range(len(self.scene.cameras)):
                self.scene.cameras[i].active = i == active

        return 0
