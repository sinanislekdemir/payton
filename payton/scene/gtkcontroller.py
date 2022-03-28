import logging

try:
    import gi

    gi.require_version('Gtk', '3.0')
    from gi.repository import Gdk, Gtk

    _GTK_SUPPORTED = True
except ModuleNotFoundError:
    _GTK_SUPPORTED = False


class GTKController:
    def __init__(self, scene):
        self.shift_down = False
        self.ctrl_down = False
        self.scene = scene

    def keyboard_release(self, event):
        keyval = event.keyval
        if keyval == Gdk.KEY_Shift_L:
            self.shift_down = False

    def keyboard_press(self, event) -> int:
        keyval = event.keyval
        print(keyval == Gdk.KEY_h)
        if keyval == Gdk.KEY_Escape:
            self.scene.terminate()
            return 2

        if keyval == Gdk.KEY_Shift_L:
            self.shift_down = True

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

        return 0
