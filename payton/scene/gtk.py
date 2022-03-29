from typing import Any

try:
    import gi

    gi.require_version('Gtk', '3.0')
    from gi.repository import Gdk, Gtk

    _GTK_SUPPORTED = True
except ModuleNotFoundError:
    _GTK_SUPPORTED = False

from payton.scene import Scene
from payton.scene.gtkcontroller import GTKController

if _GTK_SUPPORTED:  # noqa: C901

    class PaytonGTKSceneArea(Gtk.EventBox):
        """Instance of GTk GLArea."""

        def __init__(self, scene: Scene):
            """Initialize the GLArea with the Payton Scene."""
            Gtk.EventBox.__init__(self)
            self._gl_area = Gtk.GLArea()

            self._gl_area.connect("realize", self.on_realize)
            self._gl_area.connect("render", self.render)
            self._gl_area.connect("resize", self.resize)

            self.set_events(Gdk.EventMask.ALL_EVENTS_MASK)

            self.connect("button-press-event", self.button_press)
            self.connect("button-release-event", self.button_release)
            self.connect("motion-notify-event", self.motion_notify)
            self.connect("scroll-event", self.scroll)

            self._gl_area.set_has_depth_buffer(True)
            self._gl_area.set_has_stencil_buffer(True)
            self.add(self._gl_area)
            self.scene = scene
            self.controller = GTKController(self.scene)

        def render(self, area: Gtk.GLArea, ctx: Any) -> bool:
            """Render the scene."""
            ctx.make_current()
            area.make_current()
            self.scene._render()
            area.queue_draw()
            return True

        def scroll(self, widget: "PaytonGTKSceneArea", event: Gdk.EventScroll) -> None:
            """Scroll mouse."""
            self.controller.scroll(event)

        def resize(self, area: Gtk.GLArea, width: int, height: int) -> None:
            """Resize the widget."""
            self.scene.window_width = width
            self.scene.window_height = height
            for ob in self.scene.cameras:
                ob.aspect_ratio = width / height
                ob._viewport_size = [width, height, 0]
            for hud in self.scene.huds:
                self.scene.huds[hud].set_size(width, height)

        def on_realize(self, area: Gtk.GLArea) -> None:
            """Realize (initialize) the area."""
            area.make_current()
            self.scene.window_width = area.get_allocated_width()
            self.scene.window_height = area.get_allocated_height()

            self.scene.running = True
            self.scene._init_runtime()
            self.scene._context = area
            err = self._gl_area.get_error()
            if err:
                print("Something went wrong with the GTK:")
                print(err)
            else:
                print("Widget initialized.")

        def key_press(self, widget: Gtk.Window, event: Gdk.EventKey) -> None:
            """Handle key press."""
            self.controller.keyboard_press(event)

        def key_release(self, widget: Gtk.Window, event: Gdk.EventKey) -> None:
            """Handle key release."""
            self.controller.keyboard_release(event)

        def button_press(self, widget: Gtk.Window, event: Gdk.EventButton) -> None:
            """Handle button press."""
            self.controller.button_press(event)

        def button_release(self, widget: Gtk.Window, event: Gdk.EventButton) -> None:
            """Handle button release event."""
            self.controller.button_release(event)

        def motion_notify(self, widget: Gtk.Window, event: Gdk.EventMotion) -> None:
            """Handle mouse motion."""
            self.controller.mouse_move(event)

else:

    class PaytonGTKSceneArea:  # type: ignore
        """Payton GTK Scene for invalid GTK Imports."""

        def __init__(self, scene: Scene):
            """This shall fail."""
            raise BaseException("PyGTK can not be initialized")
