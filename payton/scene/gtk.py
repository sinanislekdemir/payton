try:
    import gi

    gi.require_version('Gtk', '3.0')
    from gi.repository import Gdk, GObject, Gtk

    _GTK_SUPPORTED = True
except ModuleNotFoundError:
    _GTK_SUPPORTED = False

from payton.scene import Scene
from payton.scene.gtkcontroller import GTKController

if _GTK_SUPPORTED:

    class PaytonGTKSceneArea(Gtk.EventBox):
        """Instance of GTk GLArea."""

        def __init__(self, scene: Scene):
            """Initialize the GLArea with the Payton Scene."""
            Gtk.EventBox.__init__(self)
            self._gl_area = Gtk.GLArea()

            self._gl_area.connect("realize", self.on_realize)
            self._gl_area.connect("render", self.render)
            self._gl_area.connect("resize", self.resize)

            self.connect("button-press-event", self.button_press)
            self.connect("button-release-event", self.button_release)
            self.connect("motion-notify-event", self.motion_notify)

            self._gl_area.set_has_depth_buffer(True)
            self._gl_area.set_has_stencil_buffer(True)
            self.add(self._gl_area)
            self.scene = scene
            self.controller = GTKController(self.scene)

        def render(self, area, ctx):
            """Render the scene."""
            ctx.make_current()
            area.make_current()
            self.scene._render()
            area.queue_draw()
            return True

        def resize(self, area, width: int, height: int):
            """Resize the widget."""
            self.scene.window_width = width
            self.scene.window_height = height
            for ob in self.scene.cameras:
                ob.aspect_ratio = width / height
                ob._viewport_size = [width, height, 0]
            for hud in self.scene.huds:
                self.scene.huds[hud].set_size(width, height)

        def on_realize(self, area, *args):
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

        def key_press(self, widget: Gtk.Window, event: Gdk.EventKey):
            """Handle key press."""
            self.controller.keyboard_press(event)

        def key_release(self, widget: Gtk.Window, event: Gdk.EventKey):
            pass

        def button_press(self, widget: Gtk.Window, event: Gdk.EventButton):
            """Handle button press."""
            print(widget)
            print(event.type)
            print(event.x, event.y)

        def button_release(self, widget: Gtk.Window, event: Gdk.EventButton):
            """Handle button release event."""
            pass

        def motion_notify(self, widget: Gtk.Window, event: Gdk.EventMotion):
            """Handle mouse motion."""
            print(widget)
            print(event)


else:

    class PaytonGTKSceneArea:
        def __init__(self, scene: Scene):
            raise "PyGTK can not be initialized"
