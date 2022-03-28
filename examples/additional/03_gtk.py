import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from payton.scene import Scene
from payton.scene.gtk import PaytonGTKSceneArea

scene = Scene()
window = Gtk.Window(title="Hello GTK")
window.set_default_size(800, 500)
payton_area = PaytonGTKSceneArea(scene)
window.add(payton_area)
window.connect("destroy", Gtk.main_quit)
window.connect("key-press-event", payton_area.key_press)
window.show_all()

Gtk.main()
