import math
import os

import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk

from payton.scene import Scene
from payton.scene.geometry import Cube, Plane
from payton.scene.gtk import PaytonGTKSceneArea

scene = Scene()
cube = Cube()
cube.position = [2, 1, 0.5]

ground = Plane(width=30, height=30)
wall1 = Plane(width=30, height=10)
wall1.rotate_around_x(math.radians(90))
wall1.position = [0, -10, 5]

scene.lights[0].position = [5.0, 5.0, 6.0]
cube_by_corners = Cube(from_corner=[-3, -3, 1], to_corner=[-1, -1, 3])

texture_file = os.path.join(os.path.dirname(__file__), "barrel.jpg")
cube_by_corners.material.texture = texture_file
ground.material.texture = texture_file

scene.add_object("wall", wall1)
scene.add_object("cube", cube)
scene.add_object("cube_by_corners", cube_by_corners)
scene.add_object("ground", ground)
scene.grid.visible = False

window = Gtk.Window(title="Hello GTK")
window.set_default_size(1024, 800)

store = Gtk.ListStore(str)
for obj in scene.objects:
    store.append([obj])

box = Gtk.Box()

paned_left = Gtk.Paned()
paned_left.set_size_request(300, -1)
paned_main = Gtk.Paned()

object_tree = Gtk.TreeView(model=store)
renderer = Gtk.CellRendererText()
column = Gtk.TreeViewColumn("Object name", renderer, text=0, weight=1)
object_tree.append_column(column)

paned_left.add(object_tree)

payton_area = PaytonGTKSceneArea(scene)
paned_main.add(payton_area)

box.pack_start(paned_left, False, True, 0)
box.pack_start(paned_main, True, True, 0)

window.add(box)
window.connect("destroy", Gtk.main_quit)
window.connect("key-press-event", payton_area.key_press)
window.show_all()

Gtk.main()
