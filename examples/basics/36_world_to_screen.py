from payton.scene import Scene
from payton.scene.geometry import Cube
from payton.scene.gui.help import object_box


class App(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.on_select = self.select
        cube = Cube()
        self.add_object("cube", cube)

    def select(self, obj_list):
        """Select object and show info box."""
        pos = obj_list[0].matrix[3]
        screen_coordinates = self.active_camera.world_to_screen(pos)
        self.add_object("ObjectInfo", object_box(screen_coordinates[0], screen_coordinates[1], obj_list[0]))


app = App()
app.run()
