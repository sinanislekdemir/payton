from payton.scene import Scene
from payton.scene.geometry import Cube, Sphere
from payton.tools.mesh.geometry import merge_mesh

c = Cube()
c.position = [2, 2, 2]
s = Sphere()

mm = merge_mesh(c, s)

scene = Scene()
scene.add_object("mm", mm)
scene.run()
