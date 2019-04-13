from payton.scene import Scene
from payton.scene.wavefront import Wavefront

scene = Scene()
monkey = Wavefront(filename='monkey.obj')

scene.add_object('monkey', monkey)
scene.run()
