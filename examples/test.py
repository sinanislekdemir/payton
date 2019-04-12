from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube1 = Cube()
cube2 = Cube()
cube3 = Cube()
cube4 = Cube()

cube1.material.color = [1.0, 0.0, 0.0]
cube2.material.color = [0.0, 0.0, 1.0]
cube3.material.color = [1.0, 0.0, 0.0]
cube4.material.color = [0.0, 0.0, 1.0]

cube1.set_position([0, 0, 2])
cube2.set_position([1, 0, 2])
cube3.set_position([1, 0, 1])
cube4.set_position([0, 0, 1])

scene.add_object('cube1', cube1)
scene.add_object('cube2', cube2)
scene.add_object('cube3', cube3)
scene.add_object('cube4', cube4)

scene.observers[0].position = [0, 10, 1]
scene.create_observer()
scene.observers[1].position = [0, 300, 1]
scene.observers[1].far = 1000

scene.run()
