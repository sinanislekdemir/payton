from payton.scene.geometry import RagDoll
from payton.scene import Scene

scene = Scene()
ragdoll = RagDoll()

scene.add_object("ragdoll", ragdoll)
scene.run()
