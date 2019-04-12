import math
from payton.scene import Scene
from payton.scene.geometry import Sphere

def motion(name, scene, period, total):
    particle_position = (scene
                         .objects['nucleus']
                         .children['particle'].get_position())
    sub_position = (scene.objects['nucleus']
                    .children['particle']
                    .children['sub_particle'].get_position())
    angle = (total * 10) % 360
    px = math.cos(math.radians(angle)) * 8
    py = math.sin(math.radians(angle)) * 8
    scene.objects['nucleus'].children['particle'].set_position([px, py, 0])
    
    sx = math.cos(math.radians(angle * 10)) * 2 # 10 times faster
    sy = math.sin(math.radians(angle * 10)) * 2
    (scene
     .objects['nucleus']
     .children['particle']
     .children['sub_particle']
     .set_position([sx, sy, 0]))
    scene.lights[0].set_position([px, py, 0])

space = Scene()
space.observers[0].position = [20, 20, 20]
space.grid.resize(40, 40, 1)

nucleus = Sphere(radius=5)
particle = Sphere()
particle.set_position([8, 0, 0])

sub_particle = Sphere(radius=0.5)
sub_particle.set_position([0, 2, 0])

nucleus.add_child('particle', particle)
particle.add_child('sub_particle', sub_particle)

space.add_object('nucleus', nucleus)

space.create_clock("motion", 0.01, motion)
print("Hit SPACE to continue animation")
space.run()
