"""Built-in physics: choosing a simple collision shape for a complex mesh.

The built-in engine only simulates simple convex shapes.  For an arbitrary
mesh you can say *how* it should be approximated with
``obj.collision_approximation``:

* ``"auto"``    -- box built from the mesh bounds (the default for meshes)
* ``"box"``     -- oriented box
* ``"sphere"``  -- bounding sphere
* ``"capsule"`` -- capsule aligned with the object's local Z axis

Here the same custom mesh is dropped twice: once approximated as a box and once
as a sphere.  Notice how the sphere approximation floats higher because it
bounds the whole mesh.

Run with ``--bullet`` to use PyBullet instead (requires ``pip install pybullet``).
Note that ``collision_approximation`` is a built-in-engine feature; PyBullet uses
its own (mesh) collision shape::

    python examples/physics/04_collision_approximation.py --bullet

Controls
--------
* Space             : pause / resume physics
* Middle mouse drag : pan the camera
* Mouse wheel       : zoom
"""

import sys

from payton.scene import Scene
from payton.scene.geometry import Mesh, Plane
from payton.scene.gui import Hud, Text


def make_pyramid() -> Mesh:
    """A small four-sided pyramid built from triangles."""
    mesh = Mesh()
    p0 = [-1.0, -1.0, 0.0]
    p1 = [1.0, -1.0, 0.0]
    p2 = [0.0, 1.0, 0.0]
    p3 = [0.0, 0.0, 2.0]
    mesh.add_triangle([p0, p1, p3])
    mesh.add_triangle([p1, p2, p3])
    mesh.add_triangle([p2, p0, p3])
    mesh.add_triangle([p0, p2, p1])
    return mesh


# ``--bullet`` opts into PyBullet; without it the built-in engine is used.
use_bullet = "--bullet" in sys.argv

scene = Scene(use_internal_physics=not use_bullet)
scene.lights[0].position = [20, 20, 40]
scene.active_camera.position = [8, 8, 8]

scene.add_object("ground", Plane(width=30, height=30))

box_pyramid = make_pyramid()
box_pyramid.collision_approximation = "box"
box_pyramid.mass = 1
box_pyramid.position = [-2, 0, 5]
box_pyramid.material.color = [0.9, 0.4, 0.3]
scene.add_object("box_pyramid", box_pyramid)

sphere_pyramid = make_pyramid()
sphere_pyramid.collision_approximation = "sphere"
sphere_pyramid.mass = 1
sphere_pyramid.position = [2, 0, 5]
sphere_pyramid.material.color = [0.3, 0.6, 0.9]
scene.add_object("sphere_pyramid", sphere_pyramid)

hud = Hud()
hud.add_child(
    "text",
    Text(
        label="box vs sphere approximation - press Space to pause",
        position=[5, 5, 1],
        size=[380, 35],
        color=[1, 1, 1],
    ),
)
scene.add_object("hud", hud)

scene.run(start_clocks=True)
