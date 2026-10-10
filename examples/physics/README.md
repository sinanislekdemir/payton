# Physics examples

Payton can simulate physics in two ways:

* **PyBullet** – a full external engine, used automatically when `pybullet` is
  installed (`pip install pybullet`).
* **Built-in engine** – a small, dependency-free rigid-body engine that ships
  with Payton. It is used automatically when PyBullet is *not* installed, and
  can be forced with `Scene(use_internal_physics=True)`.

Both engines read the **same object parameters**, so switching between them
does not require any changes to your scene code.

> **The built-in engine is pure Python on purpose** — keeping Payton
> dependency-free was a deliberate choice — so it **will fall short** of a
> native engine for demanding scenes (many bodies, fast impacts, joints,
> long stacks). **For better physics accuracy and performance, install
> PyBullet** and let Payton use it automatically.

## Quick start

```python
from payton.scene import Scene
from payton.scene.geometry import Cube, Plane

scene = Scene(use_internal_physics=True)   # omit to use PyBullet when available
scene.add_object("ground", Plane(width=20, height=20))  # static (mass = 0)

cube = Cube()
cube.mass = 1                              # positive mass => dynamic body
cube.position = [0, 0, 5]
scene.add_object("cube", cube)

scene.run(start_clocks=True)
```

## Supported shapes

The built-in engine simulates **solid convex objects** with full rigid-body
collision response (linear + angular motion, friction, restitution and
damping):

| Object              | Collision shape |
|---------------------|-----------------|
| `Cube`              | oriented box    |
| `Sphere`            | sphere          |
| `Capsule`           | capsule (local Z axis) |
| `Plane`             | infinite plane  |
| any other `Mesh`    | box built from the mesh bounds |

For arbitrary meshes you can choose a simpler approximation:

```python
car.collision_approximation = "box"        # "auto" | "box" | "sphere" | "capsule"
person.collision_approximation = "capsule"
```

## Common parameters

* `obj.mass` – `0` (default) is static, anything greater is dynamic.
* `obj.linear_velocity = [x, y, z]` – initial / current velocity.
* `obj.change_dynamics(restitution=..., lateralFriction=...,
  linearDamping=..., angularDamping=...)` – material properties.
* `obj.set_position(x, y, z, with_physics=True)` – teleport a body.

Point-to-point joints (`constraint_point`) are a PyBullet-only feature and are
ignored (with a warning) by the built-in engine.

## Examples

| File | Description |
|------|-------------|
| `01_internal_hello.py` | A single cube falling onto a plane |
| `02_internal_cubes.py` | Mirror of `examples/basics/37_bullet_cubes.py` — same interlocking cubes, built-in engine (good for side-by-side comparison) |
| `03_internal_shapes.py` | Boxes, spheres and capsules together |
| `04_collision_approximation.py` | Approximating a custom mesh as a box or sphere |
| `05_enclosed_balls.py` | Random balls raining into a wireframed arena (drops up to 1000) |

Run any of them with, for example:

```bash
python examples/physics/01_internal_hello.py
```

Every example also accepts a `--bullet` flag to run the *same* scene on PyBullet
instead of the built-in engine (requires `pip install pybullet`), which makes
side-by-side comparison easy:

```bash
python examples/physics/02_internal_cubes.py            # built-in engine
python examples/physics/02_internal_cubes.py --bullet   # PyBullet
```

(For example `04`, note that `collision_approximation` is a built-in-engine
feature — PyBullet uses its own mesh collision shape.)
