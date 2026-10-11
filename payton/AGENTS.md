# Payton — AI-ready agent guide

This is the agent-facing entry point that ships with the `payton` package. The
full, canonical context lives in [`llms.txt`](./llms.txt) next to this file.

If you are an AI assistant working in a project that depends on Payton, load the
full guide and the machine-readable API catalog before writing Payton code:

```bash
python -m payton              # print the full Markdown primer
python -m payton --catalog    # public API catalog (signatures + summaries) as JSON
python -m payton --examples   # list bundled, self-contained examples
python -m payton --version
```

Or from Python:

```python
import payton.ai

print(payton.ai.context())                    # full primer
api = payton.ai.catalog()                     # dict: modules -> symbols -> signature/summary
print(payton.ai.examples())                   # bundled example file names
print(payton.ai.example("basics_04_clock"))   # source of one bundled example
```

## Quick rules

* `from payton.scene import Scene` then `scene.run()` — that is a full app.
* Register everything on the `Scene`: `scene.add_object(name, obj)`,
  `scene.add_camera(cam)`, `scene.add_collision_test(name, test)`.
* 3D primitives live in `payton.scene.geometry` (`Cube`, `Sphere`, `Cylinder`,
  `Capsule`, `Plane`, `Line`, `Mesh`, `Wavefront`, `MD2`, `ParticleSystem`, ...).
* Coordinates are right-handed with **Z up**, 1 unit = 1 metre.
* Transform via `obj.position`, `obj.rotate_around_x/y/z(radians)`,
  `obj.scale(x, y, z)`. Vectors are mutable `List[float]`; assign a new list, do
  not mutate in place for transforms.
* Animate with `scene.create_clock(name, period, callback)` where
  `callback(period, total)`; call `scene.run(start_clocks=True)` to auto-start.
* Keep clocks fast and never issue OpenGL calls from them.
* Prefer the bundled examples (`examples/` in the repository) as ground truth.
