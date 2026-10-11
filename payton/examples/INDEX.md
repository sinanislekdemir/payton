# Bundled Payton examples

A curated, self-contained subset of the repository examples ships inside the
`payton` package so AI assistants and tooling can read real, working code.

Access them from Python or the command line:

```python
import payton.ai

payton.ai.examples()                          # -> ["basics_01_scene.py", ...]
print(payton.ai.example("basics_04_clock"))   # -> source text
```

```bash
python -m payton --examples              # list
python -m payton --example basics_04_clock   # print one
```

These files are verbatim copies of the repository examples (see the source
column). The full set lives at
<https://github.com/sinanislekdemir/payton/tree/master/examples>.

| Bundled file | Source | What it shows |
| --- | --- | --- |
| `basics_01_scene.py` | `examples/basics/01_scene.py` | Minimal scene: an empty interactive 3D window |
| `basics_03_cubes.py` | `examples/basics/03_cubes.py` | Adding several objects to a scene |
| `basics_04_clock.py` | `examples/basics/04_clock.py` | Animating with clocks (projectile motion) |
| `basics_07_picking.py` | `examples/basics/07_picking.py` | Selecting objects with the mouse |
| `basics_10_vertex_colors.py` | `examples/basics/10_vertex_colors.py` | Per-vertex colours on a mesh |
| `basics_11_particle_system.py` | `examples/basics/11_particle_system.py` | Billboard particle system |
| `basics_12_collision.py` | `examples/basics/12_collision.py` | Collision detection with CollisionTest |
| `basics_13_plane.py` | `examples/basics/13_plane.py` | Adding a plane to the scene |
| `basics_15_gui.py` | `examples/basics/15_gui.py` | HUD, Window, Button, Slider and EditBox GUI |
| `basics_16_keyboard.py` | `examples/basics/16_keyboard.py` | Custom keyboard shortcuts |
| `basics_17_line.py` | `examples/basics/17_line.py` | Line objects |
| `basics_20_background.py` | `examples/basics/20_background.py` | Changing the scene background |
| `basics_21_click_plane.py` | `examples/basics/21_click_plane.py` | Cursor to world coordinates (click plane) |
| `basics_32_mesh_plane.py` | `examples/basics/32_mesh_plane.py` | A grid mesh plane |
| `basics_36_world_to_screen.py` | `examples/basics/36_world_to_screen.py` | World-to-screen projection |
| `basics_38_bouncingball.py` | `examples/basics/38_bouncingball.py` | Physics: a bouncing ball |
| `basics_39_capsule.py` | `examples/basics/39_capsule.py` | Capsule geometry |
| `basics_41_fog.py` | `examples/basics/41_fog.py` | Atmospheric fog |
| `basics_42_multi_shadow.py` | `examples/basics/42_multi_shadow.py` | Multiple shadow-casting lights |
| `basics_43_navmesh.py` | `examples/basics/43_navmesh.py` | NavMesh A* pathfinding |
| `basics_46_scene2d.py` | `examples/basics/46_scene2d.py` | 2D side-view Scene2D |
| `physics_01_internal_hello.py` | `examples/physics/01_internal_hello.py` | Built-in physics: hello world |
| `physics_03_internal_shapes.py` | `examples/physics/03_internal_shapes.py` | Built-in physics: boxes, spheres, capsules |
| `physics_04_collision_approximation.py` | `examples/physics/04_collision_approximation.py` | Choosing a collision approximation |
| `mid_level_shader.py` | `examples/mid-level/shader.py` | A custom GLSL shader |
| `mid_level_ripple.py` | `examples/mid-level/ripple.py` | Animated mesh grid (ripple) |
| `tools_01_extrude_line.py` | `examples/tools/01_extrude_line.py` | Extrude a line into a mesh |
| `tools_06_sweep.py` | `examples/tools/06_sweep.py` | Sweep a profile along a path |
| `tools_13_csg.py` | `examples/tools/13_csg.py` | Constructive solid geometry (CSG) |
