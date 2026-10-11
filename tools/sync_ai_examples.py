"""Copy a curated, self-contained subset of the repository examples into the
``payton`` package so they ship with the wheel.

Agents and editors can then read real, working Payton code without cloning the
repository:

    import payton.ai
    payton.ai.examples()                       # list of bundled examples
    print(payton.ai.example("basics_04_clock"))

Every curated example must be **self-contained**: no external asset files, no
network, no GUI toolkit other than Payton's own dependencies. Maintained by
``make ai``.

Usage::

    python tools/sync_ai_examples.py
"""

from pathlib import Path

# (source path relative to the repo root, one-line label)
CURATED: list[tuple[str, str]] = [
    ("examples/basics/01_scene.py", "Minimal scene: an empty interactive 3D window"),
    ("examples/basics/03_cubes.py", "Adding several objects to a scene"),
    ("examples/basics/04_clock.py", "Animating with clocks (projectile motion)"),
    ("examples/basics/07_picking.py", "Selecting objects with the mouse"),
    ("examples/basics/10_vertex_colors.py", "Per-vertex colours on a mesh"),
    ("examples/basics/11_particle_system.py", "Billboard particle system"),
    ("examples/basics/12_collision.py", "Collision detection with CollisionTest"),
    ("examples/basics/13_plane.py", "Adding a plane to the scene"),
    ("examples/basics/15_gui.py", "HUD, Window, Button, Slider and EditBox GUI"),
    ("examples/basics/16_keyboard.py", "Custom keyboard shortcuts"),
    ("examples/basics/17_line.py", "Line objects"),
    ("examples/basics/20_background.py", "Changing the scene background"),
    ("examples/basics/21_click_plane.py", "Cursor to world coordinates (click plane)"),
    ("examples/basics/32_mesh_plane.py", "A grid mesh plane"),
    ("examples/basics/36_world_to_screen.py", "World-to-screen projection"),
    ("examples/basics/38_bouncingball.py", "Physics: a bouncing ball"),
    ("examples/basics/39_capsule.py", "Capsule geometry"),
    ("examples/basics/41_fog.py", "Atmospheric fog"),
    ("examples/basics/42_multi_shadow.py", "Multiple shadow-casting lights"),
    ("examples/basics/43_navmesh.py", "NavMesh A* pathfinding"),
    ("examples/basics/46_scene2d.py", "2D side-view Scene2D"),
    ("examples/physics/01_internal_hello.py", "Built-in physics: hello world"),
    ("examples/physics/03_internal_shapes.py", "Built-in physics: boxes, spheres, capsules"),
    ("examples/physics/04_collision_approximation.py", "Choosing a collision approximation"),
    ("examples/mid-level/shader.py", "A custom GLSL shader"),
    ("examples/mid-level/ripple.py", "Animated mesh grid (ripple)"),
    ("examples/tools/01_extrude_line.py", "Extrude a line into a mesh"),
    ("examples/tools/06_sweep.py", "Sweep a profile along a path"),
    ("examples/tools/13_csg.py", "Constructive solid geometry (CSG)"),
]

HEADER = """# Bundled Payton examples

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
"""


def dest_name(source: str) -> str:
    """Turn ``examples/basics/01_scene.py`` into ``basics_01_scene.py``."""
    parts = Path(source).with_suffix("").parts
    category = parts[-2].replace("-", "_")
    return f"{category}_{parts[-1]}.py"


def main() -> int:
    root = Path(__file__).resolve().parent.parent
    dest_dir = root / "payton" / "examples"
    dest_dir.mkdir(parents=True, exist_ok=True)

    expected: set[str] = set()
    rows: list[tuple[str, str, str]] = []

    for source, label in CURATED:
        src = root / source
        if not src.exists():
            print(f"WARNING: missing source example {source}")
            continue
        name = dest_name(source)
        expected.add(name)
        (dest_dir / name).write_text(src.read_text(encoding="utf-8"), encoding="utf-8")
        rows.append((name, source, label))

    # Remove stale files from earlier runs.
    for existing in dest_dir.glob("*.py"):
        if existing.name not in expected:
            existing.unlink()

    index = HEADER + "".join(
        f"| `{name}` | `{source}` | {label} |\n" for name, source, label in rows
    )
    (dest_dir / "INDEX.md").write_text(index, encoding="utf-8")

    print(f"Synced {len(rows)} examples into {dest_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
