# AGENTS.md — Payton 3D Graphics Toolkit

## Build & Verify

**Always run these before submitting code:**

- `make lint` — mypy (strict: `--disallow-untyped-calls --disallow-untyped-defs --disallow-incomplete-defs`) + ruff check
- `make fmt` — ruff check --fix + ruff format

Both **must** pass cleanly. If mypy fails, check `MYPYPATH` for SDL stubs.

Other targets: `make requirements` (pip install), `make development` (editable install), `make clean` (remove `__pycache__`).

## Project Structure

```
payton/                          # Main package
├── __init__.py                  # Disables OpenGL error checking
├── math/                        # Pure-Python math (no numpy needed for math)
│   ├── functions.py             # Rotations, projections, cross/dot
│   ├── geometry.py              # Raycasting, distance, intersection
│   ├── matrix.py                # 4×4 matrix ops
│   ├── types.py                 # Type aliases
│   └── vector.py                # Vector3D = List[float], Vector2D = List[float]
├── scene/                       # Core engine
│   ├── scene.py                 # Scene — central orchestrator, render loop, owns objects/cameras/lights/clocks/HUDs
│   ├── camera.py                # Camera (perspective/orthographic), orbit/pan/zoom
│   ├── clock.py                 # Clock(threading.Thread) periodic timer
│   ├── collision.py             # CollisionTest (AABB/sphere)
│   ├── controller.py            # Event dispatch chain: GUIController + SceneController
│   ├── light.py                 # Point light with shadow cubemap
│   ├── material.py              # Material — color, texture, display mode, PBR props
│   ├── shader.py                # Shader compile, uniform setters, inline GLSL
│   ├── grid.py                  # Ground-plane grid
│   ├── physics.py               # Optional pybullet physics
│   ├── theme.py                 # SceneTheme presets
│   ├── receiver.py              # Receiver base (Scene inherits)
│   ├── font/                    # Embedded fonts
│   ├── geometry/                # 3D geometry types
│   │   ├── base.py              # Object — root 3D class, matrix/VAO/buffer/hierarchy
│   │   ├── mesh.py              # Mesh — arbitrary triangle mesh
│   │   ├── cube.py, sphere.py, cylinder.py, capsule.py, plane.py
│   │   ├── particle.py          # ParticleSystem (billboard point cloud)
│   │   ├── wavefront.py         # OBJ loader
│   │   ├── md2.py               # Quake II model loader
│   │   ├── awp3d.py             # Animated zip-of-OBJ loader
│   │   ├── ragdoll.py           # RagDoll/Joint physics skeleton
│   │   └── export.py            # OBJ/MTL export
│   └── gui/                     # 2D overlay system
│       ├── base.py              # Hud, Shape2D, Rectangle, Text
│       └── window.py            # Window, Button, EditBox, ProgressBar, Slider, Panel, Theme
└── tools/                       # Utilities
    ├── bar.py                   # Terminal progress bar
    └── mesh/                    # Mesh manipulation (CSG, extrude, loft, sweep, subdivide, decimate, etc.)
plugins/
└── Blender 2.8-3.0/ & 4.x-5.x/ # Blender add-on exporters for AWP3D format
examples/
├── basics/                      # 41 numbered examples
├── mid-level/                   # Balloon, quake2, shader, RPG, ripple, engrave
├── high-level/                  # Cyberarena, heightmap, multiplayer
├── additional/                  # Bullet physics, GTK
├── designer/                    # Scene designer GUI
├── tools/                       # Mesh tool examples
└── PyFRP/                       # First-person RPG demo
```

## Core Architecture

- **Scene** is central: owns `objects` dict (name → Object), `cameras`, `lights`, `clocks`, `huds`, `collisions`.
- **Object** is base class for all 3D things: `position`/`rotation`/`scale` (4×4 matrix), VAO/VBO buffers, children hierarchy, optional bullet physics body.
- **Controller chain**: holds `GUIController` + `SceneController`; each can consume events (`stop_action = True`).
- **Clock** runs in its own thread, invokes a callback at a fixed interval.
- **Material** controls color, texture (Pillow), display mode (solid/wireframe/points), alpha, PBR.
- **Shader** compiles inline GLSL; supports default, particle, depth/shadow, background passes.
- **Math** has no external deps — uses `List[float]` for vectors, nested lists for matrices.

## Coding Conventions

- **Python ≥3.11**, no `from __future__ import annotations`
- **Imports**: stdlib → blank → third-party → blank → internal (absolute paths: `from payton.math.vector import Vector3D`)
- **Imports**: one per line in parentheses for multi-symbol imports from same module
- **Optional deps** (pybullet): guarded with `try/except ModuleNotFoundError`
- **Naming**: `CamelCase` classes, `snake_case` functions/methods, `UPPER_SNAKE_CASE` constants, `_private` attrs
- **Type hints**: required on every function/method in main source code; `Optional[X]` not `X | None`; uses `List`, `Dict`, `Tuple` from `typing`. **Examples are exempt** — no type hints in example files.
- **Defaults**: every feature must have sensible defaults and be as easy to use as possible (zero-config philosophy).
- **Docstrings**: reStructuredText (Sphinx), Parameters/Returns sections with `-----` underlines
- **Properties**: `@property`/`@setter` for side-effectful access (position, material, visible)
- **Dataclasses**: `@dataclass` for simple containers (`PhysicsParams`, `SceneTheme`)
- **Serialization**: `to_dict()` / `from_dict()` on major classes
- **GL constants**: imported individually, no `from OpenGL.GL import *`
- **Sentinel constants**: `NO_VERTEX_ARRAY = -1`, `NO_INDICE = -2`, `EMPTY_VERTEX_ARRAY = -3`
- **Complexity**: `# pylama:ignore=C901` on modules with complex methods is OK

## Dependencies (from pyproject.toml)

Pillow, PyOpenGL, pyrr, PySDL2, Cython, numpy (all required). Python ≥3.11.

## Key Files for Reference

| File | Purpose |
|---|---|
| `payton/scene/scene.py` | Scene class — entry point for rendering |
| `payton/scene/geometry/base.py` | Object class — base for all 3D objects |
| `payton/scene/controller.py` | Event dispatch architecture |
| `payton/scene/shader.py` | Shader compilation and GLSL sources |
| `payton/scene/material.py` | Material system and color constants |
| `payton/scene/camera.py` | Camera projection and navigation |
| `payton/math/matrix.py` | 4×4 matrix operations |
