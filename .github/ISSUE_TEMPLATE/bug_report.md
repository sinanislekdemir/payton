---
name: Bug report
about: Report a bug in Payton 3D SDK (rendering, input/camera controls, assets, examples, install)
title: "[Bug]: "
labels: "bug"
assignees: ""

---

## Summary
A clear and concise description of what the bug is.

## Environment
- Payton version: (e.g. `pip show payton` output)
- Python version: (e.g. 3.10.x)
- OS: (Windows / macOS / Linux + version)
- Installation method: (pip / editable install / from source)
- GPU / Driver (important for OpenGL issues):
- OpenGL version (if known):
- Backend (if applicable): SDL2 / GTK3

## Minimal Repro (code)
Please provide the smallest possible script that reproduces the issue.

```python
from payton.scene import Scene

scene = Scene()
# TODO: add the minimal objects / settings that trigger the bug
scene.run()
```

## Steps to reproduce
1. `pip install payton` (or your exact install steps)
2. Run: `python repro.py`
3. Interact / press keys: (e.g. rotate with RMB drag, zoom with wheel, press `W`, `G`, `C`, etc.)
4. Observe the issue

## Actual behavior
What happened?

## Expected behavior
What did you expect to happen?

## Logs / error output
Paste the full terminal output / stack trace.

```text
<paste logs here>
```

## Screenshots / videos
If this is a rendering, camera, GUI, picking, or animation issue, attach a screenshot or short screen recording.

## Additional context
- Does it reproduce on one of the examples in `examples/`? If yes, which one? (path + command)
- If the bug involves assets (OBJ/MD2/AWP3D/BVH), attach a minimal asset file (or a link) and mention the loader used.
