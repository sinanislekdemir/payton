# Payton Designer Example

This example is a lightweight 3D object designer built with Payton.

It lets you:

- add primitive objects
- select objects in the scene
- edit position, rotation, scale, and color
- save the scene as JSON
- export the current scene as Wavefront OBJ

## Run

From the repository root:

```bash
python examples/designer/main.py
```

If you are setting up the repository locally first, install the dependencies from the project root:

```bash
pip install -r requirements.txt
```

## Controls

### Mouse

- **Left click object**: select object
- **Right mouse drag**: rotate camera
- **Middle mouse drag**: pan camera
- **Mouse wheel**: zoom

### Keyboard

- **1**: add cube
- **2**: add sphere
- **3**: add cylinder
- **4**: add plane
- **Delete / Backspace**: delete selected object
- **Ctrl+S**: save scene JSON
- **Ctrl+L**: load scene JSON
- **Ctrl+E**: export OBJ

Payton scene defaults are also available, including:

- **G**: show/hide grid
- **W**: change display mode
- **C**: toggle camera projection
- **H**: show/hide help
- **Escape**: quit

## UI workflow

The left panel is for scene actions:

- add primitives
- save/load the scene
- export OBJ
- view the current status message

The right panel is for the selected object:

- selected object name
- primitive type
- position
- rotation in degrees
- scale
- color as RGB values in the `0.0` to `1.0` range

To edit a value, click the field, type a new comma-separated triple, and press **Enter**.

Examples:

- Position: `1.0, 2.0, 0.5`
- Rotation: `0.0, 45.0, 90.0`
- Scale: `1.0, 1.0, 1.0`
- Color: `0.4, 0.7, 1.0`

## Files

By default, the example uses these filenames from the current working directory:

- scene save: `designer_scene.json`
- OBJ export: `designer_export.obj`

You can change both filenames from the left panel before saving or exporting.
