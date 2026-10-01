<div align="center">

# Payton 3D

### From *"what if..."* to *"whoa, I just built that"* — in Python.

A batteries-included 3D graphics toolkit for **rapid prototyping, creative coding, simulation, data visualization, and play**. Real 3D, sane defaults, zero ceremony.

[![PyPI](https://img.shields.io/pypi/v/payton?color=3776AB&label=PyPI)](https://pypi.org/project/payton/)
[![Python](https://img.shields.io/pypi/pyversions/payton?color=3776AB)](https://pypi.org/project/payton/)
[![License](https://img.shields.io/pypi/l/payton?color=success)](https://github.com/sinanislekdemir/payton/blob/master/LICENSE)
[![Downloads](https://pepy.tech/badge/payton/month)](https://pepy.tech/project/payton)
[![Downloads](https://pepy.tech/badge/payton/week)](https://pepy.tech/project/payton)

**[Three lines to 3D](#-three-lines-to-3d)** · **[What's inside](#-whats-inside)** · **[Showcase](#-showcase)** · **[Examples](#-example-index)** · **[Install](#-install)**

</div>

https://github.com/user-attachments/assets/3904c4a3-d683-4816-a31d-3af958e42813

---

## ⚡ Three lines to 3D

```python
from payton.scene import Scene

scene = Scene()
scene.run()
```

That is a **live, interactive 3D scene**: a camera you can orbit, lighting, shadows, a ground grid, and a render loop — all configured for you. Want a spinning cube on top of that? Add four more lines:

```python
from payton.scene import Scene
from payton.scene.geometry import Cube

scene = Scene()
cube = Cube(width=2, depth=2, height=2)
cube.position = [0, 0, 1]
scene.add_object("cube", cube)
scene.run()
```

No engine bootstrapping. No asset pipeline. No 300-line "hello world". Just Python, and something beautiful on screen before your coffee gets cold.

---

## 🧭 Why Payton?

Python 3D libraries usually live at two extremes: **too low-level** (you hand-build cameras, lighting, materials, and scene management before you see a single triangle) or **too heavy** (you fight a production engine's learning curve for a weekend before you see results). Payton sits deliberately in the sweet spot between them.

| | The usual story | The Payton story |
| --- | --- | --- |
| **First render** | Hours of setup | Three lines |
| **Camera + lighting + shadows** | Build it yourself | Included |
| **Materials, GUI, physics, audio** | Glue five libraries together | Included, optional |
| **Learning curve** | Engine-specific patterns | Plain, Pythonic API |
| **Best for** | Shipping a AAA title | *Seeing* your idea right now |

**vs Pygame:** Pygame is a 2D library. The moment you want true 3D you must build camera orbit, lighting, shadows, materials, hierarchies, collision, and UI from scratch. Payton ships all of it, so you write your idea instead of your boilerplate.

**vs Panda3D:** Panda3D is a production engine with a C++-rooted architecture and a steep deployment curve. Payton is the opposite — three lines to a window, zero-config defaults, and a purely Pythonic feel. It's built for *prototyping, tool-building, visualization, simulation, and education*, where results in minutes beat polish in weeks. If your project outgrows Payton, moving on to Panda3D, Ursina, or Unity is the natural next step.

Payton bridges the gap between *"I need to see my 3D idea right now"* and *"I'm building a commercial game."* It shines when **speed-to-visual** matters most.

---

## 📦 What's inside

Everything is optional and off until you need it — but it's all there, waiting.

<table>
<tr><td valign="top" width="50%">

**🎨 Rendering & scene**
- Clean default scene, environment & camera controls
- Perspective / orthographic cameras, multiple cameras
- Multi-light lighting with screen-space shadows
- Scene theme presets (Blender / Studio / GameEngine)
- Fog, background & time-of-day
- Built-in profiler overlay (`P`)

**🧊 Geometry**
- Cube, Sphere, Cylinder, Capsule (tapered), Plane
- Arbitrary triangular meshes, lines, point clouds
- Particle systems (billboard clouds)
- Dynamic grid
- OBJ, Quake II MD2, AWP3D, JSON scene I/O

</td><td valign="top" width="50%">

**🛠️ Mesh toolkit**
- Extrude, rotate-around-axis, sweep, tube, loft, lines-to-mesh
- Merge, subdivide, mirror, extrude face
- Laplacian smooth, decimate
- Constructive Solid Geometry (union / difference / intersection)

**🕹️ Interaction & systems**
- Object picking, clickable planes, world↔screen projection
- Collision detection (AABB / sphere)
- Optional Bullet physics (joints, ragdoll, bouncing)
- NavMesh A* pathfinding with slope & step limits
- 3D spatial audio (miniaudio: WAV / MP3 / FLAC / OGG)
- BVH motion-capture playback
- Threaded clocks for animation & timed tasks
- HUD + full GUI (windows, buttons, sliders, edit boxes, progress bars)
- Extendable event controller chain

</td></tr>
</table>

---

## 🚀 Install

### Requirements

- Python **3.11+**
- A GPU with **OpenGL 3.3+**
- **LibSDL2** — `sudo apt install libsdl2-dev` (Debian/Ubuntu)
- **ImageMagick** — `sudo apt install imagemagick` (Debian/Ubuntu)

For other platforms, use your preferred package manager.

### Install with pip

```bash
pip install payton
```

If you hit permission errors, you're installing system-wide — use a virtualenv, or run `sudo pip3 install payton`.

### Upgrade

Payton is under active maintenance:

```bash
pip3 install payton --upgrade
```

### Optional: Bullet Physics

```bash
pip install pybullet
```

Installed in the same environment, Payton detects and activates it automatically.

### Optional: GTK3 instead of SDL2

Use Payton with native GTK3 widgets. Install the [Python GTK3 bindings](https://pygobject.readthedocs.io/en/latest/getting_started.html), then use the GTK integration.

![GTK3 integration](https://raw.githubusercontent.com/sinanislekdemir/payton/assets/assets/gtk3.png)

### Using Payton with Anaconda

Since `0.0.10`, Payton installs cleanly on Anaconda — just `pip install payton` from the Anaconda Prompt. It's then available in Spyder and JupyterLab locally.

![](https://islekdemir.com/payton/anaconda.png)

### AWP3D format & Blender exporter

AWP3D is a ZIP of one Wavefront OBJ per animation frame. The Blender add-on for exporting animated meshes lives in [`plugins/`](https://github.com/sinanislekdemir/payton/tree/master/plugins).

---

## 🎮 Controls

The default scene is immediately explorable — no setup required.

| Key / Action | Description |
| --- | --- |
| Mouse Wheel | Zoom in / out |
| Right Mouse Drag | Rotate scene |
| Middle Mouse Drag | Pan scene |
| Escape | Quit |
| C | Toggle camera mode (perspective / orthographic) |
| Space | Pause / resume scene clocks |
| G | Show / hide grid |
| W | Cycle display mode (solid / wireframe / points) |
| P | Cycle profiler (FPS → details → verbose → hide) |
| F2 / F3 | Previous / next camera |
| F12 | Screenshot (saves PNG in the current directory) |
| H | Open / close help window |

### Environment variables

- `SDL_WINDOW_WIDTH` — window width
- `SDL_WINDOW_HEIGHT` — window height
- `GL_MULTISAMPLEBUFFERS` — multisample buffer count for antialiasing (usually 1–2)
- `GL_MULTISAMPLESAMPLES` — multisample sample count for antialiasing (usually 1–16)

Set both `GL_MULTISAMPLEBUFFERS` **and** `GL_MULTISAMPLESAMPLES`, or graphics can look pixelated. There are no defaults because the ideal values vary by GPU.

---

## 🖼️ Showcase

A tiny sample of what falls out of this toolkit:

![Example](https://github.com/sinanislekdemir/payton/blob/assets/assets/02.jpg?raw=true)
![Example](https://github.com/sinanislekdemir/payton/blob/assets/assets/04.jpg?raw=true)
![Example](https://github.com/sinanislekdemir/payton/blob/assets/assets/05.jpg?raw=true)
![Example](https://github.com/sinanislekdemir/payton/blob/assets/assets/11.jpg?raw=true)
![AWP3D](https://github.com/sinanislekdemir/payton/blob/assets/assets/awp3d.jpg?raw=true)
![Bullet physics](https://github.com/sinanislekdemir/payton/blob/assets/assets/bullet.jpg?raw=true)
![Time of day](https://github.com/sinanislekdemir/payton/blob/assets/assets/day.jpg?raw=true)
![Engrave / heightmap](https://github.com/sinanislekdemir/payton/blob/assets/assets/engrave.jpg?raw=true)
![Explosion](https://github.com/sinanislekdemir/payton/blob/assets/assets/explosion.jpg?raw=true)
![GUI](https://github.com/sinanislekdemir/payton/blob/assets/assets/gui.jpg?raw=true)
![Quake II](https://github.com/sinanislekdemir/payton/blob/assets/assets/quake.jpg?raw=true)
![Ripple](https://github.com/sinanislekdemir/payton/blob/assets/assets/ripple.jpg?raw=true)
![Spotlight](https://github.com/sinanislekdemir/payton/blob/assets/assets/spot.jpg?raw=true)

**Watch it in motion:**

[![Payton showcase](https://islekdemir.com/payton/youtube.png)](https://www.youtube.com/watch?v=bKQ9G1J5JYM)

[![Payton screencast](http://i3.ytimg.com/vi/3ATRVLNuCew/maxresdefault.jpg)](https://www.youtube.com/watch?v=3ATRVLNuCew)

[![Bullet physics demo](https://www.islekdemir.com/snapshot.jpg)](https://www.youtube.com/watch?v=Zt2vnUMLYVs)

*Tested on Windows 10 (Paperspace) — works as expected.*

---

## 📖 Examples

I don't read long descriptive documentation unless I have to. I like things simple and self-explanatory. So instead of writing walls of docs, I write **simple, runnable examples** for every feature — ready to tweak, break, and learn from.

Grab them from the [examples folder](https://github.com/sinanislekdemir/payton/tree/master/examples), or clone the whole repository [as a zip](https://github.com/sinanislekdemir/payton/archive/master.zip):

```bash
git clone https://github.com/sinanislekdemir/payton.git
cd payton && pip install -e .
python examples/basics/01_scene.py
```

---

## 🗂️ Example Index

### Basics
* [Scene — your first window](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/01_scene.py)
* Objects
  * [Adding a cube](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/02_cube.py)
  * [Adding multiple cubes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/03_cubes.py)
  * [Parent–child relations](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/05_children.py)
  * [Cylinder](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/18_cylinder.py)
  * [Capsule](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/39_capsule.py)
  * [Loading complex triangular objects (monkey)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/06_monkey.py)
  * [Complex meshes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/09_mesh.py)
  * [Point cloud](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/11_point_cloud.py)
  * [Particle system](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/11_particle_system.py)
  * [Plane object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/13_plane.py)
  * [Line object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/17_line.py)
  * [Better lines](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/33_better_lines.py)
  * [Mesh plane](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/32_mesh_plane.py)
  * [Quake 2 objects](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/26_quake2.py)
  * [Ragdoll](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/28_ragdoll.py)
* [Clocks (timed animation)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/04_clock.py)
* [Object picking with the mouse](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/07_picking.py)
* [Loading textures](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/08_texture.py)
* [Vertex colors](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/10_vertex_colors.py)
* Collision detection
  * [Simple](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/12_collision.py)
  * [Detailed](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/12_1_collision_detailed.py)
* Physics engine
  * [Bullet hello world](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/01_bullet_hello.py)
  * [Point-to-point joint](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/02_joint_p2p.py)
  * [Bullet cubes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/37_bullet_cubes.py)
  * [Bouncing ball](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/38_bouncingball.py)
* [GTK3 + Payton integration](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/03_gtk.py)
* [Rotating objects](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/14_rotate.py)
* [Graphical User Interface (GUI)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/15_gui.py)
* [Custom keyboard shortcuts](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/16_keyboard.py)
* [Multiple cameras](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/19_multiple_cameras.py)
* [Changing the background](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/20_background.py)
* [Click plane (cursor in world coordinates)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/21_click_plane.py)
* [Object motion history](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/22_go_back.py)
* [Motion capture (BVH)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/23_motion.py)
* [BVH viewer](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/bvh_viewer.py)
* [Object-oriented approach](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/24_object_oriented.py)
* [Materials](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/25_materials.py)
* [Export / import a scene to JSON](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/27_json.py)
* [Changing time of day](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/29_day.py)
* [Near and far planes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/30_near_far_plane.py)
* [Spotlight](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/31_spotlight.py)
* [AWP3D](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/34_awp3d.py)
* [AWP3D animation ranges](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/35_awp3d_range.py)
* [World-to-screen projection](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/36_world_to_screen.py)
* [Fog effect](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/41_fog.py)
* [Minecraft-like voxel scene](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/40_minecraft_like.py)
* [Multi-shadow](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/42_multi_shadow.py)
* [NavMesh pathfinding](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/43_navmesh.py)
* [NavMesh maze](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/44_navmesh_maze.py)
* [3D spatial audio](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/45_3d_audio.py)

### Mesh tools
* [Extrude line](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/01_extrude_line.py)
* [Rotate line](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/02_rotate_line.py)
* [Lines to mesh](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/03_lines_to_mesh.py)
* [Merge mesh](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/04_merge_mesh.py)
* [Subdivide mesh](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/05_subdivide.py)
* [Sweep](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/06_sweep.py)
* [Loft](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/07_loft.py)
* [Tube](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/08_tube.py)
* [Mirror](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/09_mirror.py)
* [Extrude face](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/10_extrude_face.py)
* [Laplacian smooth](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/11_laplacian_smooth.py)
* [Decimate](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/12_decimate.py)
* [CSG operations](https://github.com/sinanislekdemir/payton/blob/master/examples/tools/13_csg.py)

### Mid-level
* [Popping balloons (game)](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/balloon.py)
* [Build a mesh from a heightmap](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/engrave.py)
* [A more complex Quake scene](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/quake2.py)
* [Ripple (mesh grid)](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/ripple.py)
* [RPG-like controls](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/rpg.py)
* [Custom shader](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/shader.py)

### High-level
* Multiplayer
  * [Server backend](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/multiplayer/server.py)
  * [3D block-building client](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/multiplayer/client3D.py)
* [Heightmap terrain](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/heightmap/heightmap.py)
* [Cyberarena](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/cyberarena/main.py)
* [3D object designer tool](https://github.com/sinanislekdemir/payton/blob/master/examples/designer/main.py)

---

## 🎭 Motion capture data

Payton reads **BVH (Biovision Hierarchy)** files. For extensive datasets, the [Bandai Namco Research Motion Dataset](https://github.com/BandaiNamcoResearchInc/Bandai-Namco-Research-Motiondataset) offers thousands of BVH files — the bundled example files come from there.

---

## 🧪 Troubleshooting

On older systems or machines without proper GPU drivers, try running under MESA. It works fine with some performance cost — usually unnoticeable for simple scenes:

```bash
MESA_GL_VERSION_OVERRIDE=3.3 python <path-to-your-payton-code>
```

---

## 🤝 Contributing

Contributions are welcome. A few house rules keep the codebase consistent:

* Use type hints throughout the main library; examples are exempt.
* Keep example code plain and simple.
* Every new feature needs sensible defaults and an example.
* Run `make check` before pushing.
* `isort .` is encouraged but not mandatory.
* Some methods are intentionally longer and more complex — to reduce code jumps / stack switches and run faster.

---

## 🧠 Some free thoughts and decisions

I chose `List[float]` for vectors because:

* I needed something **mutable**. Otherwise the number of memory copies and swaps would be excessive — so `Tuple` and `NamedTuple` were out.
* `dataclass` adds overhead when converting to C-type floats and arrays in memory.

To gain performance, the core library accepts the (small) risk of non-strict vector lengths. It's a deliberate trade: **speed over ceremony**.
