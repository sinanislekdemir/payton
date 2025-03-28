# Payton 3D SDK
[Payton 3D SDK Screencast](https://github.com/sinanislekdemir/payton/assets/1842484/a18c8e43-61e9-48c3-a92a-4988716a1919)

![example workflow](https://github.com/sinanislekdemir/payton/actions/workflows/pythonpackage.yml/badge.svg)
[![Downloads](https://pepy.tech/badge/payton/month)](https://pepy.tech/project/payton)
[![Downloads](https://pepy.tech/badge/payton/week)](https://pepy.tech/project/payton)

Payton is a 3D Software Development Kit that serves as a general-purpose playground for programming. With Payton, users can quickly and easily kickstart their ideas and create tools for the next step, such as map editors, small animations, small algorithms, or artificial intelligence for games. Unlike game engines and other libraries that can be too complex and time-consuming to start, Payton comes with pre-set defaults, making it unique and simple to use.

Payton is not intended to be a game engine or a full-featured 3D environment as there are already plenty of tools available for those purposes. However, it excels at tools programming and allows users to easily visualize their goals and achieve them. Users can also move forward from Payton to any other place if they desire.

While 2D graphs and charts are useful in reports, some cases require visualizing data in 3 or 4 dimensions. Payton enables users to extend their graphics into 4 dimensions and read real-time data from sensors, cameras, or any other data source. These sources can be anything that generates time-based 3D data, including thermometers, random number generators, toy cars connected to speed sensors, maps, or vehicle ports. Payton can even handle time-based formulas. Although it can get complex, Payton is designed to be easy to use, even for beginners who can follow the tutorials to kick-start their programming skills.

## Contents of this Document:

- [Payton 3D SDK](#payton-3d-sdk)
  - [Contents of this Document:](#contents-of-this-document)
  - [Features:](#features)
  - [Install](#install)
    - [Requirements:](#requirements)
    - [Install using Pip:](#install-using-pip)
      - [AWP3D Format and Exporter](#awp3d-format-and-exporter)
      - [Using Payton with Anaconda](#using-payton-with-anaconda)
    - [Upgrade to the latest version:](#upgrade-to-the-latest-version)
  - [Getting Started](#getting-started)
    - [Your first code](#your-first-code)
  - [Controls](#controls)
  - [Examples](#examples)
  - [Example Index](#example-index)
  - [Contribution](#contribution)
  - [Screenshots and Videos](#screenshots-and-videos)
  - [Troubleshooting](#troubleshooting)
  - [Some free-thoughts and decisions:](#some-free-thoughts-and-decisions)


## Features:

-   3D Math Library
-   Various base geometries:
    -   Cube
    -   Cylinder
    -   Triangular Mesh
    -   Plane
    -   Lines
    -   Particle System
    -   Sphere
    -   Dynamic Grid
-   Clean default scene
-   Pre-defined keyboard-mouse and camera controls
-   Pre-defined environment
-   Clock system for parallel tasks and time-based operations
-   Simple collision detection
-   Optional Physics Engine
    -   Basic support for Bullet Physics (to be extended)
-   Extendable controllers
-   Pre-defined lighting with shadows
-   Material support
-   Clickable objects and virtual planes
-   Shader support
-   Basic GUI Support
    -   Window
    -   Panel
    -   Button
    -   EditBox (with multi-line support)
-   3D File formats:
    -   AWP3D Animated High-Poly 3D Object
    -   Wavefront OBJ
    -   Quake 2 MD2 with Animations
-   Mesh Generation Tools
    -   Extrude Line in 3D
    -   Rotate Line around an axis in 3D
    -   Fill between lines
-   Mesh modifiers:
    -   Merge Mesh
    -   Subdivide Mesh
-   Extensive examples for every feature

## Install

### Requirements:

- LibSDL2 `sudo apt install libsdl2-dev` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- imagemagick `sudo apt install imagemagick` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- Python 3.10+
- A Graphics card that supports OpenGL 3.3+

### Install using Pip:

From a bash terminal:
```bash
pip install payton
```

This should install all dependencies. If you get any permission errors, you are probably installing the library to system-global so missing some permissions. If you do not want to use pipenv or virtualenv, then you might want to run above command as `sudo pip3 install payton`

### Optional Bullet Physics Integration

Payton supports Bullet Physics at a basic level for solid geometries.

```bash
pip install pybullet
```

Once Bullet Physics is successfully installed in the same environment as Payton, it will be automatically activated and you will be able to use its basic properties.
Check out the relevant examples.

### Optional GTK3 Integration

Instead of SDL2, you can use GTK3 (along with all nice GTK3 widgets) with Payton.
![https://raw.githubusercontent.com/sinanislekdemir/payton/assets/assets/gtk3.jpg](https://raw.githubusercontent.com/sinanislekdemir/payton/assets/assets/gtk3.png)

You need to [install Python GTK3 Bindings](https://pygobject.readthedocs.io/en/latest/getting_started.html).

#### AWP3D Format and Exporter

AWP3D is simply a ZIP file containing every frame as a Wavefront object. To export your animated Blender objects as AWP3D files, you can use the exporter add-on available here:

[https://github.com/sinanislekdemir/payton/tree/master/plugins](https://github.com/sinanislekdemir/payton/tree/master/plugins)

#### Using Payton with Anaconda

As of version `0.0.10`, Payton is installable on Anaconda. From Anaconda Prompt:

```bash
pip install payton
```

Is sufficient to install Payton and it's dependencies on Anaconda.

Payton will be available to use with Spyder or JupyterLab on local.

![](https://islekdemir.com/payton/anaconda.png)

### Upgrade to the latest version:

Payton is under active maintenance. This means I am spending some time to fix the bugs or make it better. So you might want to upgrade it occasionally.

    pip3 install payton --upgrade
    
should do the trick!

## Getting Started

### Your first code

```python
from payton.scene import Scene

scene = Scene()
scene.run()
```

This will create your first empty scene and show it inside an SDL window.

## Controls

| Key / Action | Description |
| --- | --- |
| Mouse Wheel | Zoom In - Zoom Out|
| Right Mouse Button Drag | Rotate Scene |
| Middle Mouse Button Drag | Pan Scene |
| Escape | Quit Simulation |
| C | Change Camera Mode (Perspective / Orthographic) |
| Space | UnPause / Pause Scene Clocks |
| G | Show / Hide Grid |
| W | Change Display Mode (Solid / Wireframe / Points) |
| F2 | Previous Camera |
| F3 | Next Camera |
| H | Open / Close Help Window |

## Environment variables

Some options can be configured from the environment variables.

- `SDL_WINDOW_WIDTH`: Set window width.
- `SDL_WINDOW_HEIGHT`: Set window height.
- `GL_MULTISAMPLEBUFFERES`: Set OpenGL multisample buffer count for antialiasing. (usually 1 or 2)
- `GL_MULTISAMPLESAMPLES`: Set OpenGL multisample sampling count for antialiasing. (usually 1-16)

Without `GL_MULTISAMPLEBUFFERS` AND `GL_MULTISAMPLESAMPLES`, you may notice pixelated graphics. There is no default set for those values because they can vary between graphic cards.

## Troubleshooting

In some older systems or where a decent graphics driver is not installed, you can try running Payton code with MESA. It runs fine. There will be some performance decrease. But it will not be noticeable for basic applications.

To enforce MESA 3.3, you can run payton with:

```
MESA_GL_VERSION_OVERRIDE=3.3 python <path-to-your-payton-code>
```

## Examples

Personally, I don’t really read the long descriptive documentation unless necessary. I like things simple and self-explaining. Therefore, instead of writing long documentations, I write simple examples to use each feature of Payton without digging much into the internals.

Examples can be downloaded from [Payton Github Page](https://github.com/sinanislekdemir/payton/tree/master/examples).

You can either download the whole repository [as a zip file](https://github.com/sinanislekdemir/payton/archive/master.zip) or you can just `git clone` it.

*Tested in Windows 10 Paperspace, seems to be working as expected*
![https://user-images.githubusercontent.com/1842484/84317888-38767780-ab76-11ea-8337-a102d7c59275.png](https://user-images.githubusercontent.com/1842484/84317888-38767780-ab76-11ea-8337-a102d7c59275.png)

*Supports pyBullet solid geometry physics*
[![https://www.youtube.com/watch?v=Zt2vnUMLYVs](https://www.islekdemir.com/snapshot.jpg)](https://www.youtube.com/watch?v=Zt2vnUMLYVs)

## Example Index

* Basic Examples
  * [Scene](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/01_scene.py)
  * Objects
    * [Adding a cube](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/02_cube.py)
    * [Adding multiple cubes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/03_cubes.py)
    * [Object parent-child relations](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/05_children.py)
    * [Cylinder object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/18_cylinder.py)
    * [How to load complex triangular objects](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/06_monkey.py)
    * [Complex meshes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/09_mesh.py)
    * [Point cloud](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/11_point_cloud.py)
    * [Particle System](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/11_particle_system.py)
    * [Plane Object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/13_plane.py)
    * [Line object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/17_line.py)
    * [Better Lines Example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/33_better_lines.py)
    * [Mesh plane](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/32_mesh_plane.py)
    * [Quake 2 Objects](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/26_quake2.py)
    * [Ragdoll Object](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/28_ragdoll.py)
  * [How to use "clock"](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/04_clock.py)
  * [Object picking using mouse](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/07_picking.py)
  * [Load textures](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/08_texture.py)
  * [Vertex colors](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/10_vertex_colors.py)
  * Collision Detection
    * [Simple example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/12_collision.py)
    * [Complex example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/12_1_collision_detailed.py)
  * Physics Engine
    * [Cubes example](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/01_bullet_hello.py)
    * [Joint example](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/02_joint_p2p.py)
  * [GTK3 Python OpenGL Payton Integration](https://github.com/sinanislekdemir/payton/blob/master/examples/additional/03_gtk.py)
  * [Rotating Objects](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/14_rotate.py)
  * [Graphical User Interface (GUI)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/15_gui.py)
  * [Custom keyboard shortcuts](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/16_keyboard.py)
  * [Using multiple cameras](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/19_multiple_cameras.py)
  * [Changing background](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/20_background.py)
  * [Click plane (get cursor location in world coordinates)](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/21_click_plane.py)
  * [Using object motion history](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/22_go_back.py)
  * [Object Oriented Approach](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/24_object_oriented.py)
  * [Materials](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/25_materials.py)
  * [Exporting and importing your scene to json](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/27_json.py)
  * [Changing time of day](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/29_day.py)
  * [Near and Far Planes](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/30_near_far_plane.py)
  * [Spotlight Example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/31_spotlight.py)
  * [Mesh Plane Example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/32_mesh_playne.py)
  * [AWP3D Example](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/34_awp3d.py)
  * [AWP3D Example Ranges](https://github.com/sinanislekdemir/payton/blob/master/examples/basics/35_awp3d_range.py)
* Mid Level
  * [Popping the baloons game](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/balloon.py)
  * [Build mesh using heightmap](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/engrave.py)
  * [Fetch Instagram Images and build 3D Wall](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/instagram.py)
  * [A bit more complex Quake Example](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/quake2.py)
  * [Ripple Example (Mesh Grid)](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/ripple.py)
  * [RPG-like Controls](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/rpg.py)
  * [Custom Shader](https://github.com/sinanislekdemir/payton/blob/master/examples/mid-level/shader.py)
* High Level
  * Multiplayer
    * [Server backend for a multiplayer game](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/multiplayer/server.py)
    * [3D Blocks building game multiplayer](https://github.com/sinanislekdemir/payton/blob/master/examples/high-level/multiplayer/client3D.py)

## Contribution

* Please keep using type hints in the main library.
* Type hinting can be ignored for examples.
* Example codes should be plain and simple.
* Every new features should include sensible defaults.
  * Nothing should be too verbose to use.
* Make sure that `make check` is clear before pushing your code.
* Running `isort .` is not mandatory but highly encouraged.
* Ever new feature should have an example code.
* There is a reason why some methods are longer than they should and complex.
  * To reduce code jumps and stack switches.
  * To run faster.

## Screenshots and Videos

[![](https://islekdemir.com/payton/youtube.png)](https://www.youtube.com/watch?v=bKQ9G1J5JYM)

[![](http://i3.ytimg.com/vi/3ATRVLNuCew/maxresdefault.jpg)](https://www.youtube.com/watch?v=3ATRVLNuCew)

## Examples Showcase:

![https://github.com/sinanislekdemir/payton/blob/assets/assets/02.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/02.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/04.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/04.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/05.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/05.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/11.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/11.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/awp3d.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/awp3d.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/bullet.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/bullet.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/day.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/day.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/engrave.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/engrave.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/explosion.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/explosion.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/gui.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/gui.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/quake.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/quake.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/ripple.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/ripple.jpg?raw=true)
![https://github.com/sinanislekdemir/payton/blob/assets/assets/spot.jpg?raw=true](https://github.com/sinanislekdemir/payton/blob/assets/assets/spot.jpg?raw=true)

## Some free-thoughts and decisions:

I've choosen to use `List[float]` type for Vectors because:

* I needed something mutable. Otherwise, the number of memory copy and swaps would be too much. So, I've crossed out `Tuple` and `NamedTuple`.
* `dataclass` has an overhead to convert to C-type floats and arrays in memory.

So, to gain some performance, I have created the main library with a risk of non-strict vector lengths.
