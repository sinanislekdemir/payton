# Payton 3D SDK

Payton is a general-purpose 3D Software Development Kit. Simply, a 3D Programming Playground!

* Payton is a playground. Kickstart any idea fast and easy, grow it.
* Create tools for the next step. Create map editors, small animations, small
  algorithms or artificial intelligence for your game. Whenever you need to try a new idea, don't bother to create a new application with all the details. Payton comes with all the necessary defaults and that is what makes it unique. Almost everything has a pre-set.
* Game engines and other libraries are way too complex and it takes a long time
  to start the initial playground.
* Payton never intends to take place as a game engine or a full-featured 3D
  environment. There is already plenty of stuff for that purpose.
* Tools programming is easy.
* Easy to visualize what you want to achieve or do what you want to do.
* You can move forward from Payton to any other place if you like.


We draw 2D graphs and charts in reports and we generally understand much more
easily when we visualize the data. But in some cases, visualizing exceeds 2
dimensions. We require to have third and even fourth dimensions. (And on top of
those, the definition of the fourth dimension as time can get foggy in terms of
relativity.)

Payton gives you the ability to extend your graphics into 4 dimensions. It is not
software but a software development toolkit/library built with Python.
This will give users the ability to read real-time data from sensors, cameras or
any other data sources in realtime and visualize them in real-time. The data source
can be a thermometer, a random number generator, a toy car connected to a speed
sensor, a map, a vehicle port or anything that generates time-based 3D data.
Furthermore, it can be a time-based formula. As this can get too complex,
software with that complexity will probably be too hard to use and understand
where Payton is designed to be as simple as it can be. So easy to program that
a newbie can kick-start it just by following the tutorials.

## Contents of this Document:

- [Payton 3D SDK](#payton-3d-sdk)
  - [Contents of this Document:](#contents-of-this-document)
  - [Features:](#features)
  - [Install](#install)
    - [Requirements:](#requirements)
    - [Install using Pip:](#install-using-pip)
    - [Upgrade to the latest version:](#upgrade-to-the-latest-version)
  - [Getting Started](#getting-started)
    - [Your first code](#your-first-code)
  - [Controls](#controls)
  - [Examples](#examples)
  - [Example Index](#example-index)
  - [Screenshots and Videos](#screenshots-and-videos)


## Features:

* 3D Math Library
* Various base geometries:
  * Cube
  * Cylinder
  * Triangular Mesh
  * Plane
  * Lines
  * Particle System
  * Sphere
  * Dynamic Grid
* Clean default scene.
* Pre-defined keyboard-mouse and camera controls
* Pre-defined environment
* Clock system for parallel tasks and time based operations
* Simple collision detection
* Extendable controllers
* Pre-defined lighting with shadows.
* Material support
* Clickable objects and virtual planes
* Shader support
* Basic GUI Support
  * Window
  * Panel
  * Button
  * EditBox (Multi-line support)
* 3D File formats:
  * Wavefront OBJ
  * Quake 2 MD2 with Animations
* Mesh Generation Tools
  * Extrude Line in 3D
  * Rotate Line around an axis in 3D
  * Fill between lines
* Mesh modifiers:
  * Merge Mesh
  * Subdivide Mesh
* Extensive examples for every feature


## Install

### Requirements:

- LibSDL2 `sudo apt install libsdl2-dev` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- imagemagick `sudo apt install imagemagick` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- Python 3.7+
- A Graphics card that supports OpenGL 3.3+

### Install using Pip:

From a bash terminal:
```bash
pip3 install payton
```

This should install all dependencies. If you get any permission errors, you are probably installing the library to system-global so missing some permissions. If you do not want to use pipenv or virtualenv, then you might want to run above command as `sudo pip3 install payton`

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
| F2 | Previous Observer |
| F3 | Next Observer |
| H | Open / Close Help Window |

## Examples

Personally, I don’t really read the long descriptive documentation unless necessary. I like things simple and self-explaining. Therefore, instead of writing long documentations, I write simple examples to use each feature of Payton without digging much into the internals.

Examples can be downloaded from [Payton Github Page](https://github.com/sinanislekdemir/payton/tree/master/examples).

You can either download the whole repository [as a zip file](https://github.com/sinanislekdemir/payton/archive/master.zip) or you can just `git clone` it.

*Tested in Windows 10 Paperspace, seems to be working as expected*
![https://user-images.githubusercontent.com/1842484/84317888-38767780-ab76-11ea-8337-a102d7c59275.png](https://user-images.githubusercontent.com/1842484/84317888-38767780-ab76-11ea-8337-a102d7c59275.png)

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


## Screenshots and Videos

[![](https://27x2.com/youtube.png)](https://www.youtube.com/watch?v=bKQ9G1J5JYM)

|  |  |  |
| --- | --- | --- |
| [![](https://27x2.com/thumb-cube.png)](https://27x2.com/cube.png) | [![](https://27x2.com/thumb-clock.png)](https://27x2.com/clock.png) | [![](https://27x2.com/thumb-monkey.png)](https://27x2.com/monkey.png) |
| [![](https://27x2.com/thumb-particle.png)](https://27x2.com/particle.png) | [![](https://27x2.com/thumb-mesh_plane.png)](https://27x2.com/mesh_plane.png) | [![](https://27x2.com/thumb-ripple.png)](https://27x2.com/ripple.png) |
| [![](https://27x2.com/thumb-engrave.png)](https://27x2.com/engrave.png) | [![](https://27x2.com/thumb-spotlight.png)](https://27x2.com/spotlight.png) | [![](https://27x2.com/thumb-day.png)](https://27x2.com/day.png) |
| [![](https://27x2.com/thumb-ragdoll.png)](https://27x2.com/ragdoll.png) | [![](https://27x2.com/thumb-quake.png)](https://27x2.com/quake.png) | [![](https://27x2.com/thumb-motion.png)](https://27x2.com/motion.png) |
| [![](https://27x2.com/thumb-gui.png)](https://27x2.com/gui.png) | [![](https://27x2.com/thumb-collision.png)](https://27x2.com/collision.png) | |
