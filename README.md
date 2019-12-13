# Payton

## What is Payton?

[![Python DEMO](https://www.islekdemir.com/NSL7nCZrnF4.png)](https://youtu.be/NSL7nCZrnF4)

Payton is a general purpose 3D programming toolkit. Simply, a 3D Programming Playground!

[![RippleVideo](https://27x2.com/screenshot2.png)](https://www.youtube.com/watch?v=tMK2bS_He2c)

* Payton is a playground. Kickstart any idea fast and easy, grow it.
* Create tools for the next step. Create map editors, small animations, small
  algorithms or artificial intelligence for your game. Whenever you need to
  try a new idea, don't bother to create a new application with all the
  details. Payton comes with all the necessary defaults and that is what makes
  it unique. Almost everything has a pre-set.
* Game engines and other libraries are way too complex and it takes a long time
  to start the initial playground.
* Payton never intends to take place as a game engine or a full-featured 3D
  environment. There are already plenty of stuff for that purpose.
* Tools programming is easy.
* Easy to visualise what you want to achive or do what you want to do.
* You can move forward from Payton to any other place if you like.


We draw 2D graphs and charts in reports and we generally understand much more
easily when we visualise the data. But in some cases, visualising exceeds 2
dimensions. We require to have third and even forth dimensions. (And on top of
those, definition of forth dimension as time can get foggy in terms of
relativity.)

Payton gives you the ability to extend your graphics into 4 dimensions. It is not
software but a software development toolkit/library built with Python.
This will give users ability to read real time data from sensors, cameras or
any other data sources in realtime and visualise them in real time. Data source
can be a thermometer, a random number generator, a toy car connected to a speed
sensor, a map, a vehicle port or anything that generates time based 3D data.
Furthermore, it can be a time based formula. As this can get too complex,
software with that complexity will probably be too hard to use and understand
where Payton is designed to be as simple as it can be. So easy to program that
a newbie can kick-start it just by following the tutorials.

## Features:

* 3D Math Library
* Various base geometries:
  * Cube
  * Cylinder
  * Triangular Mesh
  * Plane
  * Lines
  * Point Cloud
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
* Extensive examples for every feature and more.


## Examples:

Examples can be found at [https://github.com/sinanislekdemir/payton/tree/master/examples](https://github.com/sinanislekdemir/payton/tree/master/examples)

More information can be found in documents.

## Some Limitations:

- Currently, only the initial light source (`scene.lights[0]`) cast shadows. This is primarily for performance reasons and hardcoded.
- There can be upto 100 lights in the scene.
- Even-though there is not restriction on number of objects in the scene, it can effect the initial load time. Once it gets loaded, it should work fine as graphics card and shader program does the heavy-lifting.
- There are only two collision detection algorithms. Axis Aligned Bounding Box (AABB) is the default algorithm. Also you can reduce it to Spherical collision detection as well, which is simpler and works faster but it just checks for the bounding spheres of objects thus makes a pretty rough assumption.

## Install and kick-start

### Requirements:

- LibSDL2 `sudo apt install libsdl2-dev` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- imagemagick `sudo apt install imagemagick` for debian/ubuntu based linux distros. For other platforms, you can see your favourite package manager.
- Python 3.7+
- A Graphics card that supports OpenGL 3.3+

### Install:

    $ pip3 install payton

Then go ahead and create a `test.py`

    from payton.scene import Scene
    
    a = Scene()
    a.run()

This will bring up a default scene. You can press **h** from keyboard to show help window.  
For a more complex example you can try this:

    import math

    from payton.scene import Scene
    from payton.scene.geometry import Cube, Plane

    scene = Scene()


    def move(period, total):
        """Move function gets called by clock tick and handles our light movements"""
        angle = (total * 60) % 360
        px = math.cos(math.radians(angle)) * 8
        py = math.sin(math.radians(angle)) * 8
        scene.lights[0].position = [px, py, 4.0]


    # Create a cube for the scene
    cube = Cube()
    # Set it to a position
    cube.position = [2, 1, 0.5]

    # Create the ground plane 30x30 in size
    ground = Plane(width=30, height=30)
    # We need a wall to project our shadows
    wall1 = Plane(width=30, height=10)
    # Wall is on the ground, so lets rotate it around X axis to stand perpendicular
    wall1.rotate_around_x(math.radians(90))
    # And lets give it a position.
    wall1.position = [0, -10, 5]

    # There is already a pre-defined light in the scene, so we are modifying its location.
    scene.lights[0].position = [5.0, 5.0, 6.0]
    # Create another Cube, but this time instead of giving it size, we are using two points
    # in space and construct a cube by joining two points.
    cube_by_corners = Cube(from_corner=[-3, -3, 1], to_corner=[-1, -1, 3])
    # Create a clock in the scene to move the light
    scene.create_clock('mm', 0.001, move)

    # Now add all objects to scene
    scene.add_object("wall", wall1)
    scene.add_object("cube", cube)
    scene.add_object("cube_by_corners", cube_by_corners)
    scene.add_object("ground", ground)

    # Scene comes with a default Grid, so lets hide it for this example.
    scene.grid.visible = False

    # It's alive!
    scene.run()


### Default key mapping:

- **Zoom In-Out**: Left Ctrl + Mouse Drag (up and down)
- **Rotate**: Left Shift + Mouse Drag (left and right)
- **ESC**: Quit Simulation
- **C**: Change camera mode (Perspective / Orthographic)
- **Space**: Pause scene (stop all Clocks)
- **G**: Show/Hide Grid.
- **W**: Display mode: Wireframe / Solid
- **F2**: Previous observer
- **F3**: Next observer

### Mouse controls
- L_CTRL + Mouse(Left) Drag: Zoom In-Out
- L_SHIFT + Mouse(Left) Drag: Rotate around target
- L_SHIFT + L_CTRL + Mouse(Left) Drag: Panning

## Development

Some notes on Python3:

Currently Payton is in pre-alpha, or it is just some dust cloud in the space.
If you want to contribue, here is what you can do:

I encourage you to create a virtualenv for Payton (with Python 3.7+)

`virtualenv -p <path-to-python3> payton` should do the trick. If you don't have
`virtualenv` in your path, google it!

To start fiddling with it, `python setup.py develop` will install all
requirements and will add `payton` to site-packages. But changes to code will
immediately take effect, as opposed to `install` command.

Be sure to check your code with `flake8` + `flake8-isort` + `mypy` before sending.  
To format your code, you can use `black`.

To kick start, after `python setup.py develop` run `python examples/04_clock.py`

And once you see the white ball on the screen, hit `SPACE` from keyboard to unpause
the animation mode and see the projectile motion demo.

Also, `payton` library sources uses typehints, please keep using them but `examples` are free from all kind of best practices. Keep in mind that, the aim of this library is to let inexperienced people to do things. Therefore, examples are created as simple as possible. You don't have to use any lambda functions or generators or so forth there. Readability matters most at examples. _But **performance** is what we need in the core payton source!_  
