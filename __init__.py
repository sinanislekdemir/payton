"""
# Payton Library

## What is Payton?

Payton is a Python 3.5 (and hopefully Python 2.7 compatible) library to make
computational science (such as physics) easy to visualize.

Many things can be created with ease using Payton. Such as, simulating Newtonian
Physics or particles, fluids, astronomical objects, chemical compounds, or
mathematical graphical formulas.

### Why Python?

Python is the current rising star in computational science. Many researchers
and scientists, as well as students, use Python for analytics or statistics,
or for data science and such. With the power of the Numpy library, a
researchers' life becomes so easy. So I wanted to give people a tool within
their comfort zones.

I am well aware that Python is not the best programming language to do 3D.
But one should also keep in mind that, this library is not designed for
creating games and such. It is created to be simple and easy to use. To give
developers a rapid kick-start. Developers do not need to know anything
related to OpenGL or SDL2, or even Threading!
All they need to know is the basic structure of this library and have a
look at the sample codes defined. That's all.

### Why a relatively old OpenGL version?

Current version uses OpenGL 3.3 (GLSL 330) for rendering. Any later versions
might not be supported by relatively old computers. On the other hand, it was
hard to keep two different versions maintained. (4.5 and 3.3).

I believe that, within the OpenGL versions, there was a huge difference
between 3.3 and before. So, ground breaking changes came with the version 3.3.

### Unit Tests

Unit tests for the render pipeline are missing. It is hard to unit test that
part of the system and I was lazy. Mathematical functions and objects are
covered with Python's internal `unittest` framework.

### Primitives

Many 3D Libraries come with their own objects for almost everything. There are
things like `Vector3D`, `Matrix`, `Face3D` etc. Having that kind of mathematical
objects are often handy in terms of Object Oriented Design. On the other hand,
those makes developers tightly couple their applications to your library. Also,
cross-library references require type-object conversations, which are costly.
On top of that, more objects mean more complexity.

When a point in space can be described as a list or tuple with 3 elements
as `[x, y, z]`, why bother with `Vector(x, y, z)` and create a redundant object?
Also, memory operations with `tuple` or `list` are faster than any memory heavy
object operations.

## Getting started

Everything within Payton comes with pre-set definitions so it is quite easy
to start creating your "scenes".

A very basic Scene can be generated like:

    from payton.scene import Scene

    my_scene = Scene()
    my_scene.run()

This will bring up a simple SDL2 Window and an empty 3D Scene!

## Keyboard short-cuts

Moving inside the scene is quite easy. There are pre-defined keyboard controls
to make your life easier.

- **Zoom In-Out**: Left Ctrl + Mouse Drag (up and down)
- **Rotate**: Left Shift + Mouse Drag (left and right)
- **C** *(Press c from keyboard)*: Change camera mode (Perspective / Orthographic)
- **P** *(Press p from keyboard)*: Pause scene (stop all Clocks)
- **G** *(Press g from keyboard)*: Show/Hide Grid.

"""
