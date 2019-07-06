# Payton

## What is Payton?

Payton is a general purpose 3D programming toolkit. It is designed with the
theory that us, humans, we understand better by seeing things.


* Payton is a prototyping tool. Kickstart any idea fast and easy, grow it.
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

More information can be found in documents.

## Install and kick-start

    $ pip install payton

Then go ahead and create a `test.py`

    from payton.scene import Scene
    
    a = Scene()
    a.run()
    
And thats all!

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

You can generate the API documentation using:

    $ make docs

It should generate the documentations and open it automatically in browser.
If it fails to open up the browser, you'll find the docs in `docs/payton`
directory

I encourage you to create a virtualenv for Payton (with Python 3.5+)

`virtualenv -p <path-to-python3> payton` should do the trick. If you don't have
`virtualenv` in your path, google it!

To start fiddling with it, `python setup.py develop` will install all
requirements and will add `payton` to site-packages. But changes to code will
immediately take effect, as opposed to `install` command.

To kick start, after `python setup.py develop` run `python examples/04_clock.py`

And once you see the white ball on the screen, hit `SPACE` from keyboard to unpause
the animation mode and see the projectile motion demo.
