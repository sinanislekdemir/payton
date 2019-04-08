# Payton

## What is Payton?

Payton is a generic 3D library on top of SDL and OpenGL. It is designed to
help Computational Science experts to simulate their tests and findings.

More information can be found in documents.

## Development

Currently Payton is in pre-alpha, or it is just some dust cloud in the space.
If you want to contribue, here is what you can do:

`pdoc --html payton` will create API docs for you. (Api docs are generated
from comments and code itself, so keep comments in the code as descriptive
as possible.)

I encourage you to create a virtualenv for Payton (with Python 3.5+)

`virtualenv -p <path-to-python3> payton` should do the trick. If you don't have
`virtualenv` in your path, google it!

To start to fiddle with it, `python setup.py develop` will install all
requirements and will add `payton` to site-packages. But changes to code will
immediately take effect. (Rather than `install` command).

To kick start, after `python setup.py develop` run `python examples/projectile.py`

And once you see the white ball on the screen, hit `c` from keyboard to unpause
the animation mode and see the projectile motion demo.
