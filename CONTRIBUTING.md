# Contributing yo Payton

Dear developer! First of all, thank you so much for considering contributing to Payton. You can contribute to Payton in many ways!

* Create a new feature! (Can be a new geometry or a new process, whatever you can do!)
* Add a support for a 3D file format.
* Fix bugs
* Refactor and make it run faster / be more readable.
* Create more and more examples
* Fix commenting errors (grammar mistakes, typos, markdown issues etc)

And here is how you can not contribute:
* You can not modify the contents of `docs` directory as it is auto-generated.

So here is how to proceed.

## For any code change!

* Fork Payton, create a branch with proper branch name like `feature/added_nnn_file_support`. (`feature/.. bugfix/.. example/..`)
* Make your changes
* Write a proper commit message, not too long but long enough to fully describe your changes / additions.
* If your work involves several steps like refactoring a part of code then adding a feature then fixing a bug, please try to split them into different commits.
* Always use **pylama** (https://github.com/klen/pylama), **mypy** (https://github.com/python/mypy) and **black** (https://github.com/python/black) before committing.
  * If you have unavoidable offences for pylama (like high complexity of a function) you can use `# pylama:ignore=...` by well commmenting the reason in commit message.
  * Call Black as: `black . -t py35 -l 79` 
  * Watchout for MYPYPATH! If you haven't used it before, drop me an email on how to set MYPYPATH (`sinan(at)islekdemir.com`)
* mypy is not required for examples.

then just create a Pull Request!

## Code Standards:

* All new methods and classes should be commented at enough level. Remember that, documentation is automatically generated from source code. Be as much descriptive as possible. Level of being descriptive is: Reader of the comment should learn how it is done, not just understand what it does.
* All library contributions should have typehints. Avoid `#type:ignore` unless necessary.
* All new features must have at least one example at `examples` directory to show use case.
* All new features (methods, classes etc) should have default values whenever possible to reduce users work. (Like a default Cube to have 1x1x1 as size)
* If a feature is hard to use, consider it twice.
* If an example requires additional Python Packages, do not add them to requirements. Payton as a Python Package and as a repository are different things. If library does not need it, just mention how to install the requirement at the top of example file as a comment. Like: `# This example requires "requests" package. Can be installed via "pip install requests"`
* Try to avoid using big files in your examples. 
