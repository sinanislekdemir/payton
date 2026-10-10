# Third-party notices

Payton is distributed under the BSD 3-Clause License (see [`LICENSE`](LICENSE)).
Parts of Payton are based on, or derive from, the third-party work below. Their
notices are reproduced here in full.

---

## qu3e

The built-in physics engine in [`payton/scene/internal_physics.py`](payton/scene/internal_physics.py)
is an **original Python implementation**, but its design is **adapted from
qu3e**, a compact C++ 3D physics engine by Randy Gaul. In particular the
following ideas come from qu3e:

* persistent contact manifolds between body pairs;
* contact **warm starting** (re-using accumulated impulses across steps);
* the sequential-impulse solver with clamped, accumulated normal/friction
  impulses;
* Baumgarte stabilisation with a penetration slop;
* per-body sleeping.

This is an **altered / derived work**: none of qu3e's C++ source is copied
verbatim — the algorithms were re-implemented and modified for Python (a
vectorised NumPy solver using colour-batched Gauss–Seidel, a sweep-and-prune
broadphase, and additional sphere / capsule / plane shapes). It must **not** be
mistaken for, or presented as, the original qu3e software.

* Project: qu3e — https://github.com/RandyGaul/qu3e
* Author: Randy Gaul — http://www.randygaul.net

### License (zlib)

```
Copyright (c) 2014 Randy Gaul http://www.randygaul.net

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not
     be misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
```

---

## GLScene

The vector, matrix, rotation and projection helpers in
[`payton/math/`](payton/math/) were inspired by, and in places adapted from,
**GLScene** — the OpenGL scene-graph library for Delphi, C++ and Free Pascal
started by Mike Lischke and maintained by Eric Grange.

As with qu3e, this is an **altered / derived work**: the routines were
re-implemented in Python and must not be presented as the original software.
Any portion that is a direct derivative of GLScene source remains subject to
the Mozilla Public License.

* Project: GLScene — https://glscene.sourceforge.net — https://github.com/glscene/GLScene
* Licence: Mozilla Public License (MPL). The original GLScene was released
  under MPL 1.1; current releases use MPL 2.0 — https://mozilla.org/MPL/2.0/

```
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at https://mozilla.org/MPL/2.0/.
```

---

## Box2D — sequential impulses

The built-in solver follows the **sequential-impulse** method introduced by
Erin Catto ("Sequential Impulses", GDC 2007) and popularised by Box2D. No Box2D
source code is used; the method and its published references are credited here.

* Box2D — https://github.com/erincatto/box2d
* Author: Erin Catto — https://box2d.org
