import os

import numpy as np
from PIL import Image

from payton.scene import Scene
from payton.scene.geometry import Mesh
from payton.scene.gui import info_box

scene = Scene()
mesh = Mesh()

texture = os.path.join(os.path.dirname(__file__), "engraved.jpg")

print("Building the engraving")

img = Image.open(texture)
arr = np.asarray(img, dtype=np.float32)
heights = (arr[:, :, 0] if arr.ndim == 3 else arr) / 255.0

h, w = heights.shape
aspect = w / h
pixel_size_x = 5.0 / w
pixel_size_y = 5.0 / (h * aspect)

# Coordinate & texture grids
xi = np.arange(w - 1, dtype=np.float32)
xj = xi + 1
yj = np.arange(h - 1, dtype=np.float32)
yj2 = yj + 1

z_tl = heights[:-1, :-1].ravel()  # top-left  corner
z_tr = heights[:-1, 1:].ravel()   # top-right
z_br = heights[1:, 1:].ravel()    # bottom-right
z_bl = heights[1:, :-1].ravel()   # bottom-left

XI, YJ = np.meshgrid(xi * pixel_size_x, yj * pixel_size_y)
XJ, _ = np.meshgrid(xj * pixel_size_x, yj * pixel_size_y)
_, YJ2 = np.meshgrid(xi * pixel_size_x, yj2 * pixel_size_y)

UI, VJ = np.meshgrid(xi / w, yj / h)
UI2, _ = np.meshgrid(xj / w, yj / h)
_, VJ2 = np.meshgrid(xi / w, yj2 / h)

XI = XI.ravel(); XJ = XJ.ravel()
YJ = YJ.ravel(); YJ2 = YJ2.ravel()
UI = UI.ravel(); UI2 = UI2.ravel()
VJ = VJ.ravel(); VJ2 = VJ2.ravel()

nq = (w - 1) * (h - 1)  # number of quads

# Build vertex positions (6 verts per quad)
verts = np.empty((nq * 6, 3), dtype=np.float32)
verts[0::6, 0] = XI;   verts[0::6, 1] = YJ;  verts[0::6, 2] = z_tl
verts[1::6, 0] = XJ;   verts[1::6, 1] = YJ;  verts[1::6, 2] = z_tr
verts[2::6, 0] = XJ;   verts[2::6, 1] = YJ2; verts[2::6, 2] = z_br
verts[3::6, 0] = XI;   verts[3::6, 1] = YJ;  verts[3::6, 2] = z_tl
verts[4::6, 0] = XJ;   verts[4::6, 1] = YJ2; verts[4::6, 2] = z_br
verts[5::6, 0] = XI;   verts[5::6, 1] = YJ2; verts[5::6, 2] = z_bl

# Build texture coordinates
tex = np.empty((nq * 6, 2), dtype=np.float32)
tex[0::6, 0] = UI;  tex[0::6, 1] = VJ
tex[1::6, 0] = UI2; tex[1::6, 1] = VJ
tex[2::6, 0] = UI2; tex[2::6, 1] = VJ2
tex[3::6, 0] = UI;  tex[3::6, 1] = VJ
tex[4::6, 0] = UI2; tex[4::6, 1] = VJ2
tex[5::6, 0] = UI;  tex[5::6, 1] = VJ2

# Compute face normals for the 2 triangles per quad
e1_t1 = verts[1::6] - verts[0::6]
e2_t1 = verts[2::6] - verts[0::6]
e1_t2 = verts[4::6] - verts[3::6]
e2_t2 = verts[5::6] - verts[3::6]
n_t1 = np.cross(e1_t1, e2_t1)
n_t2 = np.cross(e1_t2, e2_t2)
n_len1 = np.linalg.norm(n_t1, axis=1, keepdims=True)
n_len2 = np.linalg.norm(n_t2, axis=1, keepdims=True)
n_t1 /= np.where(n_len1 == 0, 1, n_len1)
n_t2 /= np.where(n_len2 == 0, 1, n_len2)

# Build vertex index grid (w × h unique vertices)
I, J = np.meshgrid(np.arange(w - 1), np.arange(h - 1))
I = I.ravel(); J = J.ravel()
v_tl = J * w + I
v_tr = J * w + (I + 1)
v_br = (J + 1) * w + (I + 1)
v_bl = (J + 1) * w + I

# Accumulate face normals into vertex normals (average of adjacent faces)
vert_normals = np.zeros((w * h, 3), dtype=np.float32)
np.add.at(vert_normals, v_tl, n_t1)
np.add.at(vert_normals, v_tr, n_t1)
np.add.at(vert_normals, v_br, n_t1)
np.add.at(vert_normals, v_tl, n_t2)
np.add.at(vert_normals, v_br, n_t2)
np.add.at(vert_normals, v_bl, n_t2)
vn_len = np.linalg.norm(vert_normals, axis=1, keepdims=True)
vert_normals /= np.where(vn_len == 0, 1, vn_len)

# Assign vertex normals to the 6 vertices per quad for smooth shading
normals = np.empty_like(verts)
normals[0::6] = vert_normals[v_tl]
normals[1::6] = vert_normals[v_tr]
normals[2::6] = vert_normals[v_br]
normals[3::6] = vert_normals[v_tl]
normals[4::6] = vert_normals[v_br]
normals[5::6] = vert_normals[v_bl]

# Assign to mesh internals (bypass add_triangle overhead)
mesh._vertices = verts.tolist()
mesh._texcoords = tex.tolist()
mesh._normals = normals.tolist()

nv = nq * 6
mesh._indices = [[i, i + 1, i + 2] for i in range(0, nv, 3)]
mesh.materials["default"]._indices = list(mesh._indices)
mesh.refresh()

print("Done, now loading the scene")
mesh.material.texture = texture
mesh.position = [-2.5, -3, 0]
scene.add_object("mesh", mesh)

info = "{} vertices loaded".format(nv)
scene.add_object("info", info_box(10, 10, info))
scene.run()
