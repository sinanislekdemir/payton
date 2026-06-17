#!/usr/bin/env python3

# Create a heightmap image from given 3d Model
import argparse
import concurrent.futures
import os
import sys
from math import ceil, floor

from PIL import Image

from payton.math.geometry import ray_intersect_neg_z
from payton.scene.geometry import Wavefront
from payton.tools.bar import progress

parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Heightmap generator from 3D Wavefront object",
)
parser.add_argument("--file", default="", help="Input filename", required=True)
parser.add_argument("--output", default="out.png", help="Output image filename")
parser.add_argument(
    "--width", default="256", help="Image width, height will be calculated dynamically"
)

args = parser.parse_args()

if not os.path.exists(args.file):
    print(f"File not found: {args.file}")
    sys.exit(1)

if not args.width.isnumeric():
    print(f"Width is not a number: {args.width}")
    sys.exit(1)

print("Creating the object model")
try:
    wavefront_obj = Wavefront(args.file)
    wavefront_obj.fix_normals()
except Exception as e:
    print("Unable to create the object")
    print(e)
    sys.exit(1)

print("Calculating the object bounding box")
bbox = wavefront_obj.bounding_box
o_w = bbox[1][0] - bbox[0][0]
o_h = bbox[1][1] - bbox[0][1]
o_d = bbox[1][2] - bbox[0][2]
print(f"  Width:  {o_w}\n  Height: {o_h}\n  Depth:  {o_d}")

width = int(args.width)
x_step = o_w / width
height = int(o_h * width / o_w) if o_w > 0 else 1
y_step = o_h / height if height > 0 else 1

max_z = bbox[1][2]
min_z = bbox[0][2]
range_z = max_z - min_z
scale_z = 255 / range_z if range_z > 0 else 1

print(f"Image size: {width}x{height}")

# Convert mesh data to compact arrays and build 2D spatial grid
verts = wavefront_obj._vertices
faces = wavefront_obj._indices
normals = wavefront_obj._normals
n_tri = len(faces)
print(f"  Triangles: {n_tri}")

gs_x = min(width, 64)
gs_y = min(height, 64)
grid = [[] for _ in range(gs_x * gs_y)]

for tri_idx in range(n_tri):
    f = faces[tri_idx]
    wx_min = min(verts[f[0]][0], verts[f[1]][0], verts[f[2]][0])
    wx_max = max(verts[f[0]][0], verts[f[1]][0], verts[f[2]][0])
    wy_min = min(verts[f[0]][1], verts[f[1]][1], verts[f[2]][1])
    wy_max = max(verts[f[0]][1], verts[f[1]][1], verts[f[2]][1])

    gx0 = max(0, floor((wx_min - bbox[0][0]) / o_w * gs_x))
    gx1 = min(gs_x - 1, ceil((wx_max - bbox[0][0]) / o_w * gs_x))
    gy0 = max(0, floor((wy_min - bbox[0][1]) / o_h * gs_y))
    gy1 = min(gs_y - 1, ceil((wy_max - bbox[0][1]) / o_h * gs_y))

    for gy in range(gy0, gy1 + 1):
        row_start = gy * gs_x
        for gx in range(gx0, gx1 + 1):
            grid[row_start + gx].append(tri_idx)

grid = [tuple(cell) for cell in grid]

# Module-level globals for worker processes (fork inherits via CoW)
_WORKER_DATA = dict(
    verts=verts, faces=faces, normals=normals, grid=grid, bbox=bbox,
    gs_x=gs_x, gs_y=gs_y, width=width, height=height,
    x_step=x_step, y_step=y_step, min_z=min_z, scale_z=scale_z,
    start_z=bbox[1][2] + 10,
)


def process_rows(y_start, y_end):
    d = _WORKER_DATA
    width = d["width"]
    height = d["height"]
    gs_x = d["gs_x"]
    gs_y = d["gs_y"]
    grid = d["grid"]
    x_step = d["x_step"]
    y_step = d["y_step"]
    bbox = d["bbox"]
    min_z = d["min_z"]
    scale_z = d["scale_z"]
    start_z = d["start_z"]
    normals = d["normals"]
    faces = d["faces"]
    verts = d["verts"]

    results = []
    for y in range(y_start, y_end):
        wy = bbox[0][1] + y * y_step
        gy = min(int(y / height * gs_y), gs_y - 1)
        for x in range(width):
            wx = bbox[0][0] + x * x_step
            gx = min(int(x / width * gs_x), gs_x - 1)
            cell = grid[gy * gs_x + gx]

            max_z_val = -1000.0
            for tri_idx in cell:
                f = faces[tri_idx]
                if normals[f[0]][2] < 0:
                    continue
                z = ray_intersect_neg_z(
                    wx, wy, start_z, verts[f[0]], verts[f[1]], verts[f[2]]
                )
                if z is not None and z > max_z_val:
                    max_z_val = z

            val = int((max_z_val - min_z) * scale_z) if max_z_val != -1000 else 0
            results.append(((x, y), val))
    return results


print("Starting raycast")
n_pixels = width * height
img = Image.new(mode="RGB", size=(width, height))

# Chunk into row groups (fewer tasks, less IPC overhead)
n_workers = 12
rows_per_chunk = max(1, height // n_workers)
chunks = [(y, min(y + rows_per_chunk, height)) for y in range(0, height, rows_per_chunk)]

image_data = []
with concurrent.futures.ProcessPoolExecutor(max_workers=n_workers) as executor:
    futures = [executor.submit(process_rows, cs, ce) for cs, ce in chunks]
    for future in concurrent.futures.as_completed(futures):
        image_data.extend(future.result())
        progress(len(image_data), n_pixels, "Collecting...")

for pos, val in image_data:
    img.putpixel(pos, (val, val, val))

img.save(args.output)
