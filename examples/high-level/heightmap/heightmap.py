#!/usr/bin/env python3

# Create a heightmap image from given 3d Model
# This is a very slow and brutal way to do this
# It would be much faster by simply dumping the depth buffer
# But this is for demonstration purposes. We code for humans here.
import argparse
import concurrent.futures
import os

from PIL import Image

from payton.math.geometry import raycast_triangle_intersect
from payton.scene.geometry import Wavefront
from payton.tools.bar import progress

# Command line argument parser
parser = argparse.ArgumentParser(
    formatter_class=argparse.RawDescriptionHelpFormatter,
    description="Heightmap generator from 3D Wavefront object",
)

# Let's define our CLI arguments
parser.add_argument("--file", default="", help="Input filename", required=True)
parser.add_argument("--output", default="out.png", help="Output image filename")
parser.add_argument("--width", default="256", help="Image width, height will be calculated dynamically")

# Parse the incoming arguments
args = parser.parse_args()

# It is good to check things that can fail in the beginning
if not os.path.exists(args.file):
    print(f"File not found: {args.file}")
    exit(1)

if not args.width.isnumeric():
    print(f"Width is not a number: {args.width}")
    exit(1)

# Create the wavefront mesh object
print("Creating the object model")
try:
    wavefront_obj = Wavefront(args.file)
    wavefront_obj.fix_normals()
except Exception as e:
    print("Unable to create the object")
    print(e)
    exit(1)

print("Calculating the object bounding box")
bounding_box = wavefront_obj.bounding_box
print("Object size:")

o_w = bounding_box[1][0] - bounding_box[0][0]
o_h = bounding_box[1][1] - bounding_box[0][1]
o_d = bounding_box[1][2] - bounding_box[0][2]
print(f"  Width:  {o_w}")
print(f"  Height: {o_h}")
print(f"  Depth:  {o_d}")

# Now it is time to calculate our render steps
width = int(args.width)
x_step = o_w / width

height = int(o_h * width / o_w)
y_step = o_h / height

max_z = bounding_box[1][2]
min_z = bounding_box[0][2]
scale_z = 255 / (max_z - min_z)

print("Raycast step size:")
print(f"  X Step: {x_step}")
print(f"  Y Step: {y_step}")
print(f"Image size: {width}x{height}")
print("Object properties")
vc = len(wavefront_obj._vertices)
ic = len(wavefront_obj._indices)

print(f"  Vertices: {vc}")
print(f"  Indices: {ic}")

print("Starting raycast")
total_steps = width * height

img = Image.new(mode="RGB", size=(width, height))
start_position = [bounding_box[0][0], bounding_box[0][1], bounding_box[1][2] + 10]


def intersection(face, pos):
    p1 = wavefront_obj._vertices[face[0]]
    p2 = wavefront_obj._vertices[face[1]]
    p3 = wavefront_obj._vertices[face[2]]
    n = wavefront_obj._normals[face[0]]
    if n[2] < 0:  # Facing downwards
        return -1000
    intersection_point_tmp, _ = raycast_triangle_intersect(pos, [0, 0, -1], p1, p2, p3)
    if intersection_point_tmp is None:
        return -1000
    return intersection_point_tmp[2]


def get_pixel(position):
    x = position[0]
    y = position[1]
    pos = [start_position[0] + (x * x_step), start_position[1] + (y * y_step), start_position[2]]
    total_results = [intersection(face, pos) for face in wavefront_obj._indices]
    maxt = max(total_results)
    return (int((maxt - min_z) * scale_z) if maxt != -1000 else 0, position)


pixels = [(i, j) for i in range(width) for j in range(height)]

image_data = []

with concurrent.futures.ProcessPoolExecutor(max_workers=12) as executor:
    futures = [executor.submit(get_pixel, position) for position in pixels]
    for future in concurrent.futures.as_completed(futures):
        image_data.append(future.result())
        progress(len(image_data), len(pixels), 'Collecting...')

for item in image_data:
    img.putpixel(item[1], (item[0], item[0], item[0]))

img.save(args.output)

breakpoint()
