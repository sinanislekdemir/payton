from collections.abc import Callable
from typing import Any

from payton.math.vector import Vector3D

VList = list[Vector3D]
VListW = list[Vector3D]
IList = list[list[int]]
CPlane = tuple[Vector3D, Vector3D, Callable[[Vector3D], Any]]
