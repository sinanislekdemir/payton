from typing import Any, Callable, List, Tuple

from payton.math.vector import Vector3D

VList = List[Vector3D]
VListW = List[Vector3D]
IList = List[List[int]]
CPlane = Tuple[Vector3D, Vector3D, Callable[[Vector3D], Any]]
