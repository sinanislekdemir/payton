from typing import List, Optional, Tuple, Union

import numpy as np
import pyrr

from payton.math.functions import distance as distance_native
from payton.math.types import GArray
from payton.math.vector import Vector3D

DIFF = 0.0000001

"""Few notes about this module.

Most of the methods defined in this module expects and returns Numpy Arrays.
Numpy Arrays are simply C Type Arrays, where all the bytes are packed and ordered
properly. Regular Python arrays have some extra bytes that doesn't play well with
the expectations of Graphics card.

In order to give the graphics card the data type it expects, Numpy Arrays are
ideal"""


def point_project(p: GArray, origin: GArray, direction: GArray) -> float:
    return direction[0] * (p[0] - origin[0]) + direction[1] * (p[1] - origin[1]) + direction[2] * (p[2] - origin[2])


def distance(v1: np.ndarray, v2: np.ndarray) -> float:
    """Distance between two vectors defined as Numpy arrays

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    v3 = v2 - v1
    return pyrr.vector3.length(v3)


def distance2(v1: np.ndarray, v2: np.ndarray) -> float:
    """Square of the distance between two given vectors defined as Numpy arrays

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector"""
    v1 = v1[:3]
    v2 = v2[:3]
    v3 = v2 - v1
    return pyrr.vector3.length(v3) ** 2


def combine(v1: GArray, v2: GArray, f1: float, f2: float) -> np.ndarray:
    """Combines two vectors given by f1 and f2 ratios and returns Numpy array

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    f1 -- First vector's ratio
    f2 -- Second vector's ratio
    """
    x = (f1 * v1[0]) + (f2 * v2[0])
    y = (f1 * v1[1]) + (f2 * v2[1])
    z = (f1 * v1[2]) + (f2 * v2[2])

    return np.array([x, y, z], dtype=np.float32)


def combine3(v1: GArray, v2: GArray, v3: GArray, f1: float, f2: float, f3: float) -> np.ndarray:
    """Combine three vectors with three factors

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    v3 -- Third vector"""
    res = [
        (f1 * v1[0]) + (f2 * v2[0]) + (f3 * v3[0]),
        (f1 * v1[1]) + (f2 * v2[1]) + (f3 * v3[1]),
        (f1 * v1[2]) + (f2 * v2[2]) + (f3 * v3[2]),
        0,
    ]
    return np.array(res, dtype=np.float32)


def raycast_sphere_intersect(start: GArray, direction: GArray, sphere_center: np.ndarray, sphere_radius: float) -> bool:
    """Check if ray intersects given sphere, returns boolean

    Keyword arguments:
    start -- Start of the ray vector
    direction -- Direction vector of the given vector
    sphere_center -- Center of the sphere to test
    sphere_radius -- Radius of the sphere to test"""
    proj = point_project(sphere_center, start, direction)
    if proj < 0:
        proj = 0.0
    vc = combine(start, direction, 1.0, proj)
    dist = distance2(sphere_center, vc)
    return dist < (sphere_radius**2)


def _find_box_mid_for_intersection(start: GArray, box_a: GArray, box_b: GArray) -> Tuple[List[bool], List[float], bool]:
    is_middle = [False, False, False]
    plane = [0.0, 0.0, 0.0]
    result = True
    for i in range(3):
        if start[i] < box_a[i]:
            plane[i] = box_a[i]
            is_middle[i] = False
            result = False
        elif start[i] > box_b[i]:
            plane[i] = box_b[i]
            is_middle[i] = False
            result = False
        else:
            is_middle[i] = True
    return is_middle, plane, result


def raycast_box_intersect(start: GArray, direction: GArray, box_a: GArray, box_b: GArray) -> Optional[Vector3D]:
    """Check if ray intersects given axis-aligned-box, returns point of intersection if True

    AABB - Axis Aligned Bounding Box means the given box is parallel to the X,Y,Z axises.

    Keyword arguments:
    start -- Start of the ray vector
    direction -- Direction vector of the given vector
    box_a -- Starting vector of the box
    box_b -- Ending vector of the box"""
    max_dist = [0.0, 0.0, 0.0]

    is_middle, plane, result = _find_box_mid_for_intersection(start, box_a, box_b)

    plane_id = 0
    for i in range(3):
        if is_middle[i] or direction[i] == 0.0:
            max_dist[i] = -1
        else:
            max_dist[i] = (plane[i] - start[i]) / direction[i]
            if max_dist[i] > 0.0:
                if max_dist[plane_id] < max_dist[i]:
                    plane_id = i
                result = True

    if result:
        tmp = [0.0, 0.0, 0.0]
        for i in range(3):
            if plane_id == i:
                tmp[i] = plane[i]
            else:
                tmp[i] = start[i] + max_dist[plane_id] * direction[i]
                result = (tmp[i] >= box_a[i]) and (tmp[i] <= box_b[i])
                if not result:
                    return None

        return tmp
    return None


def point_on_line(point: Vector3D, start: Vector3D, end: Vector3D) -> bool:
    """Test if the given point is on the line defined by start-end vectors

    Keyword arguments:
    point -- Point to test
    start -- Line start
    end -- Line end
    """
    ab = distance_native(end, start)
    ap = distance_native(point, start)
    bp = distance_native(end, point)
    return abs(ap + bp - ab) < DIFF


def raycast_plane_intersect(
    start: GArray, direction: GArray, plane_point: GArray, plane_normal: GArray
) -> Optional[np.ndarray]:
    """Test if the given ray intersects with the defined plane, returns
    intersection point if True else None

    Keyword arguments:
    start -- Start of the ray vector
    direction -- Direction of the ray vector
    plane_point -- A point on the plane surface to locate the plane
    plane_normal -- Normal of the defined plane"""
    start = start[:3]
    direction = direction[:3]
    plane_point = plane_point[:3]
    plane_normal = plane_normal[:3]
    d = np.dot(direction, plane_normal)  # type: ignore
    res = (d > DIFF) or (d < -DIFF)
    if not res:
        return None
    sp = np.subtract(plane_point, start)
    d = 1.0 / d
    t = np.dot(sp, plane_normal) * d  # type: ignore
    if t <= 0:
        return None
    return combine(start, direction, 1.0, t)


def raycast_triangle_intersect(
    start: GArray, direction: GArray, p1: GArray, p2: GArray, p3: GArray
) -> Tuple[Union[np.ndarray, None], Union[np.ndarray, None]]:
    """Returns intersection point, intersection normal

    Keyword arguments:
    start -- Start of the ray vector
    direction -- Direction of the ray vector
    p1 -- First corner coordinates of the triangle
    p2 -- Second corner coordinates of the triangle
    p3 -- Third corner coordinates of the triangle
    """
    v1 = np.subtract(p2, p1)
    v2 = np.subtract(p3, p1)
    pvec = np.cross(direction, v2)
    det = np.dot(v1, pvec)  # type: ignore
    if (det < DIFF) and (det > -DIFF):
        return None, None
    inv_det = 1.0 / det
    tvec = np.subtract(start, p1)
    u = np.dot(tvec, pvec) * inv_det  # type: ignore
    if u < 0 or u > 1:
        return None, None
    qvec = np.cross(tvec, v1)
    v = np.dot(direction, qvec) * inv_det  # type: ignore
    res = (v > 0) and (u + v <= 1.0)
    if not res:
        return None, None
    t = np.dot(v2, qvec) * inv_det  # type: ignore
    if t <= 0:
        return None, None
    ip = combine(start, direction, 1, t)
    inor = np.cross(v1, v2)
    inor /= np.linalg.norm(inor)  # type: ignore
    return ip, inor


def line_triangle_intersect(
    start: np.ndarray,
    end: np.ndarray,
    p1: np.ndarray,
    p2: np.ndarray,
    p3: np.ndarray,
) -> bool:
    """Check if line intersects triangle, return boolean

    NOTE: Line does not mean Ray!! Ray goes to infinite. Line is finite.

    Keyword arguments:
    start -- Start of the given line segment
    end -- End of the given line segment
    p1 -- First corner coordinates of the triangle
    p2 -- Second corner coordinates of the triangle
    p3 -- Third corner coordinates of the triangle"""
    direction = end - start
    norm = np.linalg.norm(direction)  # type: ignore
    if norm == 0:
        return False
    direction /= norm
    ip, _inor = raycast_triangle_intersect(start, direction, p1, p2, p3)
    if ip is None:
        return False
    # a---c----b
    d1 = distance(start, ip)
    d2 = distance(ip, end)
    d3 = distance(start, end)
    diff = d3 - (d1 + d2)
    return diff < DIFF and diff > -DIFF
