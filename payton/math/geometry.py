from typing import List, Optional, Tuple, Union

import numpy as np  # type: ignore
import pyrr

from payton.math.types import GArray
from payton.math.vector import distance as distance_native

DIFF = 0.0000001


def point_project(p: GArray, origin: GArray, direction: GArray) -> float:
    return direction[0] * (p[0] - origin[0]) + direction[1] * (p[1] - origin[1]) + direction[2] * (p[2] - origin[2])


def distance(v1: np.ndarray, v2: np.ndarray) -> float:
    v3 = v2 - v1
    return pyrr.vector3.length(v3)


def distance2(v1: np.ndarray, v2: np.ndarray) -> float:
    v1 = v1[:3]
    v2 = v2[:3]
    v3 = v2 - v1
    return pyrr.vector3.length(v3) ** 2


def combine(v1: GArray, v2: GArray, f1: float, f2: float) -> np.ndarray:
    x = (f1 * v1[0]) + (f2 * v2[0])
    y = (f1 * v1[1]) + (f2 * v2[1])
    z = (f1 * v1[2]) + (f2 * v2[2])

    if len(v1) > 3 and len(v2) > 3:
        w = (f1 * v1[3]) + (f2 * v2[3])
        return np.array([x, y, z, w], dtype=np.float32)
    return np.array([x, y, z], dtype=np.float32)


def combine3(v1: GArray, v2: GArray, v3: GArray, f1: float, f2: float, f3: float) -> np.ndarray:
    res = [
        (f1 * v1[0]) + (f2 * v2[0]) + (f3 * v3[0]),
        (f1 * v1[1]) + (f2 * v2[1]) + (f3 * v3[1]),
        (f1 * v1[2]) + (f2 * v2[2]) + (f3 * v3[2]),
        0,
    ]
    return np.array(res, dtype=np.float32)


def raycast_sphere_intersect(start: GArray, vector: GArray, sphere_center: GArray, sphere_radius: float) -> bool:
    proj = point_project(sphere_center, start, vector)
    if proj < 0:
        proj = 0.0
    vc = combine(start, vector, 1.0, proj)
    dist = distance2(sphere_center, vc)
    return dist < (sphere_radius ** 2)


def raycast_box_intersect(start: GArray, vector: GArray, box_a: GArray, box_b: GArray) -> Optional[List[float]]:
    result = True
    plane = [0.0, 0.0, 0.0]
    max_dist = [0.0, 0.0, 0.0]
    res_afv = [0.0, 0.0, 0.0]
    is_middle = [False, False, False]

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
    if result:
        return start

    plane_id = 0
    for i in range(3):
        if is_middle[i] or vector[i] == 0.0:
            max_dist[i] = -1
        else:
            max_dist[i] = (plane[i] - start[i]) / vector[i]
            if max_dist[i] > 0.0:
                if max_dist[plane_id] < max_dist[i]:
                    plane_id = i
                result = True

    if result:
        for i in range(3):
            if plane_id == i:
                res_afv[i] = plane[i]
            else:
                res_afv[i] = start[i] + max_dist[plane_id] * vector[i]
                result = (res_afv[i] >= box_a[i]) and (res_afv[i] <= box_b[i])
                if not result:
                    return None
        return res_afv
    return None


def point_on_line(point: List[float], start: List[float], end: List[float]):
    ab = distance_native(end, start)
    ap = distance_native(point, start)
    bp = distance_native(end, point)
    return abs(ap + bp - ab) < DIFF


def raycast_plane_intersect(
    start: GArray, vector: GArray, plane_point: GArray, plane_normal: GArray
) -> Optional[np.ndarray]:
    d = np.dot(vector, plane_normal)
    res = (d > DIFF) or (d < -DIFF)
    if not res:
        return None
    sp = np.subtract(plane_point, start)
    d = 1.0 / d
    t = np.dot(sp, plane_normal) * d
    if t <= 0:
        return None
    return combine(start, vector, 1.0, t)


def raycast_triangle_intersect(
    start: GArray, vector: GArray, p1: GArray, p2: GArray, p3: GArray
) -> Tuple[Union[np.ndarray, None], Union[np.ndarray, None]]:
    """Returns intersection point, intersection normal"""
    v1 = np.subtract(p2, p1)
    v2 = np.subtract(p3, p1)
    pvec = np.cross(vector, v2)
    det = np.dot(v1, pvec)
    if (det < DIFF) and (det > -DIFF):
        return None, None
    inv_det = 1.0 / det
    tvec = np.subtract(start, p1)
    u = np.dot(tvec, pvec) * inv_det
    if u < 0 or u > 1:
        return None, None
    qvec = np.cross(tvec, v1)
    v = np.dot(vector, qvec) * inv_det
    res = (v > 0) and (u + v <= 1.0)
    if not res:
        return None, None
    t = np.dot(v2, qvec) * inv_det
    if t <= 0:
        return None, None
    ip = combine(start, vector, 1, t)
    inor = np.cross(v1, v2)
    inor /= np.linalg.norm(inor)
    return ip, inor


def line_triangle_intersect(
    start: np.ndarray, end: np.ndarray, p1: np.ndarray, p2: np.ndarray, p3: np.ndarray,
) -> bool:
    direction = end - start
    norm = np.linalg.norm(direction)
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


def point_in_poly_2D(x: float, y: float, poly: List[List[float]]) -> bool:
    # Originally copied from
    # http://www.ariel.com.au/a/python-point-int-poly.html
    n = len(poly)
    inside = False

    p1x, p1y = poly[0]
    for i in range(n + 1):
        p2x, p2y = poly[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y

    return inside
