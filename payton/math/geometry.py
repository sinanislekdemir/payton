import numpy as np
import pyrr

DIFF = 0.0000001


def point_project(p, origin, direction):
    return (
        direction[0] * (p[0] - origin[0])
        + direction[1] * (p[1] - origin[1])
        + direction[2] * (p[2] - origin[2])
    )


def distance(v1, v2):
    v3 = v2 - v1
    return pyrr.vector3.length(v3)


def distance2(v1, v2):
    v3 = v2 - v1
    return pyrr.vector3.length(v3) ** 2


def combine(v1, v2, f1, f2):
    x = (f1 * v1[0]) + (f2 * v2[0])
    y = (f1 * v1[1]) + (f2 * v2[1])
    z = (f1 * v1[2]) + (f2 * v2[2])

    if len(v1) > 3 and len(v2) > 3:
        w = (f1 * v1[3]) + (f2 * v2[3])
        return np.array([x, y, z, w], dtype=np.float32)
    return np.array([x, y, z], dtype=np.float32)


def combine3(v1, v2, v3, f1, f2, f3):
    res = [
        (f1 * v1[0]) + (f2 * v2[0]) + (f3 * v3[0]),
        (f1 * v1[1]) + (f2 * v2[1]) + (f3 * v3[1]),
        (f1 * v1[2]) + (f2 * v2[2]) + (f3 * v3[2]),
        0,
    ]
    return np.array(res, dtype=np.float32)


def raycast_sphere_intersect(start, vector, sphere_center, sphere_radius):
    """Raycast Sphere Intersect Test

    Args:
      start: Start of the ray
      vector: Vector (direction) of the ray
      sphere_center: Sphere center in global coordinates
      sphere_radius: Sphere radius

    Returns:
      bool
    """
    proj = point_project(sphere_center, start, vector)
    if proj < 0:
        proj = 0.0
    vc = combine(start, vector, 1.0, proj)
    dist = distance2(sphere_center, vc)
    return dist < (sphere_radius ** 2)


def raycast_plane_intersect(start, vector, plane_point, plane_normal):
    """Raycast Plane Intersection Test

    Args:
      start: Start of the ray
      vector: Vector (direction) of the ray
      plane_point: Point on plane
      plane_norma: Normal of the plane

    Returns:
      intersection: Intersection point or None
    """
    global DIFF
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


def raycast_triangle_intersect(start, vector, p1, p2, p3):
    """Raycast Triangle Intersection Test

    Args:
      start: Start of the ray
      vector: Vector (direction) of the ray
      p1: P1 point of the triangle
      p2: P2 point of the triangle
      p3: P3 point of the triangle

    Returns:
      intersection, normal: Intersectino point and intersection normal
      None, None: if there is no intersection
    """
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


def line_triangle_intersect(start, end, p1, p2, p3):
    global DIFF

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
