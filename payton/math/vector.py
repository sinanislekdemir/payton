import math
from typing import List, Tuple


def sub_vector(v1: List[float], v2: List[float]) -> List:
    return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]


def cross_product(v1: List[float], v2: List[float]) -> List[float]:
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]


def vector_norm(v: List[float]) -> float:
    return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))


def normalize_vector(v: List[float]) -> List[float]:
    vn = vector_norm(v)
    if vn == 0.0:
        return v
    invlen = 1.0 / vn
    res = [n * invlen for n in v]

    return res


def plane_normal(v1: List[float], v2: List[float], v3: List[float]) -> List[float]:
    return normalize_vector(cross_product(sub_vector(v2, v1), sub_vector(v3, v1)))


def vector_transform(v: List[float], matrix: List[List[float]]) -> List[float]:
    mx = matrix[0]
    my = matrix[1]
    mz = matrix[2]
    mw = matrix[3]

    rx = v[0] * mx[0] + v[1] * my[0] + v[2] * mz[0]
    ry = v[0] * mx[1] + v[1] * my[1] + v[2] * mz[1]
    rz = v[0] * mx[2] + v[1] * my[2] + v[2] * mz[2]

    if len(v) == 3:
        rx += mw[0]
        ry += mw[1]
        rz += mw[2]
        return [rx, ry, rz]

    rx += v[3] * mw[0]
    ry += v[3] * mw[1]
    rz += v[3] * mw[2]
    rw = v[0] * mx[3] + v[1] * my[3] + v[2] * mz[3] + v[3] * mw[3]
    return [rx, ry, rz, rw]


def invert_vector(v: List[float]) -> List[float]:
    return [-v[0], -v[1], -v[2]]


def min_max(vlist: List[List[float]]) -> Tuple[List[float], List[float]]:
    x_list = [item[0] for item in vlist]
    y_list = [item[1] for item in vlist]
    z_list = [item[2] for item in vlist]
    return (
        [min(x_list), min(y_list), min(z_list)],
        [max(x_list), max(y_list), max(z_list)],
    )


def scale_vector(v: List[float], d: float) -> List[float]:
    return [q * d for q in v]


def add_vectors(v1: List[float], v2: List[float]) -> List[float]:
    if len(v1) != len(v2):
        raise BaseException("Vector lengths must be equal")
    result = []
    for i in range(len(v1)):
        result.append(v1[i] + v2[i])
    return result


def dot_product(v1: List[float], v2: List[float]) -> float:
    """Return vector dot product of v1 and v2"""
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def vector_angle(v1: List[float], v2: List[float]) -> float:
    v1_n = normalize_vector(v1)
    v2_n = normalize_vector(v2)
    return math.acos(dot_product(v1_n, v2_n) / (vector_norm(v1_n) * vector_norm(v2_n)))


def _ensure_affine(v: List[float]):
    res = v.copy()
    if len(res) == 2:
        res.append(0)
    return res


def mid_point(v1: List[float], v2: List[float]) -> List[float]:
    v1_c = _ensure_affine(v1)
    v2_c = _ensure_affine(v2)

    diff = sub_vector(v2_c, v1_c)
    leng = vector_norm(diff)
    diff = normalize_vector(diff)
    diff = scale_vector(diff, leng / 2.0)
    result = add_vectors(diff, v1_c)
    if len(v1) == 2:
        return result[:2]
    return result


def distance(v1: List[float], v2: List[float]) -> float:
    v1_c = _ensure_affine(v1)
    v2_c = _ensure_affine(v2)
    diff = sub_vector(v2_c, v1_c)
    return vector_norm(diff)


def rotate_around_z(v: List[float], angle: float) -> List[float]:
    s = math.sin(angle)
    c = math.cos(angle)
    return [v[0] * c + v[1] * s, v[1] * c - v[0] * s, v[2]]
