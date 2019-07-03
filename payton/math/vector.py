"""
Vector geometry functions

A vector is described by a list or a tuple with 3 indices.

Example definition:

    vector_x = 3.0
    vector_y = 4.0
    vector_z = 5.0
    v = (vector_x, vector_y, vector_z)


"""

import math
from typing import List


def sub_vector(v1: List[float], v2: List[float]) -> List:
    """
    Substract two vectors. Each component of vectors are substracted separately

    Example usage:

        from payton.math.vector import sub_vector

        v1 = [20.0, 10.0, 0.0]
        v2 = [10.0, 5.0, 0.0]

        result = sub_vector(v1, v2)
        # result = [10.0, 5.0, 0.0]

    """
    return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]


def cross_product(v1: List[float], v2: List[float]) -> List[float]:
    """
    Vector cross product.

    From wikipedia: The cross product `a x b` is defined as a vector `c` that
    is perpendicular (orthogonal) to both a and b, with a direction given by
    the right-hand rule and a magnitude equal to the area of the
    parallelogram that the vectors span.

    Example usage:

        from payton.math.vector import cross_product

        v1 = [10.0, 10.0, 5.0]
        v2 = [0.0, -10.0, -2.0]

        result = cross_product(v1, v2)
        # result = [30.0, 20.0, -100.0]
    """
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]


def vector_norm(v: List[float]) -> float:
    """
    Norm of a vector. *aka* length of a vector. *(Scalar value)*

    Example usage:

        from payton.math.vector import vector_norm

        v1 = [3.0, 4.0, 5.0]
        result = vector_norm(v1)
        # result = 7.0710678118654755

    """
    return math.sqrt((v[0] ** 2) + (v[1] ** 2) + (v[2] ** 2))


def normalize_vector(v: List[float]) -> List[float]:
    """
    Normalize the given vector. Turn the given vector into a unit vector, where
    the vector length is equal to 1.0

    Example usage:

        from payton.math.vector import normalize_vector

        v1 = [3.0, 4.0, -6.0]
        result = normalize_vector(v1)
        # result = [0.384110639798687, 0.512147519731583, -0.768221279597375]
    """
    vn = vector_norm(v)
    if vn == 0.0:
        return v
    invlen = 1.0 / vn
    res = [n * invlen for n in v]

    return res


def plane_normal(
    v1: List[float], v2: List[float], v3: List[float]
) -> List[float]:
    """
    Calculate plane normal. Mathematically, three vectors in space define a
    plane and Normal of the plane is the direction that plane faces.

    This is important in terms of calculating the light shading etc. OpenGL
    calculates how a triangle or a plane is lit by looking at the direction
    that it faces.

    Plane normal is a unit vector.

    Example usage:

        from payton.math.vector import plane_normal

        v1 = [0.0, 0.0, 0.0]
        v2 = [5.0, 0.0, 0.0]
        v3 = [5.0, -0.3, 5.0] # Slightly at front.

        result = plane_normal(v1, v2, v3)
        # result = [0.0, -0.9982048454657786, -0.05989229072794672]

    Args:
      v1: Plane point 1
      v2: Plane point 2
      v3: Plane point 3

    Returns:
      vector
    """
    t1 = sub_vector(v2, v1)
    t2 = sub_vector(v3, v1)
    r = cross_product(t1, t2)
    return normalize_vector(r)


def vector_transform(v: List[float], matrix: List[List[float]]) -> List[float]:
    """Transform Vector by a matrix.

    Assuming that the base matrix for the scene is a uniform matrix,
    this function converts a local vector to scen matrix. So, this can
    be assumed as calculating the absolute vertex from a local matrix.

    Args:
      v: Vector to transform
      matrix: Local matrix

    Returns:
      vector
    """
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
    """
    Invert the given vector by multiplying its components by -1.0 for each.

    Example usage:

        from payton.math.vector import invert_vector

        v1 = [3.0, -4.0, 5.0]
        result = invert_vector(v1)
        # result = [-3.0, 4.0, -5.0]
    """
    return [-v[0], -v[1], -v[2]]
