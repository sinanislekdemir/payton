import math
from copy import deepcopy
from functools import lru_cache
from typing import Iterable, List, Tuple

import numpy as np
import pyrr

from payton.math.matrix import IDENTITY_MATRIX, Matrix
from payton.math.vector import Vector3D


def to_4(v: Iterable, fill: float = 1.0) -> Vector3D:
    res = list(v)
    for _ in range(4 - len(res)):
        res.append(fill)
    return res


def sub_vector(v1: Vector3D, v2: Vector3D) -> Vector3D:
    """Substract vector V1 from vector V2 and return resulting Vector.

    Keyword arguments:
    v1 -- Vector 1
    v2 -- Vector 2
    """
    return [v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]]


def cross_product(v1: Vector3D, v2: Vector3D) -> Vector3D:
    """Return the cross product vector of V1 and V2.

    Keyword arguments:
    v1 -- Vector 1
    v2 -- Vector 2
    """
    return [
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0],
    ]


def vector_norm(v: Vector3D) -> float:
    """Calculate and return the Norm of a given Vector."""
    return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))


def normalize_vector(v: Vector3D) -> Vector3D:
    """
    Normalize given vector and output normalized vector.

    Keyword Arguments:
    v -- Vector to normalize
    """
    vn = vector_norm(v)
    if vn == 0.0:
        return v
    invlen = 1.0 / vn
    return [v[0] * invlen, v[1] * invlen, v[2] * invlen]


def plane_normal(v1: Vector3D, v2: Vector3D, v3: Vector3D) -> Vector3D:
    """
    Calculate the normal of the plane defined by v1, v2, v3 Vectors in space.

    Keyword arguments:
    v1 -- V1 Vector of the plane (Corner of the triangle in the given plane)
    v2 -- V2 Vector of the plane (Corner of the triangle in the given plane)
    v3 -- V3 Vector of the plane (Corner of the triangle in the given plane)
    """
    return normalize_vector(cross_product(sub_vector(v2, v1), sub_vector(v3, v1)))


def vector_transform(v: Vector3D, matrix: Matrix) -> Vector3D:
    """
    Transform the given Vector into given Matrix coordinates.

    Keyword arguments:
    v -- Vector to transform
    matrix -- Matrix for the transformation
    """
    mx = matrix[0]
    my = matrix[1]
    mz = matrix[2]
    mw = matrix[3]

    rx = v[0] * mx[0] + v[1] * my[0] + v[2] * mz[0]
    ry = v[0] * mx[1] + v[1] * my[1] + v[2] * mz[1]
    rz = v[0] * mx[2] + v[1] * my[2] + v[2] * mz[2]

    return [rx + mw[0], ry + mw[1], rz + mw[2]]


def invert_vector(v: Vector3D) -> Vector3D:
    """
    Invert - Negate the given vector.

    Keyword arguments:
    v -- Vector to invert
    """
    return [-v[0], -v[1], -v[2]]


def min_max(vlist: List[Vector3D]) -> Tuple[Vector3D, Vector3D]:
    """
    Calculate the smallest and biggest components of the given vector list.

    Returns the result as Tuple of Vectors as minVector - maxVector

    Example:
        vlist = [
            Vector(-2, 3, 4),
            Vector(-3, -5, 9)
        ]

        min_max(vlist)

    would return:

        ([-3, -5, 4], [-2, 3, 9])


    Keyword arguments:
    vlist -- List of vectors
    """
    x_list = [item[0] for item in vlist]
    y_list = [item[1] for item in vlist]
    z_list = [item[2] for item in vlist]
    return (
        [min(x_list), min(y_list), min(z_list)],
        [max(x_list), max(y_list), max(z_list)],
    )


def scale_vector(v: Vector3D, factor: float) -> Vector3D:
    """
    Scale (multiply each component) of the given Vector and return as a result.

    Keyword arguments:
    v -- Vector to scale
    factor -- factor to multiply
    """
    return [v[0] * factor, v[1] * factor, v[2] * factor]


def add_vectors(v1: Vector3D, v2: Vector3D) -> Vector3D:
    """
    Add two vectors.

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    return [v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]]


def dot_product(v1: Vector3D, v2: Vector3D) -> float:
    """
    Return vector dot product of v1 and v2.

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def vector_angle(v1: Vector3D, v2: Vector3D) -> float:
    """
    Calculate the angle between two given vectors.

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    v1_n = normalize_vector(v1)
    v2_n = normalize_vector(v2)
    return math.acos(dot_product(v1_n, v2_n) / (vector_norm(v1_n) * vector_norm(v2_n)))


def mid_point(v1: Vector3D, v2: Vector3D) -> Vector3D:
    """
    Calculate the middle point between v1 and v2 vectors.

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    diff = sub_vector(v2, v1)
    leng = vector_norm(diff)
    diff = normalize_vector(diff)
    diff = scale_vector(diff, leng / 2.0)
    return add_vectors(diff, v1)


def distance(v1: Vector3D, v2: Vector3D) -> float:
    """
    Calculate the distance between vectors v1 and v2.

    Keyword arguments:
    v1 -- First vector
    v2 -- Second vector
    """
    diff = sub_vector(v2, v1)
    return vector_norm(diff)


def rotate_around_z(v: Vector3D, angle: float) -> Vector3D:
    """Rotate the given vector around Z axis by given degrees.

    Keyword arguments:
    v -- Vector to rotate
    angle -- Angle of rotation in Radians
    """
    s = math.sin(angle)
    c = math.cos(angle)
    return [v[0] * c + v[1] * s, v[1] * c - v[0] * s, v[2]]


def create_rotation_matrix_raw(axis: Vector3D, angle: float) -> Matrix:
    """
    Create a rotation matrix as Numpy array.

    If you transform a Vector by the rotation Matrix, you will be rotating
    the given Vector according to the rotation defined by the matrix.

    If you multiply a matrix by the rotation matrix, resulting matrix will be rotated
    by the given rotation matrix.

    Keyword arguments:
    axis -- Rotation axis
    angle -- Angle of rotation in Radians
    """
    sin = math.sin(angle)
    cos = math.cos(angle)
    m_cos = 1 - cos
    axis = normalize_vector(axis)
    return [
        [
            (m_cos * axis[0] * axis[0]) + cos,
            (m_cos * axis[0] * axis[1]) - (axis[2] * sin),
            (m_cos * axis[2] * axis[0]) + (axis[1] * sin),
            0,
        ],
        [
            (m_cos * axis[0] * axis[1]) + (axis[2] * sin),
            (m_cos * axis[1] * axis[1]) + cos,
            (m_cos * axis[1] * axis[2]) - (axis[0] * sin),
            0,
        ],
        [
            (m_cos * axis[2] * axis[0]) - (axis[1] * sin),
            (m_cos * axis[1] * axis[2]) + (axis[0] * sin),
            (m_cos * axis[2] * axis[2]) + cos,
            0,
        ],
        [0.0, 0.0, 0.0, 1.0],
    ]


def create_rotation_matrix(axis: Vector3D, angle: float) -> np.ndarray:
    """
    Create rotation matrix as a numpy array.

    Numpy array is a well packed proper memory array that fits well into
    graphics card. Therefore, we are using Numpy Arrays to push stuff to
    the graphics card.

    If you transform a Vector by the rotation Matrix, you will be rotating
    the given Vector according to the rotation defined by the matrix.

    If you multiply a matrix by the rotation matrix, resulting matrix will be rotated
    by the given rotation matrix.

    Keyword arguments:
    axis -- Rotation axis
    angle -- Angle of rotation in Radians.
    """
    result = create_rotation_matrix_raw(axis, angle)
    return np.array(result, dtype=np.float32)


def scale_matrix(x: float, y: float, z: float) -> np.ndarray:
    """
    Create a scale matrix.

    If you multiply an objects matrix with a scale matrix, it will scale / resize
    the coordinates relative to the object. As a result, the object will look resized.

    Keyword arguments:
    x -- X Scale Factor
    y -- Y Scale Factor
    z -- Z Scale Factor
    """
    result = deepcopy(IDENTITY_MATRIX)
    result = [[x, 0, 0, 0], [0, y, 0, 0], [0, 0, z, 0], [0, 0, 0, 1]]
    return np.array(result, dtype=np.float32)


@lru_cache(maxsize=512)
def ortho(left: float, right: float, bottom: float, top: float) -> np.ndarray:
    """
    Create an orthographic projection matrix as a Numpy Array.

    Keyword arguments:
    left -- Viewport left side (-x) coordinate
    right -- Viewport right side (+x) coordinate
    bottom -- Viewport bottom side (+y) coordinate
    top -- Viewport top side (-y) coordinate
    """
    result = [
        [2 / (right - left), 0, 0, 0],
        [0, 2 / (top - bottom), 0, 0],
        [0, 0, -1, 0],
        [-(right + left) / (right - left), -(top + bottom) / (top - bottom), 0, 1],
    ]
    return np.array(result, dtype=np.float32)


def cubemap_projection_matrices(from_point: Vector3D, far_plane: float) -> List[np.ndarray]:
    """
    Create the required Cubemap projection matrices.

    This method is suitable for generating a Shadow Map.

    Simply speaking, this method generates 6 different camera matrices from the center of
    an imaginary cube and covers all surfaces without conflicting.

    Keyword arguments;
    from_point -- Imaginary camera location
    far_plane -- How far the camera is capable of seeing. (Effects performance!)
    """

    def a2np(a: List[float]) -> np.ndarray:
        return np.array(a, dtype=np.float32)

    shadow_proj = pyrr.matrix44.create_perspective_projection(90.0, 1.0, 0.01, far_plane, np.float32)
    lightpos = np.array(list(from_point), dtype=np.float32)[:3]

    nx = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([-1.0, 0, 0]),
            dtype=np.float32,
        ),
        a2np([0, -1.0, 0]),
        dtype=np.float32,
    )
    px = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([1, 0, 0]),
            dtype=np.float32,
        ),
        a2np([0, -1.0, 0]),
        dtype=np.float32,
    )
    ny = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([0, -1, 0]),
            dtype=np.float32,
        ),
        a2np([0, 0, -1.0]),
        dtype=np.float32,
    )
    py = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([0, 1, 0]),
            dtype=np.float32,
        ),
        a2np([0, 0, 1.0]),
        dtype=np.float32,
    )
    pz = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([0, 0, 1]),
            dtype=np.float32,
        ),
        a2np([0, -1.0, 0]),
        dtype=np.float32,
    )
    nz = pyrr.matrix44.create_look_at(
        lightpos,
        np.array(
            lightpos + a2np([0, 0, -1]),
            dtype=np.float32,
        ),
        a2np([0, -1.0, 0]),
        dtype=np.float32,
    )

    return [
        px.dot(shadow_proj),
        nx.dot(shadow_proj),
        py.dot(shadow_proj),
        ny.dot(shadow_proj),
        pz.dot(shadow_proj),
        nz.dot(shadow_proj),
    ]
