import math
from copy import deepcopy
from functools import lru_cache
from typing import List

import numpy as np  # type: ignore
import pyrr

from payton.math.types import GArray
from payton.math.vector import normalize_vector

IDENTITY_MATRIX = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def create_rotation_matrix_raw(axis: GArray, angle: float) -> np.ndarray:
    sin = math.sin(angle)
    cos = math.cos(angle)
    m_cos = 1 - cos
    axis = normalize_vector(axis)
    result = deepcopy(IDENTITY_MATRIX)

    result[0][0] = (m_cos * axis[0] * axis[0]) + cos
    result[0][1] = (m_cos * axis[0] * axis[1]) - (axis[2] * sin)
    result[0][2] = (m_cos * axis[2] * axis[0]) + (axis[1] * sin)
    result[0][3] = 0

    result[1][0] = (m_cos * axis[0] * axis[1]) + (axis[2] * sin)
    result[1][1] = (m_cos * axis[1] * axis[1]) + cos
    result[1][2] = (m_cos * axis[1] * axis[2]) - (axis[0] * sin)
    result[1][3] = 0

    result[2][0] = (m_cos * axis[2] * axis[0]) - (axis[1] * sin)
    result[2][1] = (m_cos * axis[1] * axis[2]) + (axis[0] * sin)
    result[2][2] = (m_cos * axis[2] * axis[2]) + cos
    result[2][3] = 0
    return result


def create_rotation_matrix(axis: GArray, angle: float) -> np.ndarray:
    result = create_rotation_matrix_raw(axis, angle)
    return np.array(result, dtype=np.float32)


def scale_matrix(x: float, y: float, z: float) -> np.ndarray:
    result = deepcopy(IDENTITY_MATRIX)
    result[0][0] = x
    result[1][1] = y
    result[2][2] = z
    return np.array(result, dtype=np.float32)


@lru_cache(maxsize=512)
def ortho(left: float, right: float, bottom: float, top: float) -> np.ndarray:
    result = deepcopy(IDENTITY_MATRIX)
    result[0][0] = 2 / (right - left)
    result[1][1] = 2 / (top - bottom)
    result[2][2] = -1
    result[3][0] = -(right + left) / (right - left)
    result[3][1] = -(top + bottom) / (top - bottom)
    return np.array(result, dtype=np.float32)


def cubemap_projection_matrices(from_point: List[float], far_plane: float) -> List[np.ndarray]:
    def a2np(a: List[float]) -> np.ndarray:
        return np.array(a, dtype=np.float32)

    shadow_proj = pyrr.matrix44.create_perspective_projection(90.0, 1.0, 0.01, far_plane, np.float32)
    lightpos = np.array(from_point, dtype=np.float32)

    nx = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([-1.0, 0, 0]), dtype=np.float32,), a2np([0, -1.0, 0]), dtype=np.float32,
    )
    px = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([1, 0, 0]), dtype=np.float32,), a2np([0, -1.0, 0]), dtype=np.float32,
    )
    ny = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([0, -1, 0]), dtype=np.float32,), a2np([0, 0, -1.0]), dtype=np.float32,
    )
    py = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([0, 1, 0]), dtype=np.float32,), a2np([0, 0, 1.0]), dtype=np.float32,
    )
    pz = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([0, 0, 1]), dtype=np.float32,), a2np([0, -1.0, 0]), dtype=np.float32,
    )
    nz = pyrr.matrix44.create_look_at(
        lightpos, np.array(lightpos + a2np([0, 0, -1]), dtype=np.float32,), a2np([0, -1.0, 0]), dtype=np.float32,
    )

    return [
        px.dot(shadow_proj),
        nx.dot(shadow_proj),
        py.dot(shadow_proj),
        ny.dot(shadow_proj),
        pz.dot(shadow_proj),
        nz.dot(shadow_proj),
    ]
