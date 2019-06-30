import numpy as np  # type: ignore
import math
from copy import deepcopy
from functools import lru_cache
from payton.math.vector import normalize_vector
from payton.math.types import GArray

UNIFORM_MATRIX = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]


def create_rotation_matrix(axis: GArray, angle: float) -> np.ndarray:
    """Create rotation matrix along Axis with given angle

    Rotates a matrix around Axis by given angle.
    _(This function is ported from GLScene sources)_

    Args:
      axis: Object matrix
      angle: Angle in radians
      as_numpy: Return result as a numpy array.
    Returns:
      matrix
    """
    sin = math.sin(angle)
    cos = math.cos(angle)
    m_cos = 1 - cos
    axis = normalize_vector(axis)
    result = deepcopy(UNIFORM_MATRIX)

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
    return np.array(result, dtype=np.float32)


@lru_cache(maxsize=512)
def ortho(left: float, right: float, bottom: float, top: float) -> np.ndarray:
    """Create orthographic projection matrix

    Args:
      left: Window (Viewport) left (default: 0)
      right: Window (Viewport) right (default: Window Width)
      bottom: Window (Viewport) bottom (default: Window Height)
      top: Window (Viewport) top (default: 0)

    Returns:
      matrix
    """
    result = deepcopy(UNIFORM_MATRIX)
    result[0][0] = 2 / (right - left)
    result[1][1] = 2 / (top - bottom)
    result[2][2] = -1
    result[3][0] = -(right + left) / (right - left)
    result[3][1] = -(top + bottom) / (top - bottom)
    return np.array(result, dtype=np.float32)
