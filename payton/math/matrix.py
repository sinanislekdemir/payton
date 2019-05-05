import numpy as np
import math
from payton.math.vector import normalize_vector


def create_rotation_matrix(axis, angle, as_numpy=False):
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
    result = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
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
    if as_numpy:
        return np.array(result, dtype=np.float32)
    return result
