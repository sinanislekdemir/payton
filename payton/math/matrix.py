from typing import List, Union

import numpy as np

from payton.math.vector import Vector3D

DIFF = 0.0000001

Matrix = Union[List[Vector3D], np.ndarray]

IDENTITY_MATRIX: Matrix = [
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0],
]

NP_IDENTITY_MATRIX = np.array(IDENTITY_MATRIX, dtype=np.float32)


def bullet_to_matrix(quaternion: List[float]) -> Matrix:
    """Turn Bullet Quaternion to matrix.

    Transforms3d
    ============

    The transforms3d package, including all examples, code snippets and attached
    documentation is covered by the 2-clause BSD license.

    Copyright (c) 2009-2019, Matthew Brett and Christoph Gohlke
    All rights reserved.
    """
    w, x, y, z = quaternion
    Nq = w * w + x * x + y * y + z * z
    if Nq < DIFF:
        return IDENTITY_MATRIX
    s = 2.0 / Nq
    X = x * s
    Y = y * s
    Z = z * s
    wX = w * X
    wY = w * Y
    wZ = w * Z
    xX = x * X
    xY = x * Y
    xZ = x * Z
    yY = y * Y
    yZ = y * Z
    zZ = z * Z
    return [
        [1.0 - (yY + zZ), xY - wZ, xZ + wY, 0],
        [xY + wZ, 1.0 - (xX + zZ), yZ - wX, 0],
        [xZ - wY, yZ + wX, 1.0 - (xX + yY), 0],
        [0, 0, 0, 1],
    ]
