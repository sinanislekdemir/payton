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


def matrix_to_position_and_quaternion(
    matrix: Matrix,
) -> tuple[list[float], tuple[float, float, float, float]]:
    """Convert a 4x4 transformation matrix to position and quaternion.

    Args:
        matrix (list of list): A 4x4 transformation matrix.

    Returns:
        tuple: (position, quaternion)
    """
    # Ensure the input is in the form of a numpy array
    matrix = np.array(matrix)

    # Extract the position (translation) part
    position = matrix[3, :3]

    # Extract the rotation part
    rotation_matrix = matrix[:3, :3]
    rotation_matrix = rotation_matrix.T

    # Calculate quaternion components
    trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]

    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
        y = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
        z = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
    else:
        if (
            rotation_matrix[0, 0] > rotation_matrix[1, 1]
            and rotation_matrix[0, 0] > rotation_matrix[2, 2]
        ):
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
                - rotation_matrix[2, 2]
            )
            w = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
            x = 0.25 * s
            y = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            z = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
        elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[1, 1]
                - rotation_matrix[0, 0]
                - rotation_matrix[2, 2]
            )
            w = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
            x = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
            y = 0.25 * s
            z = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(
                1.0
                + rotation_matrix[2, 2]
                - rotation_matrix[0, 0]
                - rotation_matrix[1, 1]
            )
            w = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
            x = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            y = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            z = 0.25 * s

    quaternion = (x, y, z, w)

    return position.tolist(), quaternion
