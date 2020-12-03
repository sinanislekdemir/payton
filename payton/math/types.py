from typing import Union

import numpy as np  # type: ignore
from payton.math.vector import Vector3D
from payton.math.matrix import Matrix

GArray = Union[Vector3D, np.ndarray]
GMatrix = Union[Matrix, np.ndarray]
