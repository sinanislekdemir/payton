from typing import Union

import numpy as np

from payton.math.matrix import Matrix
from payton.math.vector import Vector3D

GArray = Union[Vector3D, np.ndarray]
GMatrix = Union[Matrix, np.ndarray]
