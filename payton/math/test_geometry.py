import unittest
import numpy as np  # type: ignore
from payton.math.geometry import (
    raycast_triangle_intersect,
    distance,
    distance2,
    point_project,
)


class TestGeometryMethods(unittest.TestCase):
    def test_raycast_triangle_intersect(self):
        start = np.array([0, 0, 5], dtype=np.float32)
        vector = np.array([0, 0, -1], dtype=np.float32)
        p1 = np.array([-1, -1, 0], dtype=np.float32)
        p2 = np.array([1, -1, 0], dtype=np.float32)
        p3 = np.array([0, 1, 0], dtype=np.float32)
        ip, inor = raycast_triangle_intersect(start, vector, p1, p2, p3)
        expected = np.array([0, 0, 0, 1], dtype=np.float32)
        expected_n = np.array([0, 0, 1], dtype=np.float32)
        np.testing.assert_array_almost_equal(ip, expected)
        np.testing.assert_array_almost_equal(inor, expected_n)

    def test_raycast_triangle_intersect_not(self):
        start = np.array([0, 0, 5], dtype=np.float32)
        vector = np.array([0, 0.5, 0.5], dtype=np.float32)
        p1 = np.array([-1, -1, 0], dtype=np.float32)
        p2 = np.array([1, -1, 0], dtype=np.float32)
        p3 = np.array([0, 1, 0], dtype=np.float32)
        ip, inor = raycast_triangle_intersect(start, vector, p1, p2, p3)
        self.assertIsNone(ip)
        self.assertIsNone(inor)

    def test_distance(self):
        p1 = np.array([2, 5, 3], dtype=np.float32)
        p2 = np.array([3, 9, -2], dtype=np.float32)
        d = distance(p1, p2)
        self.assertAlmostEqual(d, 6.4807405)

    def test_distance2(self):
        p1 = np.array([2, 5, 3], dtype=np.float32)
        p2 = np.array([3, 9, -2], dtype=np.float32)
        d = distance2(p1, p2)
        self.assertAlmostEqual(d, 41.999998039865204)

    def test_point_project(self):
        p = [2, 3, 1]
        origin = [4, 6, 9]
        direction = [0, 0, 1]
        expect = -8
        pp = point_project(p, origin, direction)
        self.assertEqual(pp, expect)


if __name__ == "__main__":
    unittest.main()
