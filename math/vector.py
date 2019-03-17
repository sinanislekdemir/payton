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

def sub_vector(v1, v2):
    """
    Substract two vectors. Each component of vectors are substracted separately.

    Example usage:

        from payton.math.vector import sub_vector

        v1 = [20.0, 10.0, 0.0]
        v2 = [10.0, 5.0, 0.0]

        result = sub_vector(v1, v2)
        # result = [10.0, 5.0, 0.0]

    """
    return [v1[0] - v2[0],
            v1[1] - v2[1],
            v1[2] - v2[2]]

def cross_product(v1, v2):
    """
    Vector cross product.

    From wikipedia: The cross product `a x b` is defined as a vector `c` that is
    perpendicular (orthogonal) to both a and b, with a direction given by the
    right-hand rule and a magnitude equal to the area of the parallelogram that
    the vectors span.

    Example usage:

        from payton.math.vector import cross_product

        v1 = [10.0, 10.0, 5.0]
        v2 = [0.0, -10.0, -2.0]

        result = cross_product(v1, v2)
        # result = [30.0, 20.0, -100.0]
    """
    return [v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]]

def vector_norm(v):
    """
    Norm of a vector. *aka* length of a vector. *(Scalar value)*

    Example usage:

        from payton.math.vector import vector_norm

        v1 = [3.0, 4.0, 5.0]
        result = vector_norm(v1)
        # result = 7.0710678118654755

    """
    return math.sqrt((v[0] ** 2) + (v[1] ** 2) + (v[2] ** 2))

def normalize_vector(v):
    """
    Normalize the given vector. Turn the given vector into a unit vector, where
    the vector length is equal to 1.0

    Example usage:

        from payton.math.vector import normalize_vector

        v1 = [3.0, 4.0, -6.0]
        result = normalize_vector(v1)
        # result = [0.3841106397986879, 0.5121475197315839, -0.7682212795973759]
    """
    vn = vector_norm(v)
    if vn == 0.0:
        return vn
    invlen = 1.0 / vn

    return [v[0] * invlen,
            v[1] * invlen,
            v[2] * invlen]

def plane_normal(v1, v2, v3):
    """
    Calculate plane normal. Mathematically, three vectors in space define a plane
    and Normal of the plane is the direction that plane faces.

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
    """
    t1 = sub_vector(v2, v1)
    t2 = sub_vector(v3, v1)
    r = cross_product(t1, t2)
    return normalize_vector(r)

def invert_vector(v):
    """
    Invert the given vector by multiplying its components by -1.0 for each.

    Example usage:

        from payton.math.vector import invert_vector

        v1 = [3.0, -4.0, 5.0]
        result = invert_vector(v1)
        # result = [-3.0, 4.0, -5.0]
    """
    return [-v[0], -v[1], -v[2]]
