"""Sweep and tube mesh generation tools."""

import math
from typing import List, Optional

from payton.math.functions import (
    add_vectors,
    cross_product,
    normalize_vector,
    scale_vector,
    sub_vector,
    vector_norm,
)
from payton.scene.geometry import Line, Mesh


def _frenet_frame(
    p: List[float], p_next: List[float], prev_frame: Optional[List[List[float]]]
) -> List[List[float]]:
    tangent = normalize_vector(sub_vector(p_next, p))
    if vector_norm(tangent) < 1e-10:
        if prev_frame is None:
            prev_frame = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        return prev_frame
    if prev_frame is None:
        if abs(tangent[0]) < 0.9:
            up = normalize_vector(cross_product(tangent, [1.0, 0.0, 0.0]))
        else:
            up = normalize_vector(cross_product(tangent, [0.0, 1.0, 0.0]))
        right = normalize_vector(cross_product(tangent, up))
        up = normalize_vector(cross_product(right, tangent))
    else:
        right = normalize_vector(cross_product(tangent, prev_frame[1]))
        if vector_norm(right) < 1e-10:
            right = prev_frame[0]
        up = normalize_vector(cross_product(right, tangent))
        right = normalize_vector(cross_product(tangent, up))
    return [right, up, tangent]


def sweep(profile: Line, path: Line, closed: bool = False) -> Mesh:
    """Sweep a 2D profile along a 3D path to form a mesh.

    At each point along the path, the profile is placed perpendicular to the
    path direction and connected to adjacent profiles with triangles.

    Keyword arguments:
    profile -- Cross-section profile Line (defines the shape)
    path -- 3D path Line to sweep along
    closed -- If True, connect the last profile back to the first
    """
    profile_verts = profile._vertices
    path_verts = path._vertices

    if len(path_verts) < 2:
        raise ValueError("Path must have at least 2 vertices")
    if len(profile_verts) < 2:
        raise ValueError("Profile must have at least 2 vertices")

    transformed: List[List[List[float]]] = []
    frame = None

    for i in range(len(path_verts)):
        if i < len(path_verts) - 1:
            frame = _frenet_frame(path_verts[i], path_verts[i + 1], frame)
        else:
            # Last point: use forward direction from previous to current
            frame = _frenet_frame(path_verts[i - 1], path_verts[i], frame)

        t_verts = []
        for v in profile_verts:
            # Map profile (defined in XY plane with Z forward) to the Frenet frame
            p = add_vectors(
                path_verts[i],
                add_vectors(
                    scale_vector(frame[0], v[0]),
                    add_vectors(
                        scale_vector(frame[1], v[1]),
                        scale_vector(frame[2], v[2]),
                    ),
                ),
            )
            t_verts.append(p)
        transformed.append(t_verts)

    mesh = Mesh()
    for i in range(len(transformed) - 1):
        curr = transformed[i]
        nxt = transformed[i + 1]
        for j in range(len(profile_verts) - 1):
            mesh.add_triangle(
                vertices=[nxt[j], curr[j + 1], curr[j]],
                texcoords=[[0.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            )
            mesh.add_triangle(
                vertices=[nxt[j], nxt[j + 1], curr[j + 1]],
                texcoords=[[0.0, 1.0], [1.0, 1.0], [1.0, 0.0]],
            )

    if closed:
        curr = transformed[-1]
        nxt = transformed[0]
        for j in range(len(profile_verts) - 1):
            mesh.add_triangle(
                vertices=[nxt[j], curr[j + 1], curr[j]],
                texcoords=[[0.0, 1.0], [1.0, 0.0], [0.0, 0.0]],
            )
            mesh.add_triangle(
                vertices=[nxt[j], nxt[j + 1], curr[j + 1]],
                texcoords=[[0.0, 1.0], [1.0, 1.0], [1.0, 0.0]],
            )

    mesh.fix_normals()
    return mesh


def tube(path: Line, radius: float, segments: int = 8) -> Mesh:
    """Generate a tubular mesh along a 3D path.

    At each point on the path, a circle of *radius* is placed perpendicular
    to the path direction and connected to adjacent circles with triangles.

    Keyword arguments:
    path -- 3D path Line to follow
    radius -- Radius of the tube
    segments -- Number of radial segments (default 8)
    """
    profile_verts: List[List[float]] = []
    for i in range(segments + 1):
        angle = 2.0 * math.pi * i / segments
        profile_verts.append([radius * math.cos(angle), radius * math.sin(angle), 0.0])
    profile = Line(vertices=profile_verts)
    return sweep(profile, path, closed=False)
