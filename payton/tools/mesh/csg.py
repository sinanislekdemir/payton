"""CSG (Constructive Solid Geometry) boolean operations on meshes.

Provides ``csg_union``, ``csg_difference``, and ``csg_intersect`` for
combining two manifold meshes via triangle-level boolean operations.
"""

from copy import deepcopy

from payton.math.functions import (
    add_vectors,
    cross_product,
    dot_product,
    normalize_vector,
    plane_normal,
    scale_vector,
    sub_vector,
    vector_norm,
)
from payton.scene.geometry import Mesh

# ---------------------------------------------------------------------------
#  Internal helpers
# ---------------------------------------------------------------------------

EPSILON = 1e-8


def _plane(
    v0: list[float], v1: list[float], v2: list[float]
) -> tuple[list[float], float]:
    n = plane_normal(v0, v1, v2)
    d = dot_product(n, v0)
    return (n, d)


def _point_plane_side(p: list[float], plane_n: list[float], plane_d: float) -> float:
    dist = dot_product(plane_n, p) - plane_d
    if abs(dist) < EPSILON:
        return 0.0
    return -1.0 if dist < 0 else 1.0


def _tri_plane_intersect(
    tri: list[list[float]], plane_n: list[float], plane_d: float
) -> tuple[list[int], list[float]]:
    sides = [_point_plane_side(v, plane_n, plane_d) for v in tri]
    types: list[int] = []
    t_vals: list[float] = []
    edges = [(0, 1), (1, 2), (2, 0)]
    for (i, j), si, sj in zip(edges, sides, sides[1:] + sides[:1]):
        if si == 0:
            types.append(i)
            t_vals.append(0.0)
        elif si * sj < 0:
            types.append(-1)
            d0 = abs(si)
            d1 = abs(sj)
            t = d0 / (d0 + d1) if (d0 + d1) > 1e-12 else 0.5
            t_vals.append(t)
    return (types, t_vals)


def _lerp3(a: list[float], b: list[float], t: float) -> list[float]:
    return [a[k] + (b[k] - a[k]) * t for k in range(3)]


def _tri_tri_intersect(
    ta: list[list[float]], tb: list[list[float]]
) -> tuple[list[float], list[float]] | None:
    n1, d1 = _plane(ta[0], ta[1], ta[2])
    n2, d2 = _plane(tb[0], tb[1], tb[2])

    # Parallel planes → no intersection (co-planar case ignored for simplicity)
    if abs(abs(dot_product(n1, n2)) - 1.0) < EPSILON:
        return None

    # Intersection line of the two planes
    line_dir = normalize_vector(cross_product(n1, n2))
    if vector_norm(line_dir) < EPSILON:
        return None

    # Find a point on the intersection line
    det = dot_product(n1, n1) * dot_product(n2, n2) - dot_product(n1, n2) ** 2
    if abs(det) < EPSILON:
        return None
    c1 = (d1 * dot_product(n2, n2) - d2 * dot_product(n1, n2)) / det
    c2 = (d2 * dot_product(n1, n1) - d1 * dot_product(n1, n2)) / det
    line_pt = [c1 * n1[k] + c2 * n2[k] for k in range(3)]

    # Project triangle vertices onto the line direction to get intervals
    def project_tri(tri: list[list[float]]) -> tuple[float, float]:
        pts = [dot_product(sub_vector(v, line_pt), line_dir) for v in tri]
        return (min(pts), max(pts))

    a_min, a_max = project_tri(ta)
    b_min, b_max = project_tri(tb)

    if a_max < b_min - EPSILON or b_max < a_min - EPSILON:
        return None

    # Check segments actually lie inside each triangle
    def clip_to_tri(
        tri: list[list[float]], other_n: list[float], other_d: float
    ) -> tuple[float, float] | None:
        types, t_vals = _tri_plane_intersect(tri, other_n, other_d)
        if len(types) < 2:
            return None
        pts_on_line = []
        for typ, t in zip(types, t_vals):
            if typ >= 0:
                pts_on_line.append(dot_product(sub_vector(tri[typ], line_pt), line_dir))
            else:
                ei = (typ + 1) % 3
                pts_on_line.append(
                    dot_product(
                        sub_vector(_lerp3(tri[ei], tri[(ei + 1) % 3], t), line_pt),
                        line_dir,
                    )
                )
        return (min(pts_on_line), max(pts_on_line))

    seg_a = clip_to_tri(ta, n2, d2)
    seg_b = clip_to_tri(tb, n1, d1)
    if seg_a is None or seg_b is None:
        return None

    lo = max(seg_a[0], seg_b[0])
    hi = min(seg_a[1], seg_b[1])
    if hi < lo + EPSILON:
        return None

    p1 = add_vectors(line_pt, scale_vector(line_dir, lo))
    p2 = add_vectors(line_pt, scale_vector(line_dir, hi))
    return (p1, p2)


def _split_tri_by_line(
    tri: list[list[float]], p1: list[float], p2: list[float]
) -> list[list[list[float]]]:
    """Split a triangle by a line segment. Returns up to 3 child triangles."""
    # Use barycentric coordinates to determine which side of the line each vertex is on
    line_dir = sub_vector(p2, p1)
    line_norm = normalize_vector(
        cross_product(line_dir, plane_normal(tri[0], tri[1], tri[2]))
    )
    if vector_norm(line_norm) < EPSILON:
        return [tri]

    sides = []
    for v in tri:
        d = dot_product(sub_vector(v, p1), line_norm)
        sides.append(-1 if d < 0 else (1 if d > 0 else 0))

    pos = [i for i, s in enumerate(sides) if s > 0]
    neg = [i for i, s in enumerate(sides) if s < 0]
    zero = [i for i, s in enumerate(sides) if s == 0]

    if len(zero) == 3 or len(pos) == 3 or len(neg) == 3:
        return [tri]

    tri_pos = _clip_poly_to_line(tri, p1, line_norm)
    tri_neg = _clip_poly_to_line(tri, p1, [-x for x in line_norm])
    result = []
    for poly in (tri_pos, tri_neg):
        if len(poly) >= 3:
            for k in range(1, len(poly) - 1):
                result.append([poly[0], poly[k], poly[k + 1]])
    return result if result else [tri]


def _clip_poly_to_line(
    poly: list[list[float]], line_pt: list[float], line_norm: list[float]
) -> list[list[float]]:
    """Clip a polygon to the half-space defined by line_norm · (p - line_pt) >= 0."""
    out: list[list[float]] = []
    n = len(poly)
    for i in range(n):
        curr = poly[i]
        prev = poly[(i - 1 + n) % n]
        d_curr = dot_product(sub_vector(curr, line_pt), line_norm)
        d_prev = dot_product(sub_vector(prev, line_pt), line_norm)
        curr_in = d_curr >= -EPSILON
        prev_in = d_prev >= -EPSILON
        if curr_in:
            if not prev_in:
                t = d_prev / (d_prev - d_curr) if abs(d_prev - d_curr) > 1e-12 else 0.5
                out.append(_lerp3(prev, curr, t))
            out.append(curr)
        elif prev_in:
            t = d_prev / (d_prev - d_curr) if abs(d_prev - d_curr) > 1e-12 else 0.5
            out.append(_lerp3(prev, curr, t))
    return out


def _point_in_mesh(p: list[float], mesh: Mesh) -> bool:
    """Test if a point is inside a closed mesh using ray casting."""
    # Shoot ray in +X direction, count intersections
    ray_dir = [1.0, 0.0, 0.0]
    count = 0
    for face in mesh._indices:
        v = [mesh._vertices[face[i]] for i in range(3)]
        hit = _ray_tri_intersect(p, ray_dir, v)
        if hit is not None and hit > EPSILON:
            count += 1
    # If the first attempt fails (ray passes through edges), try a different direction
    if count == 0:
        ray_dir2 = [0.0, 1.0, 0.0]
        for face in mesh._indices:
            v = [mesh._vertices[face[i]] for i in range(3)]
            hit = _ray_tri_intersect(p, ray_dir2, v)
            if hit is not None and hit > EPSILON:
                count += 1
    return count % 2 == 1


def _ray_tri_intersect(
    origin: list[float], direction: list[float], tri: list[list[float]]
) -> float | None:
    """Moeller–Trumbore ray-triangle intersection. Returns t or None."""
    edge1 = sub_vector(tri[1], tri[0])
    edge2 = sub_vector(tri[2], tri[0])
    p_vec = cross_product(direction, edge2)
    det = dot_product(edge1, p_vec)
    if abs(det) < EPSILON:
        return None
    inv_det = 1.0 / det
    t_vec = sub_vector(origin, tri[0])
    u = dot_product(t_vec, p_vec) * inv_det
    if u < 0.0 or u > 1.0:
        return None
    q_vec = cross_product(t_vec, edge1)
    v = dot_product(direction, q_vec) * inv_det
    if v < 0.0 or u + v > 1.0:
        return None
    t = dot_product(edge2, q_vec) * inv_det
    return t


# ---------------------------------------------------------------------------
#  Public CSG operations
# ---------------------------------------------------------------------------


def _centroid(tri: list[list[float]]) -> list[float]:
    return [sum(v[k] for v in tri) / 3.0 for k in range(3)]


def _reject_near_zero(verts: list[list[float]]) -> bool:
    """Reject zero-area triangles."""
    if len(verts) < 3:
        return True
    v0, v1, v2 = verts[0], verts[1], verts[2]
    e1 = sub_vector(v1, v0)
    e2 = sub_vector(v2, v0)
    return vector_norm(cross_product(e1, e2)) < 1e-12


def csg_union(mesh_a: Mesh, mesh_b: Mesh) -> Mesh:
    """Boolean union of two meshes.

    Keeps all parts of both meshes that are outside the other mesh.
    """
    return _csg_simple(mesh_a, mesh_b, "union")


def csg_difference(mesh_a: Mesh, mesh_b: Mesh) -> Mesh:
    """Subtract mesh_b from mesh_a.

    Keeps parts of A outside B and parts of B inside A (inverted).
    """
    return _csg_simple(mesh_a, mesh_b, "difference")


def csg_intersect(mesh_a: Mesh, mesh_b: Mesh) -> Mesh:
    """Boolean intersection of two meshes.

    Keeps only parts of both meshes that are inside the other mesh.
    """
    return _csg_simple(mesh_a, mesh_b, "intersect")


def _csg_simple(mesh_a: Mesh, mesh_b: Mesh, op: str) -> Mesh:
    """Simplified CSG using intersection + classification.

    This implementation:
    1. Splits intersecting triangle pairs
    2. Classifies centroids via ray casting
    3. Selects triangles based on the boolean operation
    """
    a = deepcopy(mesh_a)
    b = deepcopy(mesh_b)

    a_verts = list(a._vertices)
    b_verts = list(b._vertices)
    a_tris = [[list(a_verts[fi]) for fi in face] for face in a._indices]
    b_tris = [[list(b_verts[fi]) for fi in face] for face in b._indices]

    # Split intersecting triangles
    a_pieces = _split_all(a_tris, b_tris)
    b_pieces = _split_all(b_tris, a_tris)

    if not a_pieces:
        a_pieces = a_tris
    if not b_pieces:
        b_pieces = b_tris

    # Build test meshes
    b_test = _tri_list_to_mesh(b_tris)
    a_test = _tri_list_to_mesh(a_tris)

    result = Mesh()
    for tri in a_pieces:
        if _reject_near_zero(tri):
            continue
        inside = _point_in_mesh(_centroid(tri), b_test)
        keep = (
            (op == "union" and not inside)
            or (op == "difference" and not inside)
            or (op == "intersect" and inside)
        )
        if keep:
            result.add_triangle(vertices=tri)

    for tri in b_pieces:
        if _reject_near_zero(tri):
            continue
        inside = _point_in_mesh(_centroid(tri), a_test)
        keep = (
            (op == "union" and not inside)
            or (op == "difference" and inside)
            or (op == "intersect" and inside)
        )
        if keep:
            result.add_triangle(vertices=tri)

    result.fix_normals()
    return result


def _split_all(
    subject_tris: list[list[list[float]]],
    cutter_tris: list[list[list[float]]],
) -> list[list[list[float]]]:
    """Split all triangles in *subject_tris* by all triangles in *cutter_tris*."""
    result = []
    for s_tri in subject_tris:
        pieces = [s_tri]
        for c_tri in cutter_tris:
            inter = _tri_tri_intersect(s_tri, c_tri)
            if inter is None:
                continue
            p1, p2 = inter
            new_pieces: list[list[list[float]]] = []
            for piece in pieces:
                new_pieces.extend(_split_tri_by_line(piece, p1, p2))
            pieces = new_pieces
        result.extend(pieces)
    return result


def _tri_list_to_mesh(tris: list[list[list[float]]]) -> Mesh:
    m = Mesh()
    for tri in tris:
        m.add_triangle(vertices=tri)
    return m
