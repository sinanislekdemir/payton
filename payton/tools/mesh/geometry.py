from copy import deepcopy

from payton.math.functions import (
    add_vectors,
    mid_point,
    normalize_vector,
    plane_normal,
    scale_vector,
    sub_vector,
    vector_norm,
)
from payton.scene.geometry import Line, Mesh
from payton.scene.material import DEFAULT


def _smooth_normals(mesh: Mesh, precision: int = 5) -> None:
    """Recompute smooth normals for *mesh* in-place.

    Each face's geometric normal is accumulated into every vertex that shares
    the same 3-D position (within *precision* decimal places).  The result is
    angle-weighted smooth normals with no T-junction or interpolation artefacts.

    Vertices whose accumulated normals cancel to zero (e.g. on a hard seam)
    fall back to their original flat face normal so no vertex is left dark.

    Keyword arguments:
    mesh      -- Mesh to update (modified in place)
    precision -- Decimal places used when comparing vertex positions
    """
    # Map rounded position tuple → list of vertex indices at that position
    pos_to_indices: dict[tuple, list[int]] = {}
    for idx, v in enumerate(mesh._vertices):
        key = tuple(round(c, precision) for c in v)
        pos_to_indices.setdefault(key, []).append(idx)

    # Accumulate face normals into each vertex bucket
    accumulated: list[list[float]] = [[0.0, 0.0, 0.0] for _ in mesh._vertices]
    for face in mesh._indices:
        i0, i1, i2 = face[0], face[1], face[2]
        fn = mesh._normals[i0]  # flat normal already set by add_triangle
        for fi in (i0, i1, i2):
            key = tuple(round(c, precision) for c in mesh._vertices[fi])
            for shared_idx in pos_to_indices[key]:
                accumulated[shared_idx] = add_vectors(accumulated[shared_idx], fn)

    # Normalise and write back; fall back to existing flat normal on cancellation
    for idx, acc in enumerate(accumulated):
        if vector_norm(acc) > 1e-8:
            mesh._normals[idx] = normalize_vector(acc)
        # else: keep the flat face normal already stored by add_triangle


def _material_indices(mesh: Mesh) -> dict:
    """Return per-material face index lists for *mesh*.

    If all materials have empty ``_indices`` (e.g. the user replaced the
    material object via ``mesh.material = m`` which discards the original
    indices), fall back to assigning all faces in ``mesh._indices`` to the
    DEFAULT material.
    """
    total_in_mats = sum(len(m._indices) for m in mesh.materials.values())
    if total_in_mats == 0 and mesh._indices:
        # Material was replaced and indices are only on the mesh — reassign
        result: dict[str, list] = {name: [] for name in mesh.materials}
        result[DEFAULT] = list(mesh._indices)
        return result
    return {name: list(mat._indices) for name, mat in mesh.materials.items()}


def merge_mesh(mesh1: Mesh, mesh2: Mesh) -> Mesh:
    """Merge two meshes into a single mesh.

    Materials from both meshes are preserved. mesh2 materials that share a name
    with mesh1 materials will be suffixed with '_2'.

    Keyword arguments:
    mesh1 -- First mesh
    mesh2 -- Second mesh
    """
    m1_vertices = list(mesh1.absolute_vertices())
    m2_vertices = list(mesh2.absolute_vertices())
    mesh = Mesh()
    mesh._vertices = m1_vertices + m2_vertices
    mesh._vertex_colors = mesh1._vertex_colors + mesh2._vertex_colors
    mesh._vertex_count = len(mesh._vertices)
    mesh._texcoords = mesh1._texcoords + mesh2._texcoords
    mesh._normals = mesh1._normals + mesh2._normals
    len_v = len(mesh1._vertices)

    m1_idx = _material_indices(mesh1)
    m2_idx = _material_indices(mesh2)

    # Copy mesh1 materials with their resolved indices
    for name, mat in mesh1.materials.items():
        new_mat = deepcopy(mat)
        new_mat._indices = m1_idx.get(name, [])
        mesh.materials[name] = new_mat

    # Offset and copy mesh2 indices into their respective materials
    for name, mat in mesh2.materials.items():
        m2_material_indices = [
            [i[0] + len_v, i[1] + len_v, i[2] + len_v] for i in m2_idx.get(name, [])
        ]
        target_name = name if name not in mesh.materials else f"{name}_2"
        new_mat = deepcopy(mat)
        new_mat._indices = m2_material_indices
        mesh.materials[target_name] = new_mat

    # Rebuild the flat index list from all materials
    all_indices = []
    for mat in mesh.materials.values():
        all_indices.extend(mat._indices)
    mesh._indices = all_indices

    mesh._calc_bounds()
    mesh.refresh()
    return mesh


def subdivide(original: Mesh, smooth: bool = True) -> Mesh:
    """Subdivide the mesh by splitting each triangle into 4 sub-triangles.

    Each triangle is subdivided by inserting a midpoint on every edge, yielding
    four congruent child triangles.  Because **all three** edges are always
    split, adjacent triangles will always agree on the edge midpoint — there
    are no T-junctions and the topology is consistent.

    After splitting, face normals are recomputed from the actual geometry and
    then smoothed by averaging across faces that share the same vertex position
    (see *smooth* argument).

    Keyword arguments:
    original -- Original mesh to subdivide
    smooth   -- When True (default) compute smooth normals; when False keep
                flat per-face normals
    """
    new = Mesh()
    has_texcoords = len(original._texcoords) == len(original._vertices)
    new.clear_triangles()

    for indice in original._indices:
        i0, i1, i2 = indice[0], indice[1], indice[2]

        v0 = original._vertices[i0]
        v1 = original._vertices[i1]
        v2 = original._vertices[i2]

        # Midpoints on every edge — guarantees no T-junctions
        m01 = mid_point(v0, v1)
        m12 = mid_point(v1, v2)
        m20 = mid_point(v2, v0)

        # Four child triangles (CCW winding matches parent).
        # Normals are omitted — add_triangle computes the correct flat
        # face normal automatically; _smooth_normals refines them afterwards.
        if has_texcoords:
            tc0: list[float] = list(original._texcoords[i0])
            tc1: list[float] = list(original._texcoords[i1])
            tc2: list[float] = list(original._texcoords[i2])
            tc01: list[float] = [(tc0[0] + tc1[0]) / 2.0, (tc0[1] + tc1[1]) / 2.0]
            tc12: list[float] = [(tc1[0] + tc2[0]) / 2.0, (tc1[1] + tc2[1]) / 2.0]
            tc20: list[float] = [(tc2[0] + tc0[0]) / 2.0, (tc2[1] + tc0[1]) / 2.0]
            new.add_triangle(vertices=[v0, m01, m20], texcoords=[tc0, tc01, tc20])
            new.add_triangle(vertices=[m01, v1, m12], texcoords=[tc01, tc1, tc12])
            new.add_triangle(vertices=[m20, m12, v2], texcoords=[tc20, tc12, tc2])
            new.add_triangle(vertices=[m01, m12, m20], texcoords=[tc01, tc12, tc20])
        else:
            new.add_triangle(vertices=[v0, m01, m20])
            new.add_triangle(vertices=[m01, v1, m12])
            new.add_triangle(vertices=[m20, m12, v2])
            new.add_triangle(vertices=[m01, m12, m20])

    if smooth:
        _smooth_normals(new)

    return new


def loft(profiles: list[Line]) -> Mesh:
    """Loft between multiple 3D profile lines to form a mesh.

    Profiles may have different vertex counts; additional vertices are
    interpolated automatically so that lofting works between arbitrary
    cross-sections (e.g. square to hexagon to circle).

    Keyword arguments:
    profiles -- List of Line objects defining cross-sections in order
    """
    if len(profiles) < 2:
        raise ValueError("At least two profiles are required")

    raw_verts = [list(p._vertices) for p in profiles]
    max_verts = max(len(v) for v in raw_verts)

    normalized = [_interp_profile(v, max_verts) for v in raw_verts]

    mesh = Mesh()
    for i in range(len(normalized) - 1):
        curr = normalized[i]
        nxt = normalized[i + 1]
        for j in range(max_verts - 1):
            mesh.add_triangle(
                vertices=[curr[j], curr[j + 1], nxt[j]],
                texcoords=[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
            )
            mesh.add_triangle(
                vertices=[curr[j + 1], nxt[j + 1], nxt[j]],
                texcoords=[[1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            )

    mesh.fix_normals()
    return mesh


def _interp_profile(
    vertices: list[list[float]], target_count: int
) -> list[list[float]]:
    """Interpolate a profile to *target_count* vertices by resampling evenly
    along its perimeter."""
    n = len(vertices)
    if n >= target_count:
        return [list(v) for v in vertices]

    edge_lengths = []
    total = 0.0
    for i in range(n - 1):
        d = vector_norm(sub_vector(vertices[i + 1], vertices[i]))
        edge_lengths.append(d)
        total += d

    spacing = total / (target_count - 1)

    result = [list(vertices[0])]
    accumulated = 0.0
    ei = 0
    for _ in range(target_count - 2):
        accumulated += spacing
        while ei < len(edge_lengths) and accumulated > sum(edge_lengths[: ei + 1]):
            ei += 1
        if ei >= len(edge_lengths):
            result.append(list(vertices[-1]))
            break
        dist_before = sum(edge_lengths[:ei])
        t = (accumulated - dist_before) / edge_lengths[ei]
        v0 = vertices[ei]
        v1 = vertices[ei + 1]
        pt = add_vectors(v0, scale_vector(sub_vector(v1, v0), t))
        result.append(pt)

    while len(result) < target_count:
        result.append(list(vertices[-1]))

    return result


def mirror(mesh: Mesh, axis: str = "x") -> Mesh:
    """Mirror a mesh across an axis-aligned plane and merge with the original.

    The mirrored copy is merged into the original mesh, producing a
    symmetrical result.  Triangle winding is reversed on the mirrored side
    so normals face outward.

    Keyword arguments:
    mesh -- Mesh to mirror
    axis -- Axis to mirror across ('x', 'y', or 'z')
    """
    axis_idx = {"x": 0, "y": 1, "z": 2}.get(axis, 0)
    copy = deepcopy(mesh)

    for v in copy._vertices:
        v[axis_idx] = -v[axis_idx]
    for n in copy._normals:
        n[axis_idx] = -n[axis_idx]
    for face in copy._indices:
        face[0], face[2] = face[2], face[0]

    return _merge_two(mesh, copy)


def _merge_two(mesh1: Mesh, mesh2: Mesh) -> Mesh:
    """Minimal merge of two meshes (used internally by mirror)."""
    return merge_mesh(mesh1, mesh2)


def extrude_face(
    mesh: Mesh,
    face_indices: list[int],
    distance: float,
    taper: float = 1.0,
    segments: int = 1,
) -> Mesh:
    """Extrude selected faces of a mesh along their normals.

    Each face in *face_indices* is offset outward, and side faces are
    created to connect the original perimeter to the extruded face.

    Keyword arguments:
    mesh -- Source mesh
    face_indices -- Indices of faces (triangles) in ``mesh._indices`` to extrude
    distance -- Extrusion distance
    taper -- Scale factor for the extruded face (1.0 = no taper, <1 = smaller)
    segments -- Number of extrusion segments (for stepped extrusions)
    """
    result = deepcopy(mesh)
    seg_dist = distance / segments

    for _ in range(segments):
        curr_extrude = sorted(set(face_indices))
        new_face_indices = []

        # Process in reverse index order so removals don't shift later indices
        for face_idx in reversed(curr_extrude):
            if face_idx >= len(result._indices):
                continue

            face = list(result._indices[face_idx])
            verts = [list(result._vertices[i]) for i in face]
            normal = plane_normal(verts[0], verts[1], verts[2])

            center = [
                (verts[0][0] + verts[1][0] + verts[2][0]) / 3.0,
                (verts[0][1] + verts[1][1] + verts[2][1]) / 3.0,
                (verts[0][2] + verts[1][2] + verts[2][2]) / 3.0,
            ]

            new_verts = []
            for i, v in enumerate(verts):
                dir_to_center = sub_vector(center, v)
                offset = add_vectors(
                    scale_vector(normal, seg_dist),
                    scale_vector(dir_to_center, 1.0 - taper),
                )
                new_verts.append(add_vectors(v, offset))

            # Remove original face from index lists
            del result._indices[face_idx]
            for mat in result.materials.values():
                if face_idx < len(mat._indices):
                    del mat._indices[face_idx]

            # Extruded cap face (replace original face position with new verts)
            cap_idx = len(result._vertices)
            for v in new_verts:
                result._vertices.append(v)
            for _ in range(3):
                result._normals.append(list(normal))
                result._texcoords.append([0.0, 0.0])
            result._indices.insert(face_idx, [cap_idx + 2, cap_idx + 1, cap_idx])
            for mat in result.materials.values():
                mat._indices.insert(face_idx, [cap_idx + 2, cap_idx + 1, cap_idx])
            new_face_indices.append(face_idx)

            # Side triangles (reversed winding)
            for i in range(3):
                j = (i + 1) % 3
                oi, oj = face[i], face[j]
                ni, nj = cap_idx + i, cap_idx + j

                result._indices.append([ni, oj, oi])
                result._indices.append([ni, nj, oj])
                for mat in result.materials.values():
                    mat._indices.append([ni, oj, oi])
                    mat._indices.append([ni, nj, oj])

        face_indices = new_face_indices

    result.fix_normals()
    result.refresh()
    return result


def laplacian_smooth(mesh: Mesh, iterations: int = 1, factor: float = 0.5) -> Mesh:
    """Smooth a mesh using Laplacian smoothing.

    Each vertex moves toward the centroid of its neighbours.  Normals are
    recomputed after smoothing.

    Keyword arguments:
    mesh -- Mesh to smooth
    iterations -- Number of smoothing passes (default 1)
    factor -- Smoothing strength (0.0 = no change, 1.0 = full centroid)
    """
    result = deepcopy(mesh)

    adj: dict[int, list[int]] = {i: [] for i in range(len(result._vertices))}
    for face in result._indices:
        for i in range(3):
            a, b = face[i], face[(i + 1) % 3]
            if b not in adj[a]:
                adj[a].append(b)
            if a not in adj[b]:
                adj[b].append(a)

    for _ in range(iterations):
        new_verts = [list(v) for v in result._vertices]
        for i in range(len(result._vertices)):
            neighbours = adj[i]
            if not neighbours:
                continue
            centroid = [0.0, 0.0, 0.0]
            for nb in neighbours:
                centroid = add_vectors(centroid, result._vertices[nb])
            centroid = scale_vector(centroid, 1.0 / len(neighbours))
            diff = sub_vector(centroid, result._vertices[i])
            new_verts[i] = add_vectors(result._vertices[i], scale_vector(diff, factor))
        result._vertices = new_verts

    result.fix_normals()
    result.refresh()
    return result


def decimate(mesh: Mesh, ratio: float = 0.5) -> Mesh:
    """Reduce the polygon count of a mesh using vertex clustering.

    Vertices are grouped into a 3D grid; each group is replaced by its
    centroid.  Degenerate (zero-area) triangles are discarded.

    Keyword arguments:
    mesh -- Mesh to simplify
    ratio -- Target vertex ratio (0.5 = halve the vertex count)
    """
    verts = mesh._vertices
    n_verts = len(verts)
    target = max(3, int(n_verts * ratio))
    grid_size = max(2, round((n_verts / target) ** (1.0 / 3.0) * 5))

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]
    vmin = [min(xs), min(ys), min(zs)]
    vmax = [max(xs), max(ys), max(zs)]
    size = [max(vmax[i] - vmin[i], 1e-10) for i in range(3)]
    cell_size = [size[i] / grid_size for i in range(3)]

    cell_verts: dict[tuple[int, int, int], list[list[float]]] = {}
    cell_norms: dict[tuple[int, int, int], list[list[float]]] = {}
    cell_uvs: dict[tuple[int, int, int], list[list[float]]] = {}

    for i, v in enumerate(verts):
        cx = min(grid_size - 1, max(0, int((v[0] - vmin[0]) / cell_size[0])))
        cy = min(grid_size - 1, max(0, int((v[1] - vmin[1]) / cell_size[1])))
        cz = min(grid_size - 1, max(0, int((v[2] - vmin[2]) / cell_size[2])))
        key = (cx, cy, cz)
        cell_verts.setdefault(key, []).append(v)
        if i < len(mesh._normals):
            cell_norms.setdefault(key, []).append(mesh._normals[i])
        if i < len(mesh._texcoords):
            cell_uvs.setdefault(key, []).append(mesh._texcoords[i])

    avg_verts: dict[tuple[int, int, int], list[float]] = {}
    avg_norms: dict[tuple[int, int, int], list[float]] = {}
    avg_uvs: dict[tuple[int, int, int], list[float]] = {}
    for key, vlist in cell_verts.items():
        avg_verts[key] = scale_vector([sum(c) for c in zip(*vlist)], 1.0 / len(vlist))
        if key in cell_norms:
            nsum = [sum(c) for c in zip(*cell_norms[key])]
            nn = normalize_vector(nsum)
            avg_norms[key] = nn if nn else [0.0, 0.0, 1.0]
        if key in cell_uvs:
            uv_sum = [sum(c) for c in zip(*cell_uvs[key])]
            avg_uvs[key] = [x / len(cell_uvs[key]) for x in uv_sum]

    cell_to_idx: dict[tuple[int, int, int], int] = {}
    new_verts: list[list[float]] = []
    new_norms: list[list[float]] = []
    new_uvs: list[list[float]] = []
    for key, v in avg_verts.items():
        cell_to_idx[key] = len(new_verts)
        new_verts.append(v)
        new_norms.append(avg_norms.get(key, [0.0, 0.0, 1.0]))
        new_uvs.append(avg_uvs.get(key, [0.0, 0.0]))

    result = Mesh()
    result._vertices = list(new_verts)
    result._normals = list(new_norms)
    result._texcoords = list(new_uvs)

    for face in mesh._indices:
        cx = min(
            grid_size - 1, max(0, int((verts[face[0]][0] - vmin[0]) / cell_size[0]))
        )
        cy = min(
            grid_size - 1, max(0, int((verts[face[0]][1] - vmin[1]) / cell_size[1]))
        )
        cz = min(
            grid_size - 1, max(0, int((verts[face[0]][2] - vmin[2]) / cell_size[2]))
        )
        k0 = cell_to_idx[(cx, cy, cz)]
        cx = min(
            grid_size - 1, max(0, int((verts[face[1]][0] - vmin[0]) / cell_size[0]))
        )
        cy = min(
            grid_size - 1, max(0, int((verts[face[1]][1] - vmin[1]) / cell_size[1]))
        )
        cz = min(
            grid_size - 1, max(0, int((verts[face[1]][2] - vmin[2]) / cell_size[2]))
        )
        k1 = cell_to_idx[(cx, cy, cz)]
        cx = min(
            grid_size - 1, max(0, int((verts[face[2]][0] - vmin[0]) / cell_size[0]))
        )
        cy = min(
            grid_size - 1, max(0, int((verts[face[2]][1] - vmin[1]) / cell_size[1]))
        )
        cz = min(
            grid_size - 1, max(0, int((verts[face[2]][2] - vmin[2]) / cell_size[2]))
        )
        k2 = cell_to_idx[(cx, cy, cz)]

        if k0 == k1 or k1 == k2 or k0 == k2:
            continue

        result._indices.append([k2, k1, k0])
        result.materials[DEFAULT]._indices.append([k2, k1, k0])

    result.fix_normals()
    result.refresh()
    return result
