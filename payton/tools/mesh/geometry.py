from copy import deepcopy
from typing import Dict, List, Tuple

from payton.math.functions import add_vectors, mid_point, normalize_vector, vector_norm
from payton.scene.geometry import Mesh
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
    pos_to_indices: Dict[Tuple, List[int]] = {}
    for idx, v in enumerate(mesh._vertices):
        key = tuple(round(c, precision) for c in v)
        pos_to_indices.setdefault(key, []).append(idx)

    # Accumulate face normals into each vertex bucket
    accumulated: List[List[float]] = [[0.0, 0.0, 0.0] for _ in mesh._vertices]
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
        result: Dict[str, List] = {name: [] for name in mesh.materials}
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
            tc0: List[float] = list(original._texcoords[i0])
            tc1: List[float] = list(original._texcoords[i1])
            tc2: List[float] = list(original._texcoords[i2])
            tc01: List[float] = [(tc0[0] + tc1[0]) / 2.0, (tc0[1] + tc1[1]) / 2.0]
            tc12: List[float] = [(tc1[0] + tc2[0]) / 2.0, (tc1[1] + tc2[1]) / 2.0]
            tc20: List[float] = [(tc2[0] + tc0[0]) / 2.0, (tc2[1] + tc0[1]) / 2.0]
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
