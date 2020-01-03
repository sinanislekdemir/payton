from copy import deepcopy

from payton.math.vector import distance, mid_point
from payton.scene.geometry import Mesh


def merge_mesh(mesh1: Mesh, mesh2: Mesh) -> Mesh:
    m1_vertices = list(mesh1.absolute_vertices())
    m2_vertices = list(mesh2.absolute_vertices())
    mesh = Mesh()
    mesh.material = deepcopy(mesh1.material)
    mesh._vertices = m1_vertices + m2_vertices
    mesh._vertex_colors = mesh1._vertex_colors + mesh2._vertex_colors
    mesh._vertex_count = len(mesh._vertices)
    mesh._texcoords = mesh1._texcoords + mesh2._texcoords
    mesh._normals = mesh1._normals + mesh2._normals
    len_v = len(mesh1._vertices)
    m2_indices = [[i[0] + len_v, i[1] + len_v, i[2] + len_v] for i in mesh2._indices]
    mesh._indices = mesh1._indices + m2_indices
    mesh.material._indices = mesh._indices
    mesh._calc_bounds()
    mesh.refresh()
    return mesh


def subdivide(mesh: Mesh, rounds: int = 1) -> Mesh:
    original = deepcopy(mesh)
    new = deepcopy(original)
    for i in range(rounds):
        has_texcoords = len(original._texcoords) == len(original._vertices)
        new.clear_triangles()

        for indice in original._indices:
            v1 = original._vertices[indice[0]]
            v2 = original._vertices[indice[1]]
            v3 = original._vertices[indice[2]]

            a = v1
            a_i = indice[0]
            b = v2
            b_i = indice[1]
            c = v3
            c_i = indice[2]

            if distance(v2, v3) > distance(v1, v2):
                a = v2
                a_i = indice[1]
                b = v3
                b_i = indice[2]
                c = v1
                c_i = indice[0]

            if distance(v3, v1) > distance(v2, v3):
                a = v3
                a_i = indice[2]
                b = v1
                b_i = indice[0]
                c = v2
                c_i = indice[1]

            vc = mid_point(a, b)
            n1 = original._normals[indice[0]]
            texcoords_1 = None
            texcoords_2 = None
            if has_texcoords:
                t1 = original._texcoords[a_i]
                t2 = original._texcoords[b_i]
                t3 = original._texcoords[c_i]

                tc = mid_point(t1, t2)

                texcoords_1 = [t1, tc, t3]
                texcoords_2 = [t3, tc, t2]

            new.add_triangle(
                vertices=[a, vc, c], normals=[n1, n1, n1], texcoords=texcoords_1,
            )
            new.add_triangle(
                vertices=[c, vc, b], normals=[n1, n1, n1], texcoords=texcoords_2,
            )
        original = deepcopy(new)
    return new
