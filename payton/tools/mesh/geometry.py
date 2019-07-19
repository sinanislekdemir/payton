from copy import deepcopy

from payton.scene.geometry import Mesh


def merge_mesh(mesh1: Mesh, mesh2: Mesh) -> Mesh:
    """Merge two mesh into a single mesh keeping their positions.

    Note: This will not automatically merge children of meshes.

    Args:
      mesh1: First mesh
      mesh2: Second mesh

    Returns:
      Mesh
    """
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
    m2_indices = [
        [i[0] + len_v, i[1] + len_v, i[2] + len_v] for i in mesh2._indices
    ]
    mesh._indices = mesh1._indices + m2_indices
    mesh._calc_bounds()
    mesh.refresh()
    return mesh
