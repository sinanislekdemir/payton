"""Payton Line to Mesh module"""
from typing import List

from payton.math.matrix import create_rotation_matrix_raw
from payton.math.vector import add_vectors, scale_vector, vector_transform
from payton.scene.geometry import Line, Mesh


def extrude_line(line: Line, direction: List[float], distance: float) -> Mesh:
    vertices = line._vertices
    diff_vector = scale_vector(direction, distance)
    mirror_vertices = [add_vectors(v, diff_vector) for v in vertices]

    mesh = Mesh()
    for i in range(len(vertices) - 1):
        mesh.add_triangle(
            vertices=[vertices[i], vertices[i + 1], mirror_vertices[i]], texcoords=[[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]],
        )
        mesh.add_triangle(
            vertices=[vertices[i + 1], mirror_vertices[i + 1], mirror_vertices[i]],
            texcoords=[[1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
        )
    return mesh


def rotate_line(line: Line, axis: List[float], angle: float, steps: int = 10) -> Mesh:
    step_angle = angle / steps
    vertices = line._vertices
    step_u = 1.0 / steps
    step_v = 1.0 / len(vertices)
    mesh = Mesh()

    for i in range(steps):
        matrix = create_rotation_matrix_raw(axis, step_angle)
        mirror_vertices = [vector_transform(v, matrix) for v in vertices]
        for j in range(len(vertices) - 1):
            mesh.add_triangle(
                vertices=[vertices[j], vertices[j + 1], mirror_vertices[j]],
                texcoords=[[step_u * i, step_v * j], [step_u * (i + 1), step_v * j], [step_u * i, step_v * (j + 1)]],
            )
            mesh.add_triangle(
                vertices=[vertices[j + 1], mirror_vertices[j + 1], mirror_vertices[j]],
                texcoords=[
                    [step_u * (i + 1), step_v * j],
                    [step_u * (i + 1), step_v * (j + 1)],
                    [step_u * i, step_v * (j + 1)],
                ],
            )
        vertices = mirror_vertices.copy()
    mesh.fix_normals()
    return mesh


def lines_to_mesh(lines: List[Line]) -> Mesh:
    lens = [len(l._vertices) for l in lines]
    lmin = min(lens)
    lmax = max(lens)
    mesh = Mesh()
    if lmin != lmax:
        raise BaseException("Number of vertices for each line must be equal")
    if len(lines) == 0:
        raise BaseException("You must specify more than one line object")
    for i in range(len(lines) - 1):
        fvlist = lines[i]._vertices
        tvlist = lines[i + 1]._vertices
        for j in range(lmin - 1):
            mesh.add_triangle(vertices=[fvlist[j], fvlist[j + 1], tvlist[j]])
            mesh.add_triangle(vertices=[fvlist[j + 1], tvlist[j + 1], tvlist[j]])
    mesh.fix_normals()
    mesh.fix_texcoords()
    return mesh
