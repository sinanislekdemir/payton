# pylama:ignore=C901
# Wavefront Object File Support
# Only support ascii obj files and without material support.
# So pretty limited.
import logging
import os
from shutil import copyfile
from typing import Any, List

from payton.scene.geometry.mesh import Mesh
from payton.scene.material import DEFAULT, Material
from payton.scene.types import IList, VList


class Wavefront(Mesh):
    def __init__(self, filename: str = "", **kwargs: Any) -> None:
        super().__init__()
        self.filename: str = filename
        self.path: str = ""
        if self.filename != "":
            self.load_file(self.filename)

    def load_file(self, filename: str) -> bool:
        if not os.path.isfile(filename):
            logging.exception(f"File not found {filename}")
            return False

        self.filename = filename
        self.path = os.path.dirname(os.path.abspath(self.filename))
        f = open(filename)
        data = f.read()
        f.close()
        self.load(data)
        return True

    def load_material(self, material_string: str):
        lines = material_string.splitlines()
        material = Material()
        material_name = DEFAULT
        for line in lines:
            parts = line.split(" ")

            if parts[0] == "newmtl":
                if material_name != DEFAULT:
                    self.add_material(material_name, material)
                material = Material()
                material_name = parts[1]

            if parts[0] == "Kd":
                # Currently we only support diffuse color
                material.color = [
                    float(parts[1]),
                    float(parts[2]),
                    float(parts[3]),
                ]

            if parts[0] == "map_Kd":
                rest = " ".join(parts[1:])
                if rest == "/":
                    material.texture = rest
                else:
                    material.texture = f"{self.path}/{rest}"

        self.add_material(material_name, material)

    def load_material_file(self, filename: str) -> bool:
        if not os.path.isfile(filename):
            logging.exception(f"File not found {filename}")
            return False
        data = open(filename).read()
        return self.load_material(data)

    def load(self, obj_string: str) -> None:
        _vertices: VList = []
        _indices: List[IList] = []
        _indice_materials: List[str] = []
        _normals: VList = []
        _texcoords: VList = []
        lines: List[str] = obj_string.splitlines()
        material = DEFAULT
        for line in lines:
            line = line.replace("  ", " ")
            parts = line.split(" ")
            command = parts[0].lower()
            if parts[0] == "mtllib":
                self.load_material_file(f"{self.path}/{parts[1]}")
            if command == "v":
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _vertices.append([x, y, z])
            if command == "vt":
                u = float(parts[1])
                w = 1.0 - float(parts[2]) if len(parts) > 2 else 0
                _texcoords.append([u, w])
            if command == "vn":
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _normals.append([x, y, z])
            if command == "usemtl":
                if parts[1] in self.materials:
                    material = parts[1]

            if command == "f":
                # I guess this part of the code should be compatable
                # with POLYGON as well but IDK.
                face = []  # type: List[List[int]]
                for i in range(len(parts)):
                    if parts[i] == "f" or parts[i] == "":
                        continue
                    subs = parts[i].split("/")
                    vertex = int(subs[0]) - 1
                    textcoord = int(subs[1]) - 1 if len(subs) > 1 and len(subs[1]) > 0 else -1
                    normal = int(subs[2]) - 1 if len(subs) > 2 and len(subs[2]) > 0 else -1
                    face.append([vertex, textcoord, normal])
                if len(face) > 3:
                    logging.error("Only triangular wavefronts are accepted")
                    return

                _indices.append(face)
                _indice_materials.append(material)

        # Now unpack indices to actual object data
        i = 0
        fix_normals = False

        for _k, index in enumerate(_indices):
            ind = []
            for f in index:
                l_vertex = _vertices[f[0]]
                if f[2] != -1:
                    l_normal = _normals[f[2]]
                else:
                    fix_normals = True
                    l_normal = [0.0, 0.0, 1.0]
                l_tex = [0.0, 0.0]
                if f[1] > -1:
                    l_tex = _texcoords[f[1]]
                self._vertices.append(l_vertex)
                self._normals.append(l_normal)
                self._texcoords.append(l_tex)
                ind.append(i)
                i += 1
            self._indices.append(ind)
            self.materials[_indice_materials[_k]]._indices.append(ind)

        if fix_normals:
            self.fix_normals()


def export(mesh: Mesh, filename: str, name: str = "object"):
    if not isinstance(mesh, Mesh):
        logging.exception("Object is not an instance of Mesh")
        return None
    mat_filename = filename.replace(".obj", "") + ".mtl"

    output = [
        "# Payton Wavefront OBJ Exporter",
        f"mtllib {mat_filename}",
        f"o {name}",
    ]
    for v in mesh._vertices:
        output.append("v {}".format(" ".join([str(x) for x in v])))

    for t in mesh._texcoords:
        output.append("vt {}".format(" ".join([str(x) for x in t])))

    for n in mesh._normals:
        output.append("vn {}".format(" ".join([str(x) for x in n])))

    len_texcoords = len(mesh._texcoords) + 1
    len_normals = len(mesh._normals) + 1

    material_data = ["# Payton Wavefront OBJ Expoerter"]
    for name in mesh.materials:
        if len(mesh.materials[name]._indices) == 0:
            continue
        output.append(f"usemtl {name}")
        output.append("s off")
        material = mesh.materials[name]
        for indice in material._indices:
            f = [x + 1 for x in indice]
            t0 = str(f[0]) if len_texcoords > f[0] else ""
            n0 = str(f[0]) if len_normals > f[0] else ""

            t1 = str(f[1]) if len_texcoords > f[1] else ""
            n1 = str(f[1]) if len_normals > f[1] else ""

            t2 = str(f[2]) if len_texcoords > f[2] else ""
            n2 = str(f[2]) if len_normals > f[2] else ""
            output.append(f"f {f[0]}/{t0}/{n0} {f[1]}/{t1}/{n1} {f[2]}/{t2}/{n2}")
        material_data.append(f"newmtl {name}")
        material_data.append(f"Kd {material.color[0]} {material.color[1]} {material.color[2]}")
        if material.texture != "":
            base_name = os.path.basename(material.texture)
            path = os.path.join(os.path.abspath(os.path.dirname(filename)), base_name)
            copyfile(material.texture, path)
            material_data.append(f"map_Kd {base_name}")

    open(filename, "w").write("\n".join(output))
    open(mat_filename, "w").write("\n".join(material_data))
