# Wavefront Object File Support
# Only support ascii obj files and without material support.
# So pretty limited.

import logging
import os

from typing import Any, List, Optional
from payton.scene.types import VList, IList
from payton.scene.geometry import Mesh


class Wavefront(Mesh):
    """
    Wavefront object file class.
    Only supports ascii obj files in a limited way.
    So do not depend so much on this class.
    Only designed to accept your triangular geometries.
    """

    def __init__(self, **args: Any) -> None:
        """
        Initialize Wavefront Object.
        """
        super().__init__()
        self.filename: str = args.get("filename", "")
        if self.filename != "":
            self.load_file(self.filename)

    def load_file(self, filename: str) -> bool:
        """
        Load obj file.
        """
        if not os.path.isfile(filename):
            logging.exception(f"File not found {filename}")
            return False

        self.filename = filename
        f = open(filename)
        data = f.read()
        f.close()
        self.load(data)
        return True

    def load(self, obj_string: str) -> None:
        """
        A bit of information on file format,
        v -> x, y, z, (w)
        vt -> u, [v, (w)]
        vn -> x, y, z
        f -> vertex_index/texcoord_index/normal_index ...

        Also there are definitions of material and line and object name
        but for now, the assumption is there will always be triangulated
        wavefront object files and always a single object at a time.
        """
        _vertices: VList = []
        _indices: List[IList] = []
        _normals: VList = []
        _texcoords: VList = []
        lines: List[str] = obj_string.splitlines()
        for line in lines:
            line = line.replace("  ", " ")
            command = line[0:2].lower()
            parts = line.split(" ")
            if command == "v ":
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _vertices.append([x, y, z])
            if command == "vt":
                u = float(parts[1])
                w = float(parts[2]) if len(parts) > 2 else 0
                _texcoords.append([u, w])
            if command == "vn":
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _normals.append([x, y, z])
            if command == "f ":
                # I guess this part of the code should be compatable
                # with POLYGON as well but IDK.
                face = []  # type: List[List[int]]
                for i in range(len(parts)):
                    if parts[i] == "f" or parts[i] == "":
                        continue
                    subs = parts[i].split("/")
                    vertex = int(subs[0]) - 1
                    textcoord = (
                        int(subs[1]) - 1
                        if len(subs) > 1 and len(subs[1]) > 0
                        else -1
                    )
                    normal = (
                        int(subs[2]) - 1
                        if len(subs) > 2 and len(subs[2]) > 0
                        else -1
                    )
                    face.append([vertex, textcoord, normal])
                if len(face) > 3:
                    logging.error("Only triangular wavefronts are accepted")
                    return
                _indices.append(face)

        # Now unpack indices to actual object data
        i = 0
        fix_normals = False
        for index in _indices:
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
        if fix_normals:
            self.fix_normals()


def export(mesh: Mesh, **args: Any) -> Optional[str]:
    """Export mesh as wavefront object string

    @TODO Add material export support.

    Basic usage:

        from payton.scene.geometry import Cube
        from payton.scene.wavefront import export

        cube = Cube()
        f = open('cube.obj', 'w')
        f.write(export(cube, name='Cube'))
        f.close()


    Args:
      mesh: An instance of `payton.scene.geometry.Mesh`
      name (optional): Name of the object, otherwise `object` will be used
    """
    if not isinstance(mesh, Mesh):
        logging.exception("Object is not an instance of Mesh")
        return None

    name = args.get("name", "object")
    output = ["# Payton Wavefront OBJ Exporter", f"o {name}"]
    for v in mesh._vertices:
        output.append("v {}".format(" ".join([str(x) for x in v])))

    for t in mesh._texcoords:
        output.append("vt {}".format(" ".join([str(x) for x in t])))

    for n in mesh._normals:
        output.append("vn {}".format(" ".join([str(x) for x in n])))

    len_texcoords = len(mesh._texcoords) + 1
    len_normals = len(mesh._normals) + 1
    for f in mesh._indices:
        f = [x + 1 for x in f]
        t0 = str(f[0]) if len_texcoords > f[0] else ""
        n0 = str(f[0]) if len_normals > f[0] else ""

        t1 = str(f[1]) if len_texcoords > f[1] else ""
        n1 = str(f[1]) if len_normals > f[1] else ""

        t2 = str(f[2]) if len_texcoords > f[2] else ""
        n2 = str(f[2]) if len_normals > f[2] else ""
        output.append(f"f {f[0]}/{t0}/{n0} {f[1]}/{t1}/{n1} {f[2]}/{t2}/{n2}")
    return "\n".join(output)
