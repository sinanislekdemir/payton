# Wavefront Object File Support
# Only support ascii obj files and without material support.
# So pretty limited.

import logging
import os
from payton.scene import Object

class Wavefront(Object):
    """
    Wavefront object file class.
    Only supports ascii obj files in a limited way.
    So do not depend so much on this class.
    Only designed to accept your designs.
    """
    def __init__(self, filename=''):
        """
        Initialize Wavefront Object.
        """
        super().__init__()
        self.filename = filename
        if filename != '':
            self.load_file(filename)
    
    def load_file(self, filename):
        """
        Load obj file.
        """
        if not os.path.isfile(filename):
            logging.exception("File not found {}".format(filename))
            return False

        self.filename = filename
        f = open(filename)
        data = f.read()
        f.close()
        self.load(data)
    
    def load(self, obj_string):
        """
        This part of the code is a bit messy. Ideally, a file format
        parsing shouldn't be here. But on the other hand, this is still
        a proof of concept so technical debts can be taken.
        @TODO: Create an importer mechanism instead of directly parsing
               Wavefront Object file format inside Object class.
        A 3D object can be imported from many different file formats
        we can not restrict it to a single format and embed it to Object.
        That is an anti-pattern. Ok. I know.

        A bit of information on file format,
        v -> x, y, z, (w)
        vt -> u, [v, (w)]
        vn -> x, y, z
        f -> vertex_index/texcoord_index/normal_index ...

        Also there are definitions of material and line and object name
        but for now, the assumption is there will always be triangulated
        wavefront object files and always a single object at a time.
        """
        _vertices = []
        _indices = []
        _normals = []
        _texcoords = []
        lines = obj_string.splitlines()
        for line in lines:
            command = line[0:2].lower()
            parts = line.split(' ')
            if command == 'v ':
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _vertices.append([x, y, z])
            if command == 'vt':
                u = float(parts[1])
                w = float(parts[2]) if len(parts) > 2 else 0
                _texcoords.append([u, w])
            if command == 'vn':
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                _normals.append([x, y, z])
            if command == 'f ':
                # I guess this part of the code should be compatable
                # with POLYGON as well but IDK.
                face = []
                for i in range(len(parts)):
                    if i == 0:
                        continue
                    subs = parts[i].split('/')
                    vertex = int(subs[0]) - 1
                    textcoord = int(subs[1]) - 1 if len(subs) > 1 else -1
                    normal = int(subs[2]) - 1 if len(subs) > 2 else -1
                    face.append((vertex, textcoord, normal))
                _indices.append(face)

        # Now unpack indices to actual object data
        i = 0
        for index in _indices:
            for f in index:
                vertex = _vertices[f[0]]
                normal = _normals[f[2]]
                tex = [0, 0]
                if f[1] > -1:
                    tex = _texcoords[f[1]]
                self._vertices.extend(vertex)
                self._normals.extend(normal)
                self._texcoords.extend(tex)
                self._indices.append(i)
                i += 1

        logging.debug("""Loaded {} vertices, {} textcoords, {} normals, {} faces""".format(
            len(self._vertices), len(self._texcoords), len(self._normals), len(self._indices)))
