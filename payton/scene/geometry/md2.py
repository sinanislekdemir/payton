"""
ID Software Quake 2 Model File
Some parts of this file is based on the original work from:
https://github.com/adamlwgriffiths/PyMesh/tree/master/pymesh/md2
"""
import logging
import os
import struct
import time
from copy import deepcopy
from typing import Any, BinaryIO, Dict, List, NamedTuple, Optional, cast

import numpy as np  # type: ignore

from payton.scene.geometry.mesh import Mesh
from payton.scene.material import POINTS
from payton.scene.shader import DEFAULT_SHADER, PARTICLE_SHADER, Shader

_SIGNATURE = "IDP2"
_VERSION = 8


class MeshException(Exception):
    pass


def _interpolate(mesh_1: Mesh, mesh_2: Mesh, steps: int = 1) -> List[Mesh]:
    """Interpolate two alike meshes

    This is suitable to fill the blank frames of an animated object
    This function makes the assumption that same indices will be forming
    the same triangle.

    This function returns at least 3 meshes;
    [mesh_1, interpolated mesh, mesh_2]
    """
    if len(mesh_1._vertices) != len(mesh_2._vertices):
        raise MeshException("Mesh 1 and Mesh 2 vertex counts do not match")
    if len(mesh_1.children) > 0:
        raise MeshException("Mesh 1 has children")
    if len(mesh_2.children) > 0:
        raise MeshException("Mesh 2 has children")

    # Generate meshes
    results: List[Mesh] = [mesh_1]

    # Calculate vertex distances;
    distances = []
    for v_i, vertex in enumerate(mesh_1._vertices):
        dist = [0.0, 0.0, 0.0]
        dist[0] = (mesh_2._vertices[v_i][0] - vertex[0]) / (steps + 1)
        dist[1] = (mesh_2._vertices[v_i][1] - vertex[1]) / (steps + 1)
        dist[2] = (mesh_2._vertices[v_i][2] - vertex[2]) / (steps + 1)
        distances.append(dist)

    for i in range(1, steps + 1):
        mesh = Mesh()
        mesh.materials = deepcopy(results[i - 1].materials)
        mesh._texcoords = deepcopy(results[i - 1]._texcoords)
        mesh._indices = deepcopy(results[i - 1]._indices)
        mesh.destroy()  # Destroy references to previous objects opengl
        for v_i, vertex in enumerate(results[i - 1]._vertices):
            mesh._vertices.append(
                [vertex[0] + distances[v_i][0], vertex[1] + distances[v_i][1], vertex[2] + distances[v_i][2]]
            )
        mesh.fix_normals()
        results.append(mesh)

    results.append(mesh_2)

    return results


class MD2Header(NamedTuple):
    ident: bytes = b""
    version: int = 0
    skin_width: int = 0
    skin_height: int = 0
    frame_size: int = 0
    num_skins: int = 0
    num_vertices: int = 0
    num_st: int = 0
    num_tris: int = 0
    num_glcmds: int = 0
    num_frames: int = 0
    offset_skins: int = 0
    offset_st: int = 0
    offset_tris: int = 0
    offset_frames: int = 0
    offset_glcmds: int = 0
    offset_end: int = 0


class MD2Frame(NamedTuple):
    name: str = ""
    vertices: np.ndarray = []
    normals: np.ndarray = []


class MD2TriangleLayout(NamedTuple):
    vertex_indices: np.ndarray = []
    tc_indices: np.ndarray = []


def read_block(b: BinaryIO, format_str: str, count: int):
    def chunks(data: bytes, size):
        offset = 0
        while offset < len(data):
            yield_size = offset + size
            yield data[offset:yield_size]
            offset += size

    struct_length = struct.calcsize(format_str)
    total_length = struct_length * count
    data = b.read(total_length)
    if len(data) < total_length:
        raise ValueError("MD2: Failed to read '%d' bytes" % (total_length))
    return [struct.unpack(format_str, chunk) for chunk in chunks(data, struct_length)]


def fix_skin_name(name: bytes) -> str:
    name_str = name.decode("utf-8").replace("\x00", "")
    name_parts = os.path.splitext(name_str)
    if name_parts[1].startswith(".pcx"):
        return f"{name_parts[0]}.pcx"
    return name_str


class MD2(Mesh):
    """
    MD2 File Format Loader
    I know that this is a pretty old file format. We can consider it as ancient
    On the other hand, our target on creating Payton is not to make a game
    engine.

    Also note that there might be some bugs, I give no guarantee here.
    As this is a generic class, I haven't defined any animations before hand.
    So to make things properly looping in your code, you need to set the
    animation frame ranges yourself. For instance, as you will see at the
    defined example, infantry has a perfect walking loop between frames 2-13

    Unlike original frame names, frames in this class starts from 0 and goes
    incremented by one.

    NOTE: There are some hard-coded pre-assumptions like the scale of the
    objects and they are just defined to look okay on a default scene.

    Example use case:

        .. include:: ../../../examples/basics/26_quake2.py
    """

    def __init__(self, filename: str = "", texture_filename: str = "", **kwargs: Any):
        """Initialize the MD2 Object

        Args:
          filename: Filename to load
        """
        super().__init__(**kwargs)
        self.header: MD2Header = MD2Header()
        self.triangle_layout = MD2TriangleLayout()
        self.skins: List[str] = []
        self.texture_filename = texture_filename
        self.frames: List[MD2Frame] = []
        self._frame_children: Dict[str, Mesh] = {}
        self.animation: str = ""
        self.animations: Dict[str, List] = {}

        self._active_frame: int = 0
        self._frame_rate: float = 0
        self._from_frame: int = 0
        self._to_frame: int = 0
        self._time: float = 0
        self._path: str = ""
        self._loop: bool = False

        if os.path.exists(filename):
            self.load_file(filename)

    def add_frame_child(self, name: str, mesh: Mesh) -> None:
        self._frame_children[name] = mesh

    def bake_animation(self, animation_name: str, from_frame: int, to_frame: int, steps: int = 1,) -> None:
        """Original MD2 Format does not include all frames in an animation.

        Instead, there are "key frames". There are gaps between the key-frames
        so you need to fill-in the empty frames by calculating the motion
        between key-frames. Bake Animation method simply fills these gaps by
        calculating the N steps between key-frames. This gives you a smooth
        animation.
        """
        if animation_name not in self.animations:
            logging.error(f"Animation {animation_name} not found in object")
            return
        anim = self.animations[animation_name]
        if from_frame < anim[0] or from_frame > anim[1]:
            logging.error(f"from_frame out of bounds")
            return
        if to_frame < anim[0] or to_frame > anim[1]:
            logging.error(f"to_frame out of bounds")
            return

        frames = []
        for i in range(from_frame, to_frame):
            frame_name = f"{animation_name}{i}"
            frame = self._frame_children[frame_name]
            nf = i + 1
            next_name = f"{animation_name}{nf}"
            next_frame = self._frame_children[next_name]

            interpolated_frames = _interpolate(frame, next_frame, steps)
            frames.extend(interpolated_frames[0:-1])
        last_frame = self._frame_children[f"{animation_name}{to_frame}"]
        first_frame = self._frame_children[f"{animation_name}{from_frame}"]
        join_frames = _interpolate(last_frame, first_frame, steps)
        frames.extend(join_frames)

        for i in range(len(frames)):
            frame_name = f"{animation_name}{i}"
            self._frame_children[frame_name] = frames[i]
        self.animations[animation_name] = [0, len(frames) - 1]

    def animate(
        self, animation_name: str, from_frame: int, to_frame: int, loop: bool = True,
    ):
        """Set the model in motion

        Args:
          animation_name: Name of the animation to play
          from_frame: Loop starting frame
          to_frame: Loop ending frame
          loop: Loop the animation?
        """
        self.animation = ""  # For thread safety
        if animation_name not in self.animations:
            logging.error(f"Animation {animation_name} not found in object")
            return
        anim = self.animations[animation_name]
        if from_frame < anim[0] or from_frame > anim[1]:
            logging.error(f"from_frame out of bounds")
            return
        if to_frame < anim[0] or to_frame > anim[1]:
            logging.error(f"to_frame out of bounds")
            return
        self._loop = loop
        self._active_frame = from_frame
        self._from_frame = from_frame
        self._to_frame = to_frame
        self.animation = animation_name
        self._frame_rate = 0
        for child in self.children:
            child_obj = cast(MD2, self.children[child])
            child_obj.animate(animation_name, from_frame, to_frame, loop)

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ) -> None:
        if not self._visible:
            return

        self.update_matrix(parent_matrix=parent_matrix)
        self.track()

        # Render motion path
        if self.track_motion:
            self._motion_path_line.render(lit, shader, parent_matrix)

        if self.animation == "":
            for child in self._frame_children:
                self._frame_children[child].render(lit, shader, self._model_matrix)
                break
                # render children
            for child in self.children:
                self.children[child].render(lit, shader, self._model_matrix)
            return

        if self._time == 0:
            self._time = time.time()

        tdiff = time.time() - self._time
        if self._frame_rate == 0:
            self._frame_rate = 1 / (self._to_frame - self._from_frame)

        if tdiff >= self._frame_rate:
            self._active_frame += 1
            if self._active_frame > self._to_frame:
                if self._loop:
                    self._active_frame = self._from_frame
                else:
                    self._active_frame -= 1

            self._time = time.time()

        frame_name = f"{self.animation}{self._active_frame}"

        self._frame_children[frame_name].render(lit, shader, self._model_matrix)
        for child in self.children:
            self.children[child].render(lit, shader, self._model_matrix)

    def load_file(self, filename: str):
        if not os.path.exists(filename):
            raise BaseException(f"File not found: {filename}")
        self._path = os.path.dirname(os.path.abspath(filename))
        with open(filename, "rb") as f:
            self.load_buffer(f)

    def load_buffer(self, f: BinaryIO):
        self.read_header(f)
        self.read_skin(f)
        self.read_tex_coords(f)
        self.read_triangles(f)
        self.load_frames(f)
        self.compile()

    def read_triangles(self, f: BinaryIO):
        f.seek(self.header.offset_tris, os.SEEK_SET)
        triangles = np.array(read_block(f, "< 6H", self.header.num_tris), dtype=np.uint16)
        triangles.shape = (-1, 6)
        vertex_indices = triangles[:, :3]
        tc_indices = triangles[:, 3:]
        self.triangle_layout = MD2TriangleLayout(vertex_indices=vertex_indices, tc_indices=tc_indices)

    def compile(self):
        self.animations = {}
        for frame in self.frames:
            name = "".join(i for i in frame.name if not i.isdigit())
            if name not in self.animations:
                self.animations[name] = [0, -1]
            self.animations[name][1] += 1
            num = self.animations[name][1]
            frame_name = f"{name}{num}"
            self.build_frame(frame, frame_name)

    def build_frame(self, frame_information: MD2Frame, name: str):
        mesh = Mesh()
        for i, tri in enumerate(self.triangle_layout.vertex_indices):
            v3 = frame_information.vertices[tri[1]].tolist()
            v2 = frame_information.vertices[tri[0]].tolist()
            v1 = frame_information.vertices[tri[2]].tolist()
            v1[0], v1[1] = v1[1], v1[0]
            v2[0], v2[1] = v2[1], v2[0]
            v3[0], v3[1] = v3[1], v3[0]

            t3 = cast(np.ndarray, self._texcoords[self.triangle_layout.tc_indices[i][1]]).tolist()
            t2 = cast(np.ndarray, self._texcoords[self.triangle_layout.tc_indices[i][0]]).tolist()
            t1 = cast(np.ndarray, self._texcoords[self.triangle_layout.tc_indices[i][2]]).tolist()

            mesh.add_triangle(
                vertices=[v1, v2, v3], texcoords=[t1, t2, t3],
            )

        mesh.fix_normals(False)
        if self.texture_filename != "":
            mesh.material.texture = os.path.join(self._path, os.path.basename(self.texture_filename))
            mesh.material.particle_size = 0.1
        elif len(self.skins) > 0:
            mesh.material.texture = os.path.join(self._path, os.path.basename(self.skins[0]))
            mesh.material.particle_size = 0.1

        self.add_frame_child(name, mesh)

    def read_header(self, f: BinaryIO):
        self.header = MD2Header._make(read_block(f, "< 4s16l", 1)[0])

        if self.header.ident.decode("ascii") != _SIGNATURE:
            raise BaseException("MD2 Identifier is incorrect")
        if self.header.version != _VERSION:
            raise BaseException("Invalid Version")

    def read_skin(self, f: BinaryIO):
        f.seek(self.header.offset_skins, os.SEEK_SET)
        skin_struct = struct.Struct("< %s" % ("64s" * self.header.num_skins))

        self.skins = [fix_skin_name(skin) for skin in skin_struct.unpack(f.read(skin_struct.size))]

    def read_tex_coords(self, f: BinaryIO):
        f.seek(self.header.offset_st, os.SEEK_SET)
        tcs = np.array(read_block(f, "< 2h", self.header.num_st), dtype=np.float)
        tcs.shape = (-1, 2)
        tcs /= [float(self.header.skin_width), float(self.header.skin_height)]
        self._texcoords = tcs

    def load_frames(self, f: BinaryIO):
        f.seek(self.header.offset_frames, os.SEEK_SET)
        self.frames = [self.read_frame(f) for x in range(self.header.num_frames)]

    def read_frame(self, f: BinaryIO):
        frame_translations = np.array(read_block(f, "< 3f", 2), dtype=np.float)
        scale = frame_translations[0]
        translation = frame_translations[1]

        name = read_block(f, "< 16s", 1)[0][0]
        name = str(name).split("\x00")[0].split("\\")[0].replace("b'", "")

        frame_vertex_data = np.array(read_block(f, "<4B", self.header.num_vertices), dtype=np.uint8)

        frame_vertex_data.shape = (-1, 4)

        vertices_short = frame_vertex_data[:, :3]
        vertices = vertices_short.astype(np.float)
        vertices.shape = (-1, 3)

        vertices *= scale
        vertices += (translation[0], translation[1], 0)
        vertices /= 20.0

        return MD2Frame(name=name, vertices=vertices)

    def to_dict(self) -> Dict[str, Any]:
        animation = self.animation
        if animation == "":
            animation = list(self.animations.keys())[0]

        frame_name = f"{animation}{self._active_frame}"
        result = self._frame_children[frame_name].to_dict()
        result["matrix"] = self.matrix
        result["children"] = {name: self.children[name].to_dict() for name in self.children}
        return result

    def set_texture(self, texture_filename: str):
        for m in self.children.values():
            m.material.texture = texture_filename
            m.material.refresh()

    def toggle_wireframe(self) -> None:
        """Toggle wireframe view of the Object"""
        d = (self.material.display + 1) % 3

        for mat in self.materials.values():
            mat.display = d
            mat.particle_size = 0.01

        if d == POINTS:
            self.shader = PARTICLE_SHADER
            self.refresh()
        else:
            self.shader = DEFAULT_SHADER
            self.refresh()

        for frame in self._frame_children.values():
            frame.toggle_wireframe()
