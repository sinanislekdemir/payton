import os
from typing import Any, Dict, List, Optional

import numpy as np  # type: ignore
from OpenGL.GL import (
    GL_BLEND,
    GL_CULL_FACE,
    GL_LINEAR,
    GL_LINEAR_MIPMAP_LINEAR,
    GL_ONE,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_REPEAT,
    GL_RGB,
    GL_RGBA,
    GL_SRC_ALPHA,
    GL_TEXTURE0,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TEXTURE_WRAP_S,
    GL_TEXTURE_WRAP_T,
    GL_UNPACK_ALIGNMENT,
    GL_UNSIGNED_BYTE,
    glActiveTexture,
    glBindTexture,
    glBlendFunc,
    glDisable,
    glEnable,
    glGenerateMipmap,
    glGenTextures,
    glPixelStorei,
    glTexImage2D,
    glTexParameterf,
)
from PIL import Image  # type: ignore

from payton.scene.shader import Shader
from payton.scene.types import IList

SOLID = 0  # type: int
WIREFRAME = 1  # type: int
POINTS = 2  # type: int

RED = [1.0, 0.0, 0.0]  # type: List[float]
GREEN = [0.0, 1.0, 0.0]  # type: List[float]
BLUE = [0.0, 0.0, 1.0]  # type: List[float]
CRIMSON = [220 / 255.0, 20 / 255.0, 60 / 255.0]  # type: List[float]
PINK = [1.0, 192 / 255.0, 203 / 255.0]  # type: List[float]
VIOLET_RED = [1.0, 62 / 255.0, 150 / 255.0]  # type: List[float]
DEEP_PINK = [1.0, 20 / 255.0, 147 / 255.0]  # type: List[float]
ORCHID = [218 / 255.0, 112 / 255.0, 214 / 255.0]  # type: List[float]
PURPLE = [128 / 255.0, 0.0, 128 / 255.0]  # type: List[float]
NAVY = [0.0, 0.0, 0.5]  # type: List[float]
ROYAL_BLUE = [65 / 255.0, 105 / 255.0, 225 / 255.0]  # type: List[float]
LIGHT_STEEL_BLUE = [176 / 255.0, 196 / 255.0, 222 / 255.0]  # type: List[float]
STEEL_BLUE = [70 / 255.0, 130 / 255.0, 180 / 255.0]  # type: List[float]
TURQUOISE = [0.0, 245 / 255.0, 1.0]  # type: List[float]
YELLOW = [1.0, 1.0, 0.0]  # type: List[float]
GOLD = [1.0, 225 / 255.0, 0.0]  # type: List[float]
ORANGE = [1.0, 165 / 255.0, 0.0]  # type: List[float]
WHITE = [1.0, 1.0, 1.0]  # type: List[float]
BLACK = [0.0, 0.0, 0.0]  # type: List[float]
DARK_GRAY = [0.2, 0.2, 0.2]  # type: List[float]
LIGHT_GRAY = [0.8, 0.8, 0.8]  # type: List[float]

DEFAULT = "default"
NO_VERTEX_ARRAY = -1
NO_INDICE = -2
EMPTY_VERTEX_ARRAY = -3

BASE_PARTICLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "particle.png")


class Material(object):
    def __init__(
        self,
        color: Optional[List[float]] = None,
        display: int = SOLID,
        lights: bool = True,
        texture: str = "",
        opacity: float = 1.0,
        **kwargs: Any,
    ):
        self._color: List[float] = [1.0, 1.0, 1.0] if color is None else color
        self._color_np: np.ndarray = np.array(self._color, dtype=np.float32)
        self.display: int = display
        self.lights: bool = lights
        self.texture: str = texture
        self.particle_texture: str = BASE_PARTICLE
        self.opacity: float = opacity
        self.particle_size: float = 0.16
        self._image: Optional[Image] = None
        self._indices: IList = []
        self._vao: int = NO_VERTEX_ARRAY
        self._vbos: List[int] = []

        self._vertex_count: int = 0
        self._index_count: int = 0

        self._initialized: bool = False
        self._texture: Optional[int] = None
        self._particle_texture: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "color": self.color,
            "display": self.display,
            "texture": self.texture,
            "opacity": self.opacity,
            "indices": self._indices,
        }

    @property
    def index_count(self):
        if self._index_count > 0:
            return self._index_count
        self._index_count = len(self._indices)
        return self._index_count

    @property
    def color(self):
        return self._color

    @color.setter
    def color(self, color: List[float]) -> None:
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Material":
        res = cls()
        res.color = d["color"]
        res.display = d["display"]
        res.texture = d["texture"]
        res.opacity = d["opacity"]
        res._indices = d["indices"]
        return res

    def build(self) -> bool:
        self._initialized = True
        if os.path.isfile(self.texture):
            img = Image.open(self.texture)
            self.load_texture(img)
        if self._image is not None:
            self.load_texture(self._image)
        if os.path.isfile(self.particle_texture):
            img = Image.open(self.particle_texture)
            self.load_texture(img, particle=True)
        return True

    def load_texture(self, img: Image, particle: bool = False) -> None:
        img_data = np.fromstring(img.tobytes(), np.uint8)
        width, height = img.size
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        if particle:
            self._particle_texture = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, self._particle_texture)
        else:
            self._texture = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT)

        mode = GL_RGBA
        if img.mode == "RGB":
            mode = GL_RGB
        if img.mode == "P":
            img = img.convert("RGB")
            img_data = np.fromstring(img.tobytes(), np.uint8)
            mode = GL_RGB
        glTexImage2D(
            GL_TEXTURE_2D, 0, mode, width, height, 0, mode, GL_UNSIGNED_BYTE, img_data,
        )
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)

    def refresh(self) -> None:
        self._initialized = False

    def render(self, lit: bool, shader: Shader, mode: Optional[int] = None,) -> None:
        if not self._initialized:
            self.build()

        _mode = Shader.LIGHT_COLOR

        if self.display == SOLID:
            if lit and self.lights:
                if self._texture is not None:
                    _mode = Shader.LIGHT_TEXTURE
                else:
                    _mode = Shader.LIGHT_COLOR
            else:
                if self._texture is not None:
                    _mode = Shader.NO_LIGHT_TEXTURE
                else:
                    _mode = Shader.NO_LIGHT_COLOR
        else:
            _mode = Shader.NO_LIGHT_COLOR

        if mode is not None:
            _mode = mode

        glEnable(GL_BLEND)
        glDisable(GL_CULL_FACE)
        blend = GL_ONE_MINUS_SRC_ALPHA
        if self.display == POINTS:
            blend = GL_ONE
        glBlendFunc(GL_SRC_ALPHA, blend)

        if self._texture is not None and self.display != POINTS:
            check = shader.get_location("tex_unit")
            if check > -1:
                glActiveTexture(GL_TEXTURE0)
                glBindTexture(GL_TEXTURE_2D, self._texture)
                shader.set_int("tex_unit", 0)

        if self._particle_texture is not None and self.display == POINTS:
            check = shader.get_location("tex_unit")
            shader.set_float("particle_size", self.particle_size)
            if check > -1:
                glActiveTexture(GL_TEXTURE0)
                glBindTexture(GL_TEXTURE_2D, self._particle_texture)
                shader.set_int("tex_unit", 0)

        if not shader._depth_shader:
            shader.set_vector3_np("object_color", self._color_np)
            shader.set_float("opacity", self.opacity)
            shader.set_int("material_mode", _mode)
            shader.set_int("lit", 1 if lit else 0)
            if not self.lights:
                shader.set_int("lit", 0)
