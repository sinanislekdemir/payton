import os
from typing import Any, Dict, List, Optional

import numpy as np
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
from PIL import Image

from payton.math.vector import Vector3D
from payton.scene.shader import Shader
from payton.scene.types import IList

SOLID = 0  # type: int
WIREFRAME = 1  # type: int
POINTS = 2  # type: int

RED = [1.0, 0.0, 0.0]  # type: Vector3D
GREEN = [0.0, 1.0, 0.0]  # type: Vector3D
BLUE = [0.0, 0.0, 1.0]  # type: Vector3D
CRIMSON = [220 / 255.0, 20 / 255.0, 60 / 255.0]  # type: Vector3D
PINK = [1.0, 192 / 255.0, 203 / 255.0]  # type: Vector3D
VIOLET_RED = [1.0, 62 / 255.0, 150 / 255.0]  # type: Vector3D
DEEP_PINK = [1.0, 20 / 255.0, 147 / 255.0]  # type: Vector3D
ORCHID = [218 / 255.0, 112 / 255.0, 214 / 255.0]  # type: Vector3D
PURPLE = [128 / 255.0, 0.0, 128 / 255.0]  # type: Vector3D
NAVY = [0.0, 0.0, 0.5]  # type: Vector3D
ROYAL_BLUE = [65 / 255.0, 105 / 255.0, 225 / 255.0]  # type: Vector3D
LIGHT_STEEL_BLUE = [176 / 255.0, 196 / 255.0, 222 / 255.0]  # type: Vector3D
STEEL_BLUE = [70 / 255.0, 130 / 255.0, 180 / 255.0]  # type: Vector3D
TURQUOISE = [0.0, 245 / 255.0, 1.0]  # type: Vector3D
YELLOW = [1.0, 1.0, 0.0]  # type: Vector3D
GOLD = [1.0, 225 / 255.0, 0.0]  # type: Vector3D
ORANGE = [1.0, 165 / 255.0, 0.0]  # type: Vector3D
WHITE = [1.0, 1.0, 1.0]  # type: Vector3D
BLACK = [0.0, 0.0, 0.0]  # type: Vector3D
DARK_GRAY = [0.2, 0.2, 0.2]  # type: Vector3D
LIGHT_GRAY = [0.8, 0.8, 0.8]  # type: Vector3D

DEFAULT = "default"
NO_VERTEX_ARRAY = -1
NO_INDICE = -2
EMPTY_VERTEX_ARRAY = -3

BASE_PARTICLE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "particle.png")

_IMAGE_CACHE: Dict[str, int] = {}


class Material:
    def __init__(
        self,
        color: Optional[Vector3D] = None,
        display: int = SOLID,
        lights: bool = True,
        texture: str = "",
        opacity: float = 1.0,
        **kwargs: Any,
    ):
        """Payton materials are quite simple and does not support some
        functionalities like Game Engines or design softwares.

        Keyword arguments:
        color -- Material color
        display -- Display type of the material (SOLID, WIREFRAME, POINTS)
        lights -- Is this material effected by the light shading?
        texture -- Texture filename
        opacity -- Opacity of the material
        """
        self._color = [1.0, 1.0, 1.0] if color is None else color
        self._color_np: np.ndarray = np.array(list(self._color), dtype=np.float32)
        self.display: int = display
        self.lights: bool = lights
        self.texture: str = texture
        self.particle_texture: str = BASE_PARTICLE
        self.opacity: float = opacity
        self.particle_size: float = 0.16
        self._image: Optional[Image.Image] = None
        self._indices: IList = []
        self._vao: int = NO_VERTEX_ARRAY
        self._vbos: List[int] = []

        self._vertex_count: int = 0
        self._index_count: int = 0

        self._initialized: bool = False
        self._texture: Optional[int] = None
        self._particle_texture: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert the material into dictionary"""
        return {
            "color": self.color,
            "display": self.display,
            "texture": self.texture,
            "opacity": self.opacity,
            "indices": self._indices,
        }

    @property
    def index_count(self) -> int:
        """Return the number of indexes for OpenGL"""
        if self._index_count > 0:
            return self._index_count
        self._index_count = len(self._indices)
        return self._index_count

    @property
    def color(self) -> Vector3D:
        """Return the material color"""
        return self._color

    @color.setter
    def color(self, color: Vector3D) -> None:
        """Set the material color

        Keyword arguments:
        color -- Color to set
        """
        self._color = color
        self._color_np = np.array(list(self._color), dtype=np.float32)

    @classmethod
    def from_dict(cls, material_dictionary: Dict[str, Any]) -> "Material":
        """Import material from dictionary

        material_dictionary -- Dictionary to import"""
        res = cls()
        res.color = material_dictionary["color"]
        res.display = material_dictionary["display"]
        res.texture = material_dictionary["texture"]
        res.opacity = material_dictionary["opacity"]
        res._indices = material_dictionary["indices"]
        return res

    def build(self) -> bool:
        global _IMAGE_CACHE
        """Build the material"""
        self._initialized = True
        if os.path.isfile(self.texture):
            if self.texture in _IMAGE_CACHE:
                self._texture = _IMAGE_CACHE[self.texture]
            else:
                img = Image.open(self.texture)
                _IMAGE_CACHE[self.texture] = self.load_texture(img)
        if self._image is not None:
            self.load_texture(self._image)
        if os.path.isfile(self.particle_texture):
            img = Image.open(self.particle_texture)
            self.load_texture(img, particle=True)
        return True

    def load_texture(self, img: Image.Image, particle: bool = False) -> int:
        """Load texture directly from PIL Image object

        Keyword arguments:
        img -- Image to load
        particle -- Is this a particle material?
        """
        img_data = np.fromstring(img.tobytes(), np.uint8)  # type: ignore
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
            img_data = np.fromstring(img.tobytes(), np.uint8)  # type: ignore
            mode = GL_RGB
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            mode,
            width,
            height,
            0,
            mode,
            GL_UNSIGNED_BYTE,
            img_data,
        )
        glGenerateMipmap(GL_TEXTURE_2D)
        glBindTexture(GL_TEXTURE_2D, 0)
        return self._texture or -1

    def refresh(self) -> None:
        """Refresh / apply the material changes into OpenGL context"""
        self._initialized = False

    def material_mode(self, lit: bool) -> int:
        """Return the material mode

        Keyword argument:
        lit -- Is this a lit material?
        """
        if self.display == SOLID and lit and self.lights and self._texture is not None:
            return Shader.LIGHT_TEXTURE
        elif self.display == SOLID and lit and self.lights:
            return Shader.LIGHT_COLOR
        elif self.display == SOLID and self._texture is not None:
            return Shader.NO_LIGHT_TEXTURE
        else:
            return Shader.NO_LIGHT_COLOR

    def render(
        self,
        lit: bool,
        shader: Shader,
        mode: Optional[int] = None,
    ) -> None:
        """Render the material

        Keyword arguments:
        lit -- Is this a lit material?
        shader -- Shader to use for rendering the material
        mode -- Material mode
        """
        if not self._initialized:
            self.build()

        _mode = mode or self.material_mode(lit)

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
