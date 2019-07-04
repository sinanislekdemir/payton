"""
What is a material?

Materials define how your scene entities look like. Their colors, shininess,
or displaying them as solid objects or wireframes, all are defined inside
object materials. This also effects if your object will respond to light
sources or not.

There are also pre-defined colors in this module

* Material
  * Shader (`payton.scene.shader`)

"""
import os
import copy
import numpy as np  # type: ignore
from PIL import Image  # type: ignore

from OpenGL.GL import (
    glGenTextures,
    glPixelStorei,
    GL_UNPACK_ALIGNMENT,
    glBindTexture,
    GL_TEXTURE_2D,
    glTexParameterf,
    GL_TEXTURE_MAG_FILTER,
    GL_LINEAR,
    GL_TEXTURE_MIN_FILTER,
    GL_LINEAR_MIPMAP_LINEAR,
    GL_TEXTURE_WRAP_S,
    GL_CLAMP_TO_EDGE,
    GL_TEXTURE_WRAP_T,
    glTexImage2D,
    glActiveTexture,
    glEnable,
    glDisable,
    GL_BLEND,
    glBlendFunc,
    GL_SRC_ALPHA,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_TEXTURE0,
    GL_RGBA,
    GL_RGB,
    GL_UNSIGNED_BYTE,
    glGenerateMipmap,
)

from typing import Any, List, Optional

from payton.scene.shader import Shader
from payton.scene.light import Light

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

GLOBAL_SHADER: Optional[Shader] = None


class Material(object):
    """
    Material information holder.
    """

    def __init__(self, **args: Any):
        """
        Initialize Material

        Color is constructed as a tuple of 3 floats. (Payton does not currently
        support transparency at MVP.) [1.0, 1.0, 1.0] are [Red, Green, Blue]

        Each element of color is a float between 0 and 1.
        (0 - 255 respectively)
        Also, there are pre-defined colors.

        Display Mode has 2 modes. Solid and Wireframe. Wireframe is
        often rendered in a faster way. Also good for debugging your
        object.

        Default variables:

            {'color': [1.0, 1.0, 1.0, 1.0],
             'display': SOLID}

        Args:
          color: Color of material
          display: Display type of material, SOLID / WIREFRAME (Default SOLID)
          lights: Effected by lights? (Default true)
          texture: Texture file name
          opacity: Opacity of the material (0 fully transparent, 1 opaque)
        """

        self.color: List[float] = args.get("color", [1.0, 1.0, 1.0])
        self.display: int = args.get("display", SOLID)
        self.lights: bool = args.get("lights", True)
        self.texture: str = args.get("texture", "")
        self.opacity: float = args.get("opacity", 1.0)
        self._image: Optional[Image] = None

        variables = [
            "model",
            "view",
            "projection",
            "material_mode",
            "light_pos",
            "light_color",
            "LIGHT_COUNT",
            "object_color",
            "opacity",
            "view_mode",
        ]  # type: List[str]

        self.shader: Shader = Shader(variables=variables)

        self._initialized: bool = False
        self._texture: Optional[int] = None

    def build_shader(self) -> bool:
        """Build material shaders

        Must be called at object build stage after generating vba.
        An active vba is required for building shader properly.
        """
        global GLOBAL_SHADER
        if GLOBAL_SHADER is None:
            self.shader.build()
            GLOBAL_SHADER = copy.deepcopy(self.shader)
        else:
            self.shader = GLOBAL_SHADER
        self._initialized = True
        if os.path.isfile(self.texture):
            img = Image.open(self.texture)
            self.load_texture(img)
        if self._image is not None:
            self.load_texture(self._image)
        return True

    def load_texture(self, img: Image) -> None:
        img_data = np.fromstring(img.tobytes(), np.uint8)
        width, height = img.size
        self._texture = glGenTextures(1)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(
            GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR
        )
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        mode = GL_RGBA
        if img.mode == "RGB":
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

    def refresh(self) -> None:
        self._initialized = False

    def render(
        self,
        proj: np.ndarray,
        view: np.ndarray,
        model: np.ndarray,
        lights: List[Light],
        mode: Optional[int] = None,
    ) -> None:
        """Render material

        This function must be called before rendering the actual object

        Args:
          proj: Projection materix
          view: View matrix
          model: Model matrix
          lights: Light objects in the scene
          mode: Set explicit shader mode (optional - used for vertex colors)
        """
        if not self._initialized:
            self.build_shader()

        if self.display == SOLID:
            if self.lights and len(lights) > 0:
                if self._texture is not None:
                    self.shader._mode = Shader.LIGHT_TEXTURE
                else:
                    self.shader._mode = Shader.LIGHT_COLOR
            else:
                if self._texture is not None:
                    self.shader._mode = Shader.NO_LIGHT_TEXTURE
                else:
                    self.shader._mode = Shader.NO_LIGHT_COLOR
        else:
            self.shader._mode = Shader.NO_LIGHT_COLOR

        if mode is not None:
            self.shader._mode = mode

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        self.shader.use()
        self.shader.set_int("material_mode", self.shader._mode)
        self.shader.set_matrix4x4_np("model", model)
        if view is None:
            self.shader.set_int("view_mode", 1)
        else:
            self.shader.set_matrix4x4_np("view", view)
            self.shader.set_int("view_mode", 0)
        self.shader.set_matrix4x4_np("projection", proj)
        self.shader.set_float("opacity", self.opacity)

        if self._texture is not None:
            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, self._texture)
            self.shader.set_int("tex_unit", 0)

        light_array = [light.position for light in lights]
        lcolor_array = [light.color for light in lights]
        light_array = np.array(light_array, dtype=np.float32)
        lcolor_array = np.array(lcolor_array, dtype=np.float32)
        self.shader.set_vector3_array_np("light_pos", light_array, len(lights))
        self.shader.set_vector3_array_np(
            "light_color", lcolor_array, len(lights)
        )
        self.shader.set_int("LIGHT_COUNT", len(lights))

        self.shader.set_vector3_np(
            "object_color", np.array(self.color, dtype=np.float32)
        )

    def end(self) -> None:
        self.shader.end()
        glDisable(GL_BLEND)
