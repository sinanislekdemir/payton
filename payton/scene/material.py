"""
What is a material?

Materials define how your scene entities look like. Their colors, shininess,
or displaying them as solid objects or wireframes, all are defined inside
object materials. This also effects if your object will respond to light
sources or not.

There are also pre-defined colors in this module
"""
import os
from PIL import Image
import numpy as np
from payton.scene.shader import Shader
from OpenGL.GL import (glGenTextures, glPixelStorei, GL_UNPACK_ALIGNMENT,
                       glBindTexture, GL_TEXTURE_2D, glTexParameterf,
                       GL_TEXTURE_MAG_FILTER, GL_LINEAR, GL_TEXTURE_MIN_FILTER,
                       GL_LINEAR_MIPMAP_LINEAR, GL_TEXTURE_WRAP_S,
                       GL_CLAMP_TO_EDGE, GL_TEXTURE_WRAP_T, glTexImage2D,
                       glActiveTexture, GL_TEXTURE0,
                       GL_RGBA, GL_UNSIGNED_BYTE, glGenerateMipmap)


SOLID = 0
WIREFRAME = 1
POINTS = 2

RED = [1.0, 0.0, 0.0]
GREEN = [0.0, 1.0, 0.0]
BLUE = [0.0, 0.0, 1.0]
CRIMSON = [220/255.0, 20/255.0, 60/255.0]
PINK = [1.0, 192/255.0, 203/255.0]
VIOLET_RED = [1.0, 62/255.0, 150/255.0]
DEEP_PINK = [1.0, 20/255.0, 147/255.0]
ORCHID = [218/255.0, 112/255.0, 214/255.0]
PURPLE = [128/255.0, 0.0, 128/255.0]
NAVY = [0.0, 0.0, 0.5]
ROYAL_BLUE = [65/255.0, 105/255.0, 225/255.0]
LIGHT_STEEL_BLUE = [176/255.0, 196/255.0, 222/255.0]
STEEL_BLUE = [70/255.0, 130/255.0, 180/255.0]
TURQUOISE = [0.0, 245/255.0, 1.0]
YELLOW = [1.0, 1.0, 0.0]
GOLD = [1.0, 225/255.0, 0.0]
ORANGE = [1.0, 165/255.0, 0.0]
WHITE = [1.0, 1.0, 1.0]
BLACK = [0.0, 0.0, 0.0]
DARK_GRAY = [0.2, 0.2, 0.2]
LIGHT_GRAY = [0.8, 0.8, 0.8]


class Material(object):
    """
    Material information holder.
    """
    def __init__(self, **args):
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
        """

        self.color = args.get('color', [1.0, 1.0, 1.0])
        self.display = args.get('display', SOLID)
        self.lights = args.get('lights', True)
        self.texture = args.get('texture', '')

        variables = ['model', 'view', 'projection', 'material_mode',
                     'light_pos', 'light_color', 'object_color']
        self.shader = Shader(variables=variables)

        self._initialized = False
        self._texture = None

    def build_shader(self):
        """Build material shaders

        Must be called at object build stage after generating vba.
        An active vba is required for building shader properly.
        """
        self.shader.build()
        self._initialized = True
        if os.path.isfile(self.texture):
            self.load_texture()
        return True

    def load_texture(self):
        img = Image.open(self.texture).transpose(Image.FLIP_TOP_BOTTOM)
        img_data = np.fromstring(img.tobytes(), np.uint8)
        width, height = img.size
        self._texture = glGenTextures(1)
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1)
        glBindTexture(GL_TEXTURE_2D, self._texture)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                        GL_LINEAR_MIPMAP_LINEAR)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, img_data)
        glGenerateMipmap(GL_TEXTURE_2D)

    def render(self, proj, view, model, lights, mode=None):
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
            if self.lights:
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

        self.shader.use()
        self.shader.set_int('material_mode', self.shader._mode)
        self.shader.set_matrix4x4_np('model', model)
        self.shader.set_matrix4x4_np('view', view)
        self.shader.set_matrix4x4_np('projection', proj)

        if self._texture is not None:
            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, self._texture)
            self.shader.set_int('tex_unit', 0)

        for light in lights:
            self.shader.set_vector3_np('light_pos', light._position)
            self.shader.set_vector3_np('light_color', light._color)
        self.shader.set_vector3_np('object_color', np.array(self.color,
                                                            dtype=np.float32))

    def end(self):
        self.shader.end()
