import math
import os
from textwrap import wrap
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar, cast

import numpy as np
from OpenGL.GL import GL_DEPTH_TEST, glDisable, glEnable
from PIL import Image, ImageDraw, ImageFont

from payton.math.functions import ortho
from payton.math.vector import Vector3D
from payton.scene.geometry.base import Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.shader import Shader

S2 = TypeVar("S2", bound="Shape2D")


class Shape2D(Mesh):
    def __init__(
        self,
        position: List[int],
        size: List[int],
        on_click: Optional[Callable] = None,
        opacity: float = 0.1,
        **kwargs: Any,
    ):
        """Initialize 2D Shape

        Keyword arguments:
        position -- Position of the shape on the screen (x, y, and optionally Z as integer)
        size -- Size of the shape (w, h as integer)
        on_click -- On Click event callback (takes no arguments)
        opacity -- Opacity of the shape
        """
        super().__init__(**kwargs)
        self.material.opacity = opacity
        position = list(position)
        if len(position) == 2:
            position.append(0)
        position[2] = -1
        self.__position = position
        self.position = [float(x) for x in self.__position]
        self.size = size
        self.on_click: Optional[Callable] = on_click
        self._font: ImageFont = None
        self.parent: Any = None
        self._scene_width: int = 0
        self._scene_height: int = 0

    @property
    def physics(self) -> bool:
        return False

    def add_child(self, name: str, obj: Object) -> bool:
        """Add a child shape to this shape object.

        This method overrides the parent by arguments. Child object's position
        follows the parent.

        Keyword arguments:
        name -- Name of the object to be added
        obj -- Shape2D Object to add.
        """
        res = super().add_child(name, cast(Object, obj))
        if not res:
            return res
        if isinstance(obj, Shape2D):
            obj.parent = self
        return res

    def draw(self) -> None:
        """Placeholder for draw function"""
        ...

    def draw_text(self) -> None:
        """Placeholder for draw text function"""
        ...

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
        _primitive: int = None,
    ) -> None:
        """Render cycle for the Shape 2D

        We override the `lit` parameter for the super.
        This is not actually intended to be used publicly except if you want to
        setup your own render cycle.
        """
        super().render(False, shader, parent_matrix)

    @property
    def font(self) -> None:
        """Return the fond of the shape"""
        if self._font is not None:
            return self._font
        if self.parent is not None:
            return self.parent.font
        return None

    @font.setter
    def font(self, font: ImageFont) -> None:
        """Set the font of the Shape

        Keyword arguments:
        font -- ImageFont object
        """
        self._font = font
        self.draw_text()

    def _set_parent_size(self, w: int, h: int) -> None:
        self._parent_width = w
        self._parent_height = h
        self._init = False
        self.draw()
        for child in self.children.values():
            cast("Shape2D", child)._set_parent_size(self.size[0], self.size[1])

    def click(self, x: int, y: int) -> Optional[Mesh]:
        """Check for click event

        Keyword arguments:
        x -- Cursor X
        y -- Cursor Y
        """
        if not callable(self.on_click):
            for child in self.children:
                c = cast(Mesh, self.children[child]).click(x, y)
                if c:
                    return cast(Mesh, self.children[child])
            return None

        if self._model_matrix is None:
            return None
        mm = self._model_matrix[3]
        if x > mm[0] and x < mm[0] + self.size[0] and y > mm[1] and y < mm[1] + self.size[1]:
            self.on_click()
            return self
        return None


class Rectangle(Shape2D):
    def __init__(
        self,
        position: List[int],
        size: List[int],
        **kwargs: Any,
    ):
        """Initialize Rectangle

        Keyword arguments:
        position -- Tuple as [X, Y, Z] positions
        size -- Size of the rectangle
        """
        super().__init__(position=position, size=size, **kwargs)
        self._init: bool = False
        self.draw()

    def draw(self) -> None:
        """Create the rectangle polygons"""
        w, h = self.size
        self.clear_triangles()
        if not self._init:
            self.add_triangle(
                [[0, 0, 1], [w, h, 1], [w, 0, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
            )
            self.add_triangle(
                [[0, 0, 1], [0, h, 1], [w, h, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
            )
            self._init = True


class Text(Rectangle):
    def __init__(
        self,
        position: List[int],
        size: List[int],
        label: str = "lorem",
        bgcolor: Optional[Vector3D] = None,
        color: Optional[Vector3D] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize Text object (label)

        Keyword arguments:
        position -- Tuple as [X, Y, Z] positions on the screen
        size -- Size of the text area
        label -- Text / Label to be printed
        bgcolor -- Background color of the text
        color -- Color of the text"""
        super().__init__(position=position, size=size, **kwargs)
        self.__label: str = label
        self.bgcolor: Vector3D = [0, 0, 0, 0]
        if bgcolor is not None:
            self.bgcolor = bgcolor

        self.color: Vector3D = [0, 0, 0]
        if color is not None:
            self.color = color

        self.crop = [0, 0, 0, 0]
        self._init_text: bool = False
        self._max_char_width = 0.0

    @property
    def label(self) -> str:
        """Get the label of the text"""
        return self.__label

    @label.setter
    def label(self, label: str) -> None:
        """Set the label of the text

        Keyword arguments:
        label -- Label string
        """
        self.__label = label
        self._init_text = False

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
        _primitive: int = None,
    ) -> None:
        """Render the text object by initializing the text material if not initialized yet

        Keyword arguments:
        lit -- Is the object illuminated?
        shader -- Shader to use while rendering the object
        parent_matrix -- Parent object's matrix if this is a child object
        _primitive -- override the scene wide primitive (rendering) mode. (Point, Wire, Solid)
        """
        if not self._init_text:
            self.draw_text()
        super().render(lit, shader, parent_matrix)

    @property
    def text_size(self) -> Tuple[int, int]:
        """Return the text size in pixels"""
        res = (0, 0)
        timg = Image.new("RGBA", (1, 1))
        d = ImageDraw.Draw(timg)

        res = d.textsize(self.label, font=self.font)
        lres = list(res)
        lres[0] = max(lres[0], self.size[0])
        lres[1] = max(lres[1], self.size[1])
        return lres[0], lres[1]

    def wrap(self, width_in_pixels: int) -> None:
        """Word-wrap the text to fit into the given pixel size

        Keyword arguments:
        width_in_pixels -- Width size in pixels to fit the text into
        """
        original = self.label
        self.label = ""
        if self.font is None:
            return
        if self._max_char_width == 0.0:
            self._max_char_width = self.font.getsize('_')[0]
        split_length = int(math.floor(width_in_pixels / self._max_char_width)) - 1
        parts = wrap(original, split_length)
        self.__label = "\n".join(parts)

        self._init_text = False

    def draw_text(self) -> None:
        """Create the text material and initialize the font if needed"""
        if self._init_text:
            return
        bgcolor = (
            int(self.bgcolor[0] * 255),
            int(self.bgcolor[1] * 255),
            int(self.bgcolor[2] * 255),
            int(self.bgcolor[3] * 255),
        )
        img = Image.new("RGBA", self.text_size, color=bgcolor)
        d = ImageDraw.Draw(img, "RGBA")
        color = (int(self.color[0] * 255), int(self.color[1] * 255), int(self.color[2] * 255), 255)
        if self.font is not None:
            d.text((1, 1), self.label, fill=color, font=self.font)
        else:
            d.text((1, 1), self.label, fill=color)
        if any(self.crop):
            img = img.crop(self.crop)
        del d

        self.material._image = img
        self.material.opacity = 1.0
        self.material.refresh()
        self._init_text = True


class Hud(Object):
    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        font: str = "",
        font_size: int = 15,
        **kwargs: Any,
    ) -> None:
        """Initialize HUD Object to render HUD elements.

        You need a HUD in a scene to render 2D shapes on top of your scene

        Keyword arguments:
        width -- Width of the HUD area
        height -- Height of the HUD area
        font -- Font name to use (default: monofonto provided by the package)
        font_size -- Font size in pixels
        """
        super().__init__(**kwargs)
        self.width: int = width
        self.height: int = height
        self._fontname: str = font
        self.children: Dict[str, Object] = {}
        self._font_size: int = font_size
        if self._fontname != "":
            self.set_font(self._fontname, self._font_size)
        self._font: ImageFont = None
        self._projection_matrix: Optional[np.ndarray] = None
        try:
            self.set_font(os.path.join(os.path.dirname(os.path.abspath(__file__)), "monofonto.ttf"))
        except OSError:
            # font not found but pillow has better than nothinf font
            pass

    @property
    def font(self) -> ImageFont:
        """Return the HUD Image Font"""
        return self._font

    def add_child(self, name: str, obj: Object) -> bool:
        """Add 2D Shape into Hud.

        Note: this is a type ignore due to it's mismatch with Object add_child method

        Keyword arguments:
        name -- Name of the object
        obj -- Shape2D object to add"""
        res = super().add_child(name, obj)
        if isinstance(obj, Shape2D):
            obj._scene_height = self.height
            obj._scene_width = self.width
            obj.parent = self

        if not res:
            return res
        return res

    def render(self, lit: bool, shader: Shader, *_args: Any) -> None:
        """Main render cycle for HUD

        Disables the DEPTH test and draws the objects from parent to child
        """
        if not self.visible:
            return
        lit = False
        glDisable(GL_DEPTH_TEST)
        if self._projection_matrix is None:
            self._projection_matrix = ortho(0, self.width, self.height, 0)

        shader.set_int("view_mode", 1)
        shader.set_matrix4x4_np("projection", self._projection_matrix)
        for child in self.children:
            self.children[child].render(lit, shader, None)
        glEnable(GL_DEPTH_TEST)

    def set_size(self, w: int, h: int) -> None:
        """Set HUD size

        Keyword arguments:
        w -- Width of the HUD
        h -- Height of the HUD
        """
        self.width = w
        self.height = h
        for child in self.children.values():
            if isinstance(child, Shape2D):
                child._set_parent_size(w, h)
                child._init = False
                child.draw()

        self._projection_matrix = None

    def set_font(self, font_name: str, font_size: int = 16) -> None:
        """Set True Type font for the Hud

        Keyword arguments:
        font_name -- Name of the True Type font installed in the system
        font_size -- Size of the font in pixels
        """
        self._font = ImageFont.truetype(font_name, font_size)
