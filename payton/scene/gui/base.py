"""Graphical User Interface support

GUI Module contains several GUI elements which are all instance of
`payton.scene.geometry.mesh.Mesh` and mainly, their projection matrices
differ in pipeline.

Example code:

    .. include:: ../../examples/basics/15_gui.py

"""
import math
import os
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar, cast

import numpy as np  # type: ignore
from OpenGL.GL import GL_DEPTH_TEST, glDisable, glEnable
from PIL import Image, ImageDraw, ImageFont  # type: ignore

from payton.math.matrix import ortho
from payton.scene.geometry.base import Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.shader import Shader

S2 = TypeVar("S2", bound="Shape2D")


class Shape2D(Mesh):
    """Shape2D is the base of GUI primitives

    An instance of `payton.scene.geometry.mesh.Mesh` so as a result of this,
    you can use the material information to change the color and texture
    of the shape.
    """

    def __init__(
        self,
        position: Tuple[int, int, int],
        size: Tuple[int, int],
        on_click: Optional[Callable] = None,
        opacity: float = 0.5,
        **kwargs: Any,
    ):
        """Initialize Shape2D

        Args:
          opacity: Opacity of the shape. (1: transparent)
          position: Position of the shape in screen. (0, 0 by default.)
        """
        super().__init__(**kwargs)
        self.material.opacity = opacity
        self.__position: Tuple[int, int, int] = position
        self.position = list([float(x) for x in self.__position])
        self.size: Tuple[int, int] = size
        self.on_click: Optional[Callable] = on_click
        self._font: ImageFont = None
        self.parent: Any = None
        self._scene_width: int = 0
        self._scene_height: int = 0

    def add_child(self, name: str, obj: "Shape2D") -> bool:  # type: ignore
        """Add child to the shape, childs position will be relative to its
        parent shape and will be rendered on top of it."""
        res = super().add_child(name, obj)  # type: ignore
        if not res:
            return res
        obj.parent = self
        return res

    def draw(self):
        """Placeholder for draw function"""
        pass

    def draw_text(self):
        """Placeholder for draw text function"""
        pass

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
    ):
        super().render(False, shader, parent_matrix)

    @property
    def font(self) -> None:
        """Font of the shape"""
        if self._font is not None:
            return self._font
        if self.parent is not None:
            return self.parent.font
        return None

    @font.setter
    def font(self, font: ImageFont) -> None:
        """Set font of the text

        Args:
          font: An instance of `PIL.ImageFont`
        """
        self._font = font
        self.draw_text()

    def set_parent_size(self, w: int, h: int):
        self._parent_width = w
        self._parent_height = h
        self._init = False
        self.draw()
        for child in self.children.values():
            cast("Shape2D", child).set_parent_size(self.size[0], self.size[1])

    def click(self, x: int, y: int) -> bool:
        """Click trigger function

        **IMPORTANT!!**

        Basically, this is getting called from
        `payton.scene.controller.Controller` so if you overwrite the
        controller, remember that your GUI callbacks will not work unless
        you handle them yourself or you call the `super()` for mouse action

        Args:
          x: Mouse X coordinate
          y: Mouse Y coordinate

        Returns:
          bool: Hit
        """
        if not callable(self.on_click):
            for child in self.children:
                c = cast("Shape2D", self.children[child]).click(
                    x, y
                )  # type: bool
                if c:
                    return True
            return False

        if self._model_matrix is None:
            return False
        mm = self._model_matrix[3]
        if (
            x > mm[0]
            and x < mm[0] + self.size[0]
            and y > mm[1]
            and y < mm[1] + self.size[1]
        ):
            self.on_click()
            return True
        return False


class Rectangle(Shape2D):
    """Rectangle primitive, an instance of `payton.scene.gui.Shape2D`

    Args:
      size: Size of the rectangle.
    """

    def __init__(
        self,
        position: Tuple[int, int, int],
        size: Tuple[int, int],
        **kwargs: Any,
    ):
        """Initialize the rectangle"""
        super().__init__(position=position, size=size, **kwargs)
        self._init: bool = False
        self.draw()

    def draw(self) -> None:
        """Draw the rectangle.

        Basically, this method gets called right after the initialization
        to build the rectangle buffer.

        Also, if you change the size of rectangle, you have to call this
        method again.
        """
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
    """Text object.

    This is an instance of `payton.scene.gui.Rectangle` and uses Pillow
    library to generate a texture with a text and assigns the texture
    to own material
    """

    def __init__(
        self,
        position: Tuple[int, int, int],
        size: Tuple[int, int],
        label: str = "lorem",
        bgcolor: Optional[Tuple[float, float, float]] = None,
        color: Optional[Tuple[float, float, float]] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize Text

        Args:
          label: Label of the text. (Text to be written)
          bgcolor: Background color of the text (Default, r:0, g:0, b:0, a:0)
          color: Color of the text (Default, r:0, g:0, b:0)
        """
        super().__init__(position=position, size=size, **kwargs)
        self.__label: str = label
        self.bgcolor: List[int] = [0, 0, 0, 0]
        if bgcolor is not None:
            self.bgcolor = [
                math.floor(bgcolor[0] * 255),
                math.floor(bgcolor[1] * 255),
                math.floor(bgcolor[2] * 255),
            ]

        self.color: List[int] = [0, 0, 0]
        if color is not None:
            self.color = [
                math.floor(color[0] * 255),
                math.floor(color[1] * 255),
                math.floor(color[2] * 255),
            ]

        self.draw_text()
        self._init_text: bool = False

    @property
    def label(self) -> str:
        return self.__label

    @label.setter
    def label(self, label: str) -> None:
        self.__label = label
        self.draw_text()

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
    ) -> None:
        """Render the Text

        This calls the render method of
        `payton.scene.geometry.base.Object.render` then renders the text on top
        of the rectangle.
        """
        if not self._init_text:
            self.draw_text()
            self._init_text = True
        super().render(lit, shader, parent_matrix)

    @property
    def text_size(self) -> Tuple[int, int]:
        timg = Image.new("RGBA", (1, 1))
        d = ImageDraw.Draw(timg)
        if self.font is None:
            return d.textsize(self.label)

        return d.textsize(self.label, font=self.font)

    def draw_text(self) -> None:
        """Draw text

        Create an empty transparent image with the rectangle size
        and draw the text on it. Then, assign the image to the material
        """
        img = Image.new("RGBA", self.size, color=tuple(self.bgcolor))
        d = ImageDraw.Draw(img)

        if self.font is not None:
            d.text((1, 1), self.label, fill=tuple(self.color), font=self.font)
        else:
            d.text((1, 1), self.label, fill=tuple(self.color))

        self.material._image = img
        self.material.refresh()


class Hud(Object):
    """Main 2D Hud

    HUD stands for Heads Up Display and it is the Graphical User Interface
    layer on top of your 3D Scene. HUDs are rendered after your 3D Scene by
    disabling Depth Test. So, order of drawing is an important factor here.

    You can create a HUD for debugging reasons. If you want to use any 2D
    primitives, you need to add it into a HUD.

    One HUD is enough to handle all your 2D drawings but it is not restricted
    to one in the scene.
    """

    def __init__(
        self,
        width: int = 800,
        height: int = 600,
        font: str = "",
        font_size: int = 15,
        **kwargs: Any,
    ) -> None:
        """Initialize HUD

        Args:
          width: Window width (overriden by Scene on add_object)
          height: Window height (overriden by Scene on add_object)
          font: Path to font file (See:
            https://pillow.readthedocs.io/en/3.1.x/reference/ImageFont.html)
          font_size: Font size
        """

        super().__init__(**kwargs)
        self.width: int = width
        self.height: int = height
        self._fontname: str = font
        self.children: Dict[str, Shape2D] = {}  # type: ignore
        self._font_size: int = font_size
        if self._fontname != "":
            self.set_font(self._fontname, self._font_size)
        self._font: ImageFont = None
        self._projection_matrix: Optional[np.ndarray] = None
        try:
            self.set_font(
                os.path.join(
                    os.path.dirname(os.path.abspath(__file__)), "monofonto.ttf"
                )
            )
        except OSError:
            # font not found but pillow has better than nothinf font
            pass

    @property
    def font(self) -> ImageFont:
        return self._font

    def add_child(self, name: str, obj: Shape2D) -> bool:  # type: ignore
        """Add child to HUD

        Basically, anything that you want to draw on the screen should be a
        child of HUD. Scene gathers HUDs in a different loop and renders them
        in a separate cycle.

        Args:
          name: Name of the child shape
          obj: Shape object to add
        """
        res = super().add_child(name, obj)
        obj._scene_height = self.height
        obj._scene_width = self.width

        if not res:
            return res
        obj.parent = self
        return res

    def render(self, lit: bool, shader: Shader, *_args: Any) -> None:
        """Render HUD

        Disables depth test and renders child primitives
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

    def set_size(self, w: int, h: int):
        """Set size of the HUD view

        Args:
          w: Width of the viewport
          h: Height of the viewport
        """
        self.width = w
        self.height = h
        for child in self.children.values():
            cast(Shape2D, child).set_parent_size(w, h)
            child._init = False
            child.draw()

        self._projection_matrix = None

    def set_font(self, font_name: str, font_size: int = 16) -> None:
        """Set font of the HUD

        Args:
          font_name: Path to the font file. TrueType or OpenType fonts are
            also supported.
          font_size: Font size.
        """
        self._font = ImageFont.truetype(font_name, font_size)
