"""Graphical User Interface support

GUI Module contains several GUI elements which are all instance of
`payton.scene.geometry.Mesh` and mainly, their projection matrices
differ in pipeline.

Example code:

    .. include:: ../../examples/basics/15_gui.py

"""
import numpy as np  # type: ignore
from typing import Any, Tuple, Callable, TypeVar, List, Optional, Dict
from PIL import Image, ImageDraw, ImageFont  # type: ignore

from OpenGL.GL import glEnable, GL_DEPTH_TEST, glDisable

from payton.scene.geometry import Mesh, Object
from payton.scene.light import Light
from payton.math.matrix import ortho


S2 = TypeVar("S2", bound="Shape2D")


class Shape2D(Mesh):
    """Shape2D is the base of GUI primitives

    An instance of `payton.scene.geometry.Mesh` so as a result of this,
    you can use the material information to change the color and texture
    of the shape.
    """

    def __init__(self, **args: Any):
        """Initialize Shape2D

        Args:
          opacity: Opacity of the shape. (1: transparent)
          position: Position of the shape in screen. (0, 0 by default.)
        """
        super().__init__(**args)
        self.material.opacity = args.get("opacity", 0.5)
        self.__position: Tuple[int, int, int] = args.get("position", (0, 0, 0))
        self.position = list([float(x) for x in self.__position])
        self.size: Tuple[int, int] = args.get("size", (100, 100))
        self.on_click: Callable = args.get("on_click", None)
        self._font: ImageFont = None
        self.parent: Any = None

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
                c = self.children[child].click(x, y)  # type: bool
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

    def __init__(self, **args: Any):
        """Initialize the rectangle"""
        super().__init__(**args)
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
                [[0, 0, 1], [w, 0, 1], [w, h, 1]],
                texcoords=[[0, 0], [1, 0], [1, 1]],
            )
            self.add_triangle(
                [[0, 0, 1], [w, h, 1], [0, h, 1]],
                texcoords=[[0, 0], [1, 1], [0, 1]],
            )
            self._init = True


class Text(Rectangle):
    """Text object.

    This is an instance of `payton.scene.gui.Rectangle` and uses Pillow
    library to generate a texture with a text and assigns the texture
    to own material
    """

    def __init__(self, **args: Any) -> None:
        """Initialize Text

        Args:
          label: Label of the text. (Text to be written)
          bgcolor: Background color of the text (Default, r:0, g:0, b:0, a:0)
          color: Color of the text (Default, r:0, g:0, b:0)
        """
        super().__init__(**args)
        self.__label: str = args.get("label", "lorem")
        self.bgcolor: Tuple[int, ...] = args.get("bgcolor", (0, 0, 0, 0))
        self.color: Tuple[int, ...] = args.get("color", (0, 0, 0))
        self.bgcolor = tuple(map(lambda x: int(x * 255), self.bgcolor))
        self.color = tuple(map(lambda x: int(x * 255), self.color))
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
        proj: np.ndarray,
        view: np.ndarray,
        lights: List[Light],
        parent_matrix: Optional[np.ndarray] = None,
    ) -> None:
        """Render the Text

        This calls the super render of `payton.scene.geometry.Object.render`
        then renders the text on top of the rectangle.
        """
        super().render(proj, view, lights, parent_matrix)
        if not self._init_text:
            self.draw_text()
            self._init_text = True

    def draw_text(self) -> None:
        """Draw text

        Create an empty transparent image with the rectangle size
        and draw the text on it. Then, assign the image to the material
        """
        img = Image.new("RGBA", self.size, color=self.bgcolor)
        d = ImageDraw.Draw(img)
        if self.font is not None:
            d.text((5, 5), self.label, fill=self.color, font=self.font)
        else:
            d.text((5, 5), self.label, fill=self.color)

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

    def __init__(self, **args: Any) -> None:
        """Initialize HUD

        Args:
          width: Window width (overriden by Scene on add_object)
          height: Window height (overriden by Scene on add_object)
          font: Path to font file (See:
            https://pillow.readthedocs.io/en/3.1.x/reference/ImageFont.html)
          font_size: Font size
        """

        super().__init__(**args)
        self.width: int = args.get("width", 800)
        self.height: int = args.get("height", 600)
        self._fontname: str = args.get("font", "")
        self.children: Dict[str, Shape2D] = {}  # type: ignore
        self._font_size: int = args.get("font_size", 15)
        if self._fontname != "":
            self.set_font(self._fontname, self._font_size)
        self._font: ImageFont = None
        self._projection_matrix: Optional[np.ndarray] = None

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
        res = super().add_child(name, obj)  # type: ignore
        if not res:
            return res
        obj.parent = self
        return res

    def render(self, *_args: Any) -> None:
        """Render HUD

        Disables depth test and renders child primitives
        """
        glDisable(GL_DEPTH_TEST)
        if self._projection_matrix is None:
            self._projection_matrix = ortho(0, self.width, self.height, 0)

        for child in self.children:
            self.children[child].render(self._projection_matrix, None, [])
        glEnable(GL_DEPTH_TEST)

    def set_size(self, w: int, h: int):
        """Set size of the HUD view

        Args:
          w: Width of the viewport
          h: Height of the viewport
        """
        self.width = w
        self.height = h

    def set_font(self, font_name: str, font_size: int = 15) -> None:
        """Set font of the HUD

        Args:
          font_name: Path to the font file. TrueType or OpenType fonts are
            also supported.
          font_size: Font size.
        """
        self._font = ImageFont.truetype(font_name, font_size)
