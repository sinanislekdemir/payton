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
    def __init__(
        self,
        position: Tuple[int, int, int],
        size: Tuple[int, int],
        on_click: Optional[Callable] = None,
        opacity: float = 0.1,
        **kwargs: Any,
    ):
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
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ):
        super().render(False, shader, parent_matrix)

    @property
    def font(self) -> None:
        if self._font is not None:
            return self._font
        if self.parent is not None:
            return self.parent.font
        return None

    @font.setter
    def font(self, font: ImageFont) -> None:
        self._font = font
        self.draw_text()

    def set_parent_size(self, w: int, h: int):
        self._parent_width = w
        self._parent_height = h
        self._init = False
        self.draw()
        for child in self.children.values():
            cast("Shape2D", child).set_parent_size(self.size[0], self.size[1])

    def click(self, x: int, y: int) -> Optional[Mesh]:
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
        self, position: Tuple[int, int, int], size: Tuple[int, int], **kwargs: Any,
    ):
        super().__init__(position=position, size=size, **kwargs)
        self._init: bool = False
        self.draw()

    def draw(self) -> None:
        w, h = self.size
        self.clear_triangles()
        if not self._init:
            self.add_triangle(
                [[0, 0, 1], [w, h, 1], [w, 0, 1]], texcoords=[[0, 0], [1, 1], [1, 0]],
            )
            self.add_triangle(
                [[0, 0, 1], [0, h, 1], [w, h, 1]], texcoords=[[0, 0], [0, 1], [1, 1]],
            )
            self._init = True


class Text(Rectangle):
    def __init__(
        self,
        position: Tuple[int, int, int],
        size: Tuple[int, int],
        label: str = "lorem",
        bgcolor: Optional[Tuple[float, float, float]] = None,
        color: Optional[Tuple[float, float, float]] = None,
        **kwargs: Any,
    ) -> None:
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

        self.crop = [0, 0, 0, 0]
        self._init_text: bool = False

    @property
    def label(self) -> str:
        return self.__label

    @label.setter
    def label(self, label: str) -> None:
        self.__label = label
        self._init_text = False

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ) -> None:
        if not self._init_text:
            self.draw_text()
        super().render(lit, shader, parent_matrix)

    @property
    def text_size(self) -> Tuple[int, int]:
        res = (0, 0)
        timg = Image.new("RGBA", (1, 1))
        d = ImageDraw.Draw(timg)

        res = d.textsize(self.label, font=self.font)
        lres = list(res)
        if lres[0] < self.size[0]:
            lres[0] = self.size[0]
        if lres[1] < self.size[1]:
            lres[1] = self.size[1]
        return lres[0], lres[1]

    def wrap(self, width_in_pixels: int):
        dummy = ""
        original = self.label
        self.label = ""
        for char in original:
            dummy += char
            res = (0, 0)
            timg = Image.new("RGBA", (1, 1))
            d = ImageDraw.Draw(timg)
            res = d.textsize(dummy, font=self.font)
            if res[0] < width_in_pixels:
                self.label += char
            else:
                self.label += "\n" + char
                dummy = ""
        self._init_text = False

    def draw_text(self) -> None:
        if self._init_text:
            return
        img = Image.new("RGBA", self.text_size, color=tuple(self.bgcolor))
        d = ImageDraw.Draw(img)

        if self.font is not None:
            d.text((1, 1), self.label, fill=tuple(self.color), font=self.font)
        else:
            d.text((1, 1), self.label, fill=tuple(self.color))

        if any(self.crop):
            img = img.crop(self.crop)

        self.material._image = img
        self.material.refresh()
        self._init_text = True


class Hud(Object):
    def __init__(
        self, width: int = 800, height: int = 600, font: str = "", font_size: int = 15, **kwargs: Any,
    ) -> None:
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
            self.set_font(os.path.join(os.path.dirname(os.path.abspath(__file__)), "monofonto.ttf"))
        except OSError:
            # font not found but pillow has better than nothinf font
            pass

    @property
    def font(self) -> ImageFont:
        return self._font

    def add_child(self, name: str, obj: Shape2D) -> bool:  # type: ignore
        res = super().add_child(name, obj)
        obj._scene_height = self.height
        obj._scene_width = self.width

        if not res:
            return res
        obj.parent = self
        return res

    def render(self, lit: bool, shader: Shader, *_args: Any) -> None:
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
        self.width = w
        self.height = h
        for child in self.children.values():
            cast(Shape2D, child).set_parent_size(w, h)
            child._init = False
            child.draw()

        self._projection_matrix = None

    def set_font(self, font_name: str, font_size: int = 16) -> None:
        self._font = ImageFont.truetype(font_name, font_size)
