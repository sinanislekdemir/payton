from enum import Enum
from typing import Any, Callable, Dict, Optional, Tuple

import numpy as np

from payton.scene.gui.base import Shape2D, Text
from payton.scene.shader import Shader


class Theme:
    def __init__(self):
        self.opacity = 0.9

        self.background_color: Tuple[float, float, float] = [0.05, 0.05, 0.05]
        self.text_color: Tuple[float, float, float] = [1.0, 1.0, 1.0]
        self.title_background_color: Tuple[float, float, float] = [
            235 / 255,
            210 / 255,
            52 / 255,
        ]
        self.title_text_color: Tuple[float, float, float] = [0.0, 0.0, 0.0]

    def secondary(self):
        self.background_color, self.text_color = self.title_background_color, self.title_text_color
        self.title_background_color, self.title_text_color = self.background_color, self.text_color


class WindowAlignment(Enum):
    FREE = "free"
    LEFT = "left"
    TOP = "top"
    RIGHT = "right"
    BOTTOM = "bottom"


class WindowElement(Shape2D):
    def __init__(
        self,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Optional[Theme] = None,
        **kwargs,
    ):
        kwargs["position"] = (left, top, 0)
        kwargs["size"] = (width, height)
        super().__init__(**kwargs)
        self._init: bool = False
        self.align = align
        self.theme = theme if theme is not None else Theme()
        self.material.opacity = self.theme.opacity

    def _reposition(self):
        if self.align == WindowAlignment.LEFT:
            self.position = [0.0, 0.0, 0.0]
            self.size = (self.size[0], self._parent_height)
        elif self.align == WindowAlignment.RIGHT:
            self.position = [self._parent_width - self.size[0], 0.0, 0.0]
            self.size = (self.size[0], self._parent_height)
        elif self.align == WindowAlignment.TOP:
            self.position = [0.0, 0.0, 0.0]
            self.size = (self._parent_width, self.size[1])
        elif self.align == WindowAlignment.BOTTOM:
            pos = [0, self._parent_height - self.size[1], 0]
            self.position = pos
            self.size = (self._parent_width, self.size[1])

    def draw(self):
        self._reposition()

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None,
    ) -> None:
        if not self._init:
            self.draw()

        super().render(lit, shader, parent_matrix)

    def add_child(self, name, obj: Shape2D) -> bool:  # type: ignore
        if obj.position[0] > self.size[0]:
            obj.position[0] = self.size[0] - 1
        total_v = obj.position[0] + obj.size[0]
        exceed = int(total_v - self.size[0])
        if exceed > 0:
            obj.size = (obj.size[0] - exceed, obj.size[1])

        if obj.position[1] > self.size[1]:
            obj.position[1] = self.size[1] - 1
        total_v = obj.position[1] + obj.size[1]
        exceed = int(total_v - self.size[1])
        if exceed > 0:
            obj.size = (obj.size[0], obj.size[1] - exceed)

        super().add_child(name, obj)
        obj.set_parent_size(self.size[0], self.size[1])


class Window(WindowElement):
    def __init__(
        self,
        title: str = "",
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Optional[Theme] = None,
        **kwargs: Dict[str, Any],
    ):
        """Initialize Window

        @NOTE: I am aware that I could have just define "title" as other
               elemenets are already arguments of WindowElement but some
               code hinting and completion tools are a bit stupid. They
               can not inherit from parent class.
        """
        super().__init__(
            width=width, height=height, left=left, top=top, align=align, theme=theme, **kwargs,
        )
        self.title = title
        self.add_child(
            "title",
            Text(
                position=(0, 0, 0),
                size=(self.size[0], 20),
                label=self.title,
                # bgcolor=self.theme.title_background_color,
                color=self.theme.title_text_color,
                opacity=0.5,
            ),
        )

    def draw(self):
        super().draw()
        w, h = self.size[0], self.size[1]
        self.clear_triangles()
        if not self._init:
            self.add_triangle(
                [[0, 22, 1], [w, h, 1], [w, 22, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[self.theme.background_color, self.theme.background_color, self.theme.background_color],
            )
            self.add_triangle(
                [[0, 22, 1], [0, h, 1], [w, h, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[self.theme.background_color, self.theme.background_color, self.theme.background_color],
            )
            self.add_triangle(
                [[0, 0, 1], [w - 1, 22, 1], [w - 1, 0, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self.add_triangle(
                [[0, 0, 1], [0, 22, 1], [w - 1, 22, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self._init = True


class Panel(WindowElement):
    def draw(self):
        super().draw()
        w, h = self.size[0], self.size[1]
        self.clear_triangles()
        if not self._init:
            self.add_triangle(
                [[0, 0, 1], [w, h, 1], [w, 0, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self.add_triangle(
                [[0, 0, 1], [0, h, 1], [w, h, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self.add_triangle(
                [[1, 1, 1], [w - 1, h - 1, 1], [w - 1, 1, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[self.theme.background_color, self.theme.background_color, self.theme.background_color],
            )
            self.add_triangle(
                [[1, 1, 1], [1, h - 1, 1], [w - 1, h - 1, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[self.theme.background_color, self.theme.background_color, self.theme.background_color],
            )

            self._init = True


class Button(Panel):
    def __init__(
        self,
        label: str,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Optional[Theme] = None,
        on_click: Optional[Callable] = None,
        **kwargs: Any,
    ):
        kwargs["on_click"] = on_click
        super().__init__(
            width=width, height=height, left=left, top=top, align=align, theme=theme, **kwargs,
        )
        if height < 30:
            height = 30

        self._label = label
        self.text = Text(position=(0, 0, 1), size=(10, 10), label=label, color=self.theme.text_color,)
        self.add_child("label", self.text)

    def draw(self, **kwargs):
        super().draw()
        text_size = self.text.text_size
        x = (self.size[0] - text_size[0]) / 2
        y = (self.size[1] - text_size[1]) / 2
        if x < 0:
            x = 0
        if y < 0:
            y = 0
        y -= 4
        self.text.label = self._label
        self.text.position = [x, y]
        crop = [0, 0, text_size[0], text_size[1] + 4]
        if text_size[0] > self.size[0]:
            crop[0] = (text_size[0] - self.size[0]) / 2.0
            crop[2] = text_size[0] - crop[0]
            diff = int(crop[2] - crop[0])
            text_size = (diff, text_size[1])
        if text_size[1] > self.size[1]:
            crop[1] = (text_size[1] - self.size[1]) / 2.0
            crop[3] = text_size[1] - crop[1]
            text_size = (text_size[0], self.size[1])
        self.text.crop = crop
        self.text.size = (text_size[0], text_size[1] + 4)

    @property
    def label(self) -> str:
        return self._label

    @label.setter
    def label(self, text: str):
        self._label = text
        self.text.label = self._label


class EditBox(Panel):
    def __init__(
        self,
        value: str,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Optional[Theme] = None,
        on_change: Optional[Callable] = None,
        multiline: bool = False,
        **kwargs: Any,
    ):
        kwargs["on_click"] = self._focus
        if height < 30:
            height = 30

        super().__init__(
            width=width, height=height, left=left, top=top, align=align, theme=theme, **kwargs,
        )

        self.multiline = multiline
        self.theme.secondary()
        self.on_change = on_change
        self._value = value
        self.text = Text(position=(0, 0, 1), size=(width, height), label=self._value, color=self.theme.text_color,)
        self.add_child("label", self.text)
        self._cursor = -1

    def _on_keypress(self, instr: str):
        new = self.value[0 : self._cursor] + instr + self.value[self._cursor :]
        self._cursor = len(self.value[0 : self._cursor] + instr)
        self.value = new

    def _focus(self):
        self._cursor = len(self.value)
        self._init = False
        # Dummy holder to pass on_click test
        pass

    def cursor_left(self):
        self._cursor -= 1
        if self._cursor < 0:
            self._cursor = 0
        self._init = False

    def cursor_right(self):
        self._cursor += 1
        if self._cursor > len(self.value):
            self._cursor = len(self.value)
        self._init = False

    def backspace(self):
        if self._cursor == 0:
            return
        self.value = self.value[0 : self._cursor - 1] + self.value[self._cursor :]
        self.cursor_left()

    def _exit(self):
        self._cursor = -1
        if callable(self.on_change):
            self.on_change(self._value)
        self._init = False

    def draw(self, **kwargs):
        super().draw()
        if self._cursor > -1:
            label = self._value[0 : self._cursor] + "|" + self._value[self._cursor :]
        else:
            label = self._value
        self.text.label = label
        if self.multiline:
            self.text.wrap(self.size[0])
        text_size = list(self.text.text_size)
        x = 1
        y = (self.size[1] - text_size[1]) / 2
        if y < 0:
            y = 0
        y += 4
        self.text.position = [x, y]
        crop = [0, 0, text_size[0], text_size[1] + 4]
        if text_size[0] > self.size[0]:
            crop[0] = text_size[0] - self.size[0]
            crop[2] = text_size[0]

        if text_size[1] > self.size[1]:
            crop[1] = text_size[1] - self.size[1]
            crop[3] = self.text.text_size[1]

        self.text.crop = crop
        self.text._init = False
        self.text._init_text = False

    @property
    def value(self) -> str:
        return self._value

    @value.setter
    def value(self, text: str):
        self._value = text
        self.text.label = self._value
        if self.text.text_size[0] > self.size[0] and self.multiline:
            self.text.wrap(self.size[0])
        self.refresh()
        self._init = False
