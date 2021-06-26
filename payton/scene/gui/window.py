"""Basic Window and UI Elements Support

This is a simple support but works without much hassle.
I tried to make the whole mechanism as basic as possible.
Anyone without any UI coding experience should be able to get started
with the basic stuff"""

from enum import Enum
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from payton.math.vector import Vector3D
from payton.scene.geometry.base import Object
from payton.scene.gui.base import Shape2D, Text
from payton.scene.shader import Shader


class Theme:
    """User interface theme"""

    def __init__(self) -> None:
        """Initialize the theme"""
        self.opacity = 0.9

        self.background_color: Vector3D = [0.05, 0.05, 0.05]
        self.text_color: Vector3D = [1.0, 1.0, 1.0]
        self.title_background_color: Vector3D = [
            235 / 255,
            210 / 255,
            52 / 255,
        ]
        self.title_text_color: Vector3D = [0.0, 0.0, 0.0]

    def secondary(self) -> None:
        """Secondary colors for the theme"""
        self.background_color, self.text_color = self.title_background_color, self.title_text_color
        self.title_background_color, self.title_text_color = self.background_color, self.text_color


class WindowAlignment(Enum):
    """Window alignment on the HUD"""

    FREE = "free"
    LEFT = "left"
    TOP = "top"
    RIGHT = "right"
    BOTTOM = "bottom"


class WindowElement(Shape2D):
    """Window Base Element"""

    def __init__(
        self,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Optional[Theme] = None,
        **kwargs: Any,
    ) -> None:
        """Initialize the Window Element

        Keyword arguments:
        width -- Width of the window element
        height -- Height of the window element
        left -- Left position of the window element
        top -- Top position of the window element
        align -- Alignment of the element
        theme -- Theme to be used
        """
        kwargs["position"] = (left, top, 0)
        kwargs["size"] = (width, height)
        super().__init__(**kwargs)
        self._init: bool = False
        self.align = align
        self.theme = theme if theme is not None else Theme()
        self.material.opacity = self.theme.opacity

    def _reposition(self) -> None:
        if self.align == WindowAlignment.LEFT:
            self.position = [0.0, 0.0, 0.0]
            self.size = [self.size[0], self._parent_height]
        elif self.align == WindowAlignment.RIGHT:
            self.position = [self._parent_width - self.size[0], 0.0, 0.0]
            self.size = [self.size[0], self._parent_height]
        elif self.align == WindowAlignment.TOP:
            self.position = [0.0, 0.0, 0.0]
            self.size = [self._parent_width, self.size[1]]
        elif self.align == WindowAlignment.BOTTOM:
            self.position = [0, self._parent_height - self.size[1], 0]
            self.size = [self._parent_width, self.size[1]]

    def draw(self) -> None:
        self._reposition()

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: Optional[np.ndarray] = None,
        _primitive: int = None,
    ) -> None:
        if not self._init:
            self.draw()

        super().render(lit, shader, parent_matrix)

    def add_child(self, name: str, obj: Object) -> bool:
        """Add a window element as a child

        Keyword arguments:
        name -- Name of the window element to be added
        obj -- Object to add
        """
        if not isinstance(obj, Shape2D):
            return False
        if obj.position[0] > self.size[0]:
            obj.position[0] = self.size[0] - 1
        total_v = obj.position[0] + obj.size[0]
        exceed = int(total_v - self.size[0])
        if exceed > 0:
            obj.size = [obj.size[0] - exceed, obj.size[1]]

        if obj.position[1] > self.size[1]:
            obj.position[1] = self.size[1] - 1
        total_v = obj.position[1] + obj.size[1]
        exceed = int(total_v - self.size[1])
        if exceed > 0:
            obj.size = [obj.size[0], obj.size[1] - exceed]

        super().add_child(name, obj)
        obj._set_parent_size(self.size[0], self.size[1])
        return True


class Window(WindowElement):
    """Main window frame"""

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
        """Initialize Window Frame

        @NOTE: I am aware that I could have just define "title" as other
               elemenets are already arguments of WindowElement but some
               code hinting and completion tools are a bit stupid. They
               can not inherit from parent class. So this is a ide-candy

        Keyword arguments:
        title -- Title of the window
        width -- Width of the window
        height -- Height of the window
        left -- Left position of the window
        top -- Top position of the window
        align -- Alignment of the
        theme -- Theme to be used
        """
        super().__init__(
            width=width,
            height=height,
            left=left,
            top=top,
            align=align,
            theme=theme,
            **kwargs,
        )
        self.title = title
        self.add_child(
            "title",
            Text(
                position=[0, 0, 0],
                size=[self.size[0], 20],
                label=self.title,
                # bgcolor=self.theme.title_background_color,
                color=self.theme.title_text_color,
                opacity=0.5,
            ),
        )

    def draw(self) -> None:
        """Create the frame polygons"""
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
    """Panel element"""

    def draw(self) -> None:
        """Create the panel polygons"""
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
        """Initialize the Button element

        label -- Label of the button
        width -- Width of the button
        height -- Height of the button
        left -- Left position of the button
        top -- Top position of the button
        align -- Alignment of the button
        theme -- Theme to be used
        on_click -- On Click callback method
        """
        kwargs["on_click"] = on_click
        super().__init__(
            width=width,
            height=height,
            left=left,
            top=top,
            align=align,
            theme=theme,
            **kwargs,
        )
        height = max(height, 30)

        self._label = label
        self.text = Text(
            position=[0, 0, 1],
            size=[10, 10],
            label=label,
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)

    def draw(self, **kwargs: Any) -> None:
        """Create the button polygons"""
        super().draw()
        text_size = self.text.text_size
        x = (self.size[0] - text_size[0]) / 2
        y = (self.size[1] - text_size[1]) / 2
        x = max(x, 0)
        y = max(y, 0)
        y -= 4
        self.text.label = self._label
        self.text.position = [x, y]
        crop: List[int] = [0, 0, text_size[0], text_size[1] + 4]
        if text_size[0] > self.size[0]:
            crop[0] = int((text_size[0] - self.size[0]) / 2.0)
            crop[2] = int(text_size[0] - crop[0])
            diff = int(crop[2] - crop[0])
            text_size = (diff, text_size[1])
        if text_size[1] > self.size[1]:
            crop[1] = int((text_size[1] - self.size[1]) / 2.0)
            crop[3] = int(text_size[1] - crop[1])
            text_size = (text_size[0], self.size[1])
        self.text.crop = crop
        self.text.size = [int(text_size[0]), int(text_size[1] + 4)]

    @property
    def label(self) -> str:
        """Get the button label"""
        return self._label

    @label.setter
    def label(self, text: str) -> None:
        """Set the button label

        Keyword arguments:
        text -- Text of the label
        """
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
        """Initialize the edit box element

        value -- Default value of the editbox
        width -- Width of the editbox
        height -- Height of the editbox
        left -- Left position of the editbox
        top -- Top position of the editbox
        align -- Alignment of the editbox
        theme -- Theme to be used
        on_change -- On change callback method (expects a string argument) `def on_change(text: str)`
        """
        kwargs["on_click"] = self._focus
        height = max(height, 30)

        super().__init__(
            width=width,
            height=height,
            left=left,
            top=top,
            align=align,
            theme=theme,
            **kwargs,
        )

        self.multiline = multiline
        self.theme.secondary()
        self.on_change = on_change
        self._value = value
        self.text = Text(
            position=[0, 0, 0],
            size=[width, height],
            label=self._value,
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)
        self._cursor = -1

    def _on_keypress(self, instr: str) -> None:
        new = self.value[0 : self._cursor] + instr + self.value[self._cursor :]
        self._cursor = len(self.value[0 : self._cursor] + instr)
        self.value = new

    def _focus(self) -> None:
        self._cursor = len(self.value)
        self._init = False

    def cursor_left(self) -> None:
        """Move the cursor to the left"""
        self._cursor -= 1
        self._cursor = max(self._cursor, 0)
        self._init = False

    def cursor_right(self) -> None:
        """Move the cursor to the right"""
        self._cursor += 1
        self._cursor = min(self._cursor, len(self.value))
        self._init = False

    def backspace(self) -> None:
        """Delete the previous character"""
        if self._cursor == 0:
            return
        self.value = self.value[0 : self._cursor - 1] + self.value[self._cursor :]
        self.cursor_left()

    def _exit(self) -> None:
        self._cursor = -1
        if callable(self.on_change):
            self.on_change(self._value)
        self._init = False

    def draw(self, **kwargs: Any) -> None:
        """Create the editbox polygons and texture along with cropping"""
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
        y = max(y, 0)
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
        """Return the editbox value as a string"""
        return self._value

    @value.setter
    def value(self, text: str) -> None:
        """Set the editbox value

        Keyword arguments:
        text -- Text value to set
        """
        self._value = text
        self.text.label = self._value
        if self.text.text_size[0] > self.size[0] and self.multiline:
            self.text.wrap(self.size[0])
        self.refresh()
        self._init = False
