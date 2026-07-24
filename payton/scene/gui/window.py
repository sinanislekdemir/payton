"""Basic Window and UI Elements Support

This is a simple support but works without much hassle.
I tried to make the whole mechanism as basic as possible.
Anyone without any UI coding experience should be able to get started
with the basic stuff"""

import logging
import time
from collections.abc import Callable
from enum import Enum
from typing import Any, cast

import numpy as np
from PIL import Image, ImageDraw

from payton.math.vector import Vector3D
from payton.scene.geometry.base import Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.gui.base import Rectangle, Shape2D, Text, text_size
from payton.scene.shader import Shader
from payton.scene.theme import SceneTheme

logger = logging.getLogger(__name__)

# Height in pixels of the window title bar
TITLE_BAR_HEIGHT: int = 28


class Theme:
    """User interface theme"""

    def __init__(self) -> None:
        """Initialize the theme with a modern dark-editor palette."""
        self.opacity: float = 0.9

        self.background_color: Vector3D = [0.12, 0.12, 0.14]
        self.text_color: Vector3D = [0.95, 0.95, 0.95]
        self.title_background_color: Vector3D = [0.20, 0.45, 0.75]
        self.title_text_color: Vector3D = [1.0, 1.0, 1.0]
        self.border_color: Vector3D = [0.08, 0.08, 0.10]
        self.highlight_color: Vector3D = [0.35, 0.60, 0.90]

    def secondary(self) -> None:
        """Swap background/title palettes (e.g. for EditBox active state)."""
        # Simultaneous swap — no temporary variable needed in Python
        self.background_color, self.title_background_color = (
            self.title_background_color,
            self.background_color,
        )
        self.text_color, self.title_text_color = (
            self.title_text_color,
            self.text_color,
        )

    @classmethod
    def from_scene_theme(cls, scene_theme: SceneTheme) -> "Theme":
        """Derive a UI Theme whose palette is coherent with a SceneTheme.

        The window background is drawn from the scene's bottom background
        colour (darkened slightly).  The title bar takes the scene's light
        colour (desaturated to a cool tint).  Text colours are chosen for
        maximum contrast.

        Keyword arguments:
        scene_theme -- A :class:`~payton.scene.theme.SceneTheme` instance.
        """
        t = cls()

        # -- Background: darken the bottom background colour
        bb = scene_theme.background_bottom_color
        t.background_color = [
            max(bb[0] * 0.7, 0.06),
            max(bb[1] * 0.7, 0.06),
            max(bb[2] * 0.7, 0.08),
        ]
        t.border_color = [
            max(t.background_color[0] - 0.04, 0.0),
            max(t.background_color[1] - 0.04, 0.0),
            max(t.background_color[2] - 0.04, 0.0),
        ]

        # -- Title bar: derived from scene light colour with a blue tint
        lc = scene_theme.light_color
        title = [
            lc[0] * 0.25 + 0.10,
            lc[1] * 0.25 + 0.25,
            lc[2] * 0.40 + 0.35,
        ]
        t.title_background_color = [min(c, 1.0) for c in title]
        t.highlight_color = [min(c + 0.15, 1.0) for c in t.title_background_color]

        # -- Text contrast: pick white or black based on background luminance
        bg_lum = (
            0.2126 * t.background_color[0]
            + 0.7152 * t.background_color[1]
            + 0.0722 * t.background_color[2]
        )
        t.text_color = [0.95, 0.95, 0.95] if bg_lum < 0.5 else [0.05, 0.05, 0.05]

        tb_lum = (
            0.2126 * t.title_background_color[0]
            + 0.7152 * t.title_background_color[1]
            + 0.0722 * t.title_background_color[2]
        )
        t.title_text_color = [1.0, 1.0, 1.0] if tb_lum < 0.5 else [0.05, 0.05, 0.05]

        return t


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
        theme: Theme | None = None,
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
        parent_matrix: np.ndarray | None = None,
        _primitive: int | None = None,
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
        theme: Theme | None = None,
        **kwargs: dict[str, Any],
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
                position=[10, 0, 0],
                size=[self.size[0], TITLE_BAR_HEIGHT],
                label=self.title,
                color=self.theme.title_text_color,
                opacity=0.8,
            ),
        )
        self._dragging: bool = False
        self._drag_offset_x: float = 0.0
        self._drag_offset_y: float = 0.0

    def start_drag(self, x: int, y: int) -> None:
        self._dragging = True
        self.align = WindowAlignment.FREE
        self._drag_offset_x = x - self.position[0]
        self._drag_offset_y = y - self.position[1]

    def drag_to(self, x: int, y: int) -> None:
        if not self._dragging:
            return
        self.position = [
            x - self._drag_offset_x,
            y - self._drag_offset_y,
            self.position[2],
        ]

    def stop_drag(self) -> None:
        self._dragging = False

    def click(self, x: int, y: int) -> Mesh | None:
        if self._model_matrix is None or len(self._model_matrix) == 0:
            return None
        mm = self._model_matrix[3]
        inside = (
            x > mm[0]
            and x < mm[0] + self.size[0]
            and y > mm[1]
            and y < mm[1] + self.size[1]
        )
        if not inside:
            return None
        if y - mm[1] < TITLE_BAR_HEIGHT:
            self.start_drag(x, y)
            return self
        for child in self.children:
            c = cast(Mesh, self.children[child]).click(x, y)
            if c:
                return c
        return None

    def draw(self) -> None:
        """Create the frame polygons"""
        super().draw()
        w, h = self.size[0], self.size[1]
        tb = TITLE_BAR_HEIGHT
        self.clear_triangles()
        if not self._init:
            # Content area
            self.add_triangle(
                [[0, tb, 1], [w, h, 1], [w, tb, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.background_color,
                    self.theme.background_color,
                    self.theme.background_color,
                ],
            )
            self.add_triangle(
                [[0, tb, 1], [0, h, 1], [w, h, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.background_color,
                    self.theme.background_color,
                    self.theme.background_color,
                ],
            )
            # Title bar
            self.add_triangle(
                [[0, 0, 1], [w - 1, tb, 1], [w - 1, 0, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self.add_triangle(
                [[0, 0, 1], [0, tb, 1], [w - 1, tb, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            # Separator line between title bar and content area
            sep_y = tb
            sep_bottom = tb + 2
            self.add_triangle(
                [[0, sep_y, 1], [w, sep_bottom, 1], [w, sep_y, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
                ],
            )
            self.add_triangle(
                [[0, sep_y, 1], [0, sep_bottom, 1], [w, sep_bottom, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
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
            # Outer border (2 px)
            self.add_triangle(
                [[0, 0, 1], [w, h, 1], [w, 0, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
                ],
            )
            self.add_triangle(
                [[0, 0, 1], [0, h, 1], [w, h, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
                ],
            )
            # Inner fill (inset 2 px)
            self.add_triangle(
                [[2, 2, 1], [w - 2, h - 2, 1], [w - 2, 2, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.background_color,
                    self.theme.background_color,
                    self.theme.background_color,
                ],
            )
            self.add_triangle(
                [[2, 2, 1], [2, h - 2, 1], [w - 2, h - 2, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.background_color,
                    self.theme.background_color,
                    self.theme.background_color,
                ],
            )
            # Top-edge highlight stripe (1 px bevel effect)
            self.add_triangle(
                [[2, 2, 1], [w - 2, 3, 1], [w - 2, 2, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                ],
            )
            self.add_triangle(
                [[2, 2, 1], [2, 3, 1], [w - 2, 3, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                ],
            )

            self._init = True


class ProgressBar(Panel):
    def __init__(
        self,
        label: str = "",
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Theme | None = None,
        value: float = 0.0,
        min_value: float = 0.0,
        max_value: float = 100.0,
        **kwargs: Any,
    ):
        kwargs["on_click"] = None
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

        self._label = label
        self._min_value = min_value
        self._max_value = max_value
        self._value = max(min_value, min(max_value, value))
        self._bar_init: bool = False

        self.text = Text(
            position=[0, 0, 1],
            size=[10, 10],
            label=self._make_label(),
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)

    def _make_label(self) -> str:
        pct = (
            (self._value - self._min_value)
            / max(self._max_value - self._min_value, 1.0)
            * 100.0
        )
        if self._label:
            return f"{self._label}: {pct:.0f}%"
        return f"{pct:.0f}%"

    def _update_label(self) -> None:
        self.text.label = self._make_label()
        self.text._init_text = False

    @property
    def value(self) -> float:
        return self._value

    @value.setter
    def value(self, val: float) -> None:
        clamped = max(self._min_value, min(self._max_value, val))
        if clamped == self._value:
            return
        self._value = clamped
        self._update_label()
        self._init = False
        self._bar_init = False

    @property
    def min_value(self) -> float:
        return self._min_value

    @min_value.setter
    def min_value(self, val: float) -> None:
        self._min_value = val
        self._update_label()
        self._init = False
        self._bar_init = False

    @property
    def max_value(self) -> float:
        return self._max_value

    @max_value.setter
    def max_value(self, val: float) -> None:
        self._max_value = val
        self._update_label()
        self._init = False
        self._bar_init = False

    def draw(self, **kwargs: Any) -> None:
        super().draw()
        w, h = self.size[0], self.size[1]
        if not self._bar_init:
            ratio = (self._value - self._min_value) / max(
                self._max_value - self._min_value, 1.0
            )
            ratio = max(0.0, min(1.0, ratio))
            bar_w = (w - 4) * ratio
            if bar_w > 0:
                self.add_triangle(
                    [[2, 2, 1], [2 + bar_w, h - 2, 1], [2 + bar_w, 2, 1]],
                    texcoords=[[0, 0], [1, 1], [1, 0]],
                    colors=[
                        self.theme.title_background_color,
                        self.theme.title_background_color,
                        self.theme.title_background_color,
                    ],
                )
                self.add_triangle(
                    [[2, 2, 1], [2, h - 2, 1], [2 + bar_w, h - 2, 1]],
                    texcoords=[[0, 0], [0, 1], [1, 1]],
                    colors=[
                        self.theme.title_background_color,
                        self.theme.title_background_color,
                        self.theme.title_background_color,
                    ],
                )
                # Bar top-edge highlight
                if bar_w > 4:
                    self.add_triangle(
                        [[4, 4, 1], [2 + bar_w - 2, 5, 1], [2 + bar_w - 2, 4, 1]],
                        texcoords=[[0, 0], [1, 1], [1, 0]],
                        colors=[
                            self.theme.highlight_color,
                            self.theme.highlight_color,
                            self.theme.highlight_color,
                        ],
                    )
                    self.add_triangle(
                        [[4, 4, 1], [4, 5, 1], [2 + bar_w - 2, 5, 1]],
                        texcoords=[[0, 0], [0, 1], [1, 1]],
                        colors=[
                            self.theme.highlight_color,
                            self.theme.highlight_color,
                            self.theme.highlight_color,
                        ],
                    )
            self._bar_init = True


class Slider(Panel):
    TRACK_HEIGHT = 8
    THUMB_WIDTH = 16
    THUMB_HEIGHT = 22

    def __init__(
        self,
        label: str = "",
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Theme | None = None,
        on_change: Callable | None = None,
        min_value: float = 0.0,
        max_value: float = 100.0,
        value: float | None = None,
        **kwargs: Any,
    ):
        kwargs["on_click"] = self._start_slide
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

        self._label = label
        self._raw_on_change = on_change
        self.on_change_call = self._safe_on_change
        self._min_value = min_value
        self._max_value = max_value
        if value is None:
            value = min_value
        self._value = max(min_value, min(max_value, value))
        self._dragging = False
        self._drag_offset_x: float = 0.0
        self._slider_init: bool = False

        self.text = Text(
            position=[0, 0, 1],
            size=[10, 10],
            label=self._make_label(),
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)

    def _make_label(self) -> str:
        if self._label:
            return f"{self._label}: {self._value:.2f}"
        return f"{self._value:.2f}"

    def _update_label(self) -> None:
        self.text.label = self._make_label()
        self.text._init_text = False

    def _safe_on_change(self, val: float) -> None:
        if callable(self._raw_on_change):
            try:
                self._raw_on_change(val)
            except Exception:
                logger.exception("Slider on_change callback failed")

    @property
    def value(self) -> float:
        return self._value

    @value.setter
    def value(self, val: float) -> None:
        clamped = max(self._min_value, min(self._max_value, val))
        if clamped == self._value:
            return
        self._value = clamped
        self._update_label()
        self.on_change_call(self._value)
        self._init = False
        self._slider_init = False

    @property
    def min_value(self) -> float:
        return self._min_value

    @min_value.setter
    def min_value(self, val: float) -> None:
        self._min_value = val
        if self._value < val:
            self.value = val

    @property
    def max_value(self) -> float:
        return self._max_value

    @max_value.setter
    def max_value(self, val: float) -> None:
        self._max_value = val
        if self._value > val:
            self.value = val

    def _value_to_x(self) -> float:
        track_pad = self.THUMB_WIDTH / 2 + 2
        track_w = self.size[0] - track_pad * 2
        if track_w <= 0:
            return track_pad
        ratio = (self._value - self._min_value) / (self._max_value - self._min_value)
        return track_pad + ratio * track_w - self.THUMB_WIDTH / 2

    def _x_to_value(self, x: float) -> float:
        track_pad = self.THUMB_WIDTH / 2 + 2
        track_w = self.size[0] - track_pad * 2
        if track_w <= 0:
            return self._min_value
        ratio = max(0.0, min(1.0, (x - track_pad) / track_w))
        return self._min_value + ratio * (self._max_value - self._min_value)

    def _start_slide(self) -> None:
        self._dragging = True
        self._drag_offset_x = 0.0

    def drag_to(self, global_x: int, global_y: int) -> None:
        if (
            not self._dragging
            or self._model_matrix is None
            or len(self._model_matrix) == 0
        ):
            return
        mm = self._model_matrix[3]
        local_x = global_x - mm[0]
        self.value = self._x_to_value(local_x - self._drag_offset_x)

    def stop_drag(self) -> None:
        self._dragging = False

    def click(self, x: int, y: int) -> Mesh | None:
        if self._model_matrix is None or len(self._model_matrix) == 0:
            return None
        mm = self._model_matrix[3]
        inside_me = (
            x > mm[0]
            and x < mm[0] + self.size[0]
            and y > mm[1]
            and y < mm[1] + self.size[1]
        )
        if not inside_me:
            return None

        local_x = x - mm[0]
        thumb_x = self._value_to_x()
        on_thumb = local_x >= thumb_x and local_x <= thumb_x + self.THUMB_WIDTH

        if on_thumb:
            self._dragging = True
            self._drag_offset_x = local_x - (thumb_x + self.THUMB_WIDTH / 2)
        else:
            self.value = self._x_to_value(local_x)
            self._dragging = True
            self._drag_offset_x = 0.0

        self._init = False
        self._slider_init = False
        return self

    def draw(self, **kwargs: Any) -> None:
        super().draw()
        w, h = self.size[0], self.size[1]
        if not self._slider_init:
            # Track
            track_y = (h - self.TRACK_HEIGHT) / 2
            pad = self.THUMB_WIDTH / 2 + 2
            self.add_triangle(
                [
                    [pad, track_y, 1],
                    [w - pad, track_y + self.TRACK_HEIGHT, 1],
                    [w - pad, track_y, 1],
                ],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
                ],
            )
            self.add_triangle(
                [
                    [pad, track_y, 1],
                    [pad, track_y + self.TRACK_HEIGHT, 1],
                    [w - pad, track_y + self.TRACK_HEIGHT, 1],
                ],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.border_color,
                    self.theme.border_color,
                    self.theme.border_color,
                ],
            )
            # Thumb
            thumb_x = self._value_to_x()
            thumb_y = (h - self.THUMB_HEIGHT) / 2
            tx, ty, tw, th = thumb_x, thumb_y, self.THUMB_WIDTH, self.THUMB_HEIGHT
            self.add_triangle(
                [[tx, ty, 1], [tx + tw, ty + th, 1], [tx + tw, ty, 1]],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            self.add_triangle(
                [[tx, ty, 1], [tx, ty + th, 1], [tx + tw, ty + th, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                    self.theme.title_background_color,
                ],
            )
            # Thumb top-edge highlight
            self.add_triangle(
                [
                    [tx + 2, ty + 2, 1],
                    [tx + tw - 2, ty + 3, 1],
                    [tx + tw - 2, ty + 2, 1],
                ],
                texcoords=[[0, 0], [1, 1], [1, 0]],
                colors=[
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                ],
            )
            self.add_triangle(
                [[tx + 2, ty + 2, 1], [tx + 2, ty + 3, 1], [tx + tw - 2, ty + 3, 1]],
                texcoords=[[0, 0], [0, 1], [1, 1]],
                colors=[
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                    self.theme.highlight_color,
                ],
            )
            self._slider_init = True


class Button(Panel):
    def __init__(
        self,
        label: str,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Theme | None = None,
        on_click: Callable | None = None,
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

        self._label = label
        self.text = Text(
            position=[0, 0, 1],
            size=[10, 10],
            label=label,
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)

    def draw(self, **kwargs: Any) -> None:
        """Create the button polygons and position the text label."""
        super().draw()

        if self.text.font is None:
            return

        # Use the full textbbox so we know *where* text pixels land in the
        # PIL image.  draw_text() renders at (1, 1), so actual text pixels
        # span x:(1+bbox[0])..(1+bbox[2])  y:(1+bbox[1])..(1+bbox[3]).
        # text_size() returns only bbox[2]-bbox[0] × bbox[3]-bbox[1], which
        # is too small: the bottom bbox[1]+1 rows are always clipped.
        timg = Image.new("RGBA", (1, 1))
        d = ImageDraw.Draw(timg)
        _bbox = d.textbbox((0, 0), self._label, font=self.text.font)
        del d, timg
        bbox = (int(_bbox[0]), int(_bbox[1]), int(_bbox[2]), int(_bbox[3]))

        if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
            return  # empty / zero-size text

        # PIL image dimensions: must fit all text pixels drawn at (1, 1).
        img_w = bbox[2] + 2
        img_h = bbox[3] + 2

        # Centre of the rendered text region within the PIL image.
        cx = 1.0 + (bbox[0] + bbox[2]) / 2.0
        cy = 1.0 + (bbox[1] + bbox[3]) / 2.0

        # Position the quad so its text centre aligns with the button centre.
        xi = max(int(self.size[0] / 2.0 - cx), 0)
        yi = max(int(self.size[1] / 2.0 - cy), 0)

        self.text.label = self._label
        crop: list[int] = [0, 0, img_w, img_h]

        # If text image is wider than the button, centre-crop horizontally.
        if img_w > self.size[0]:
            overflow = img_w - int(self.size[0])
            crop = [overflow // 2, 0, img_w - overflow // 2, img_h]
            img_w = crop[2] - crop[0]
            xi = 0

        # If text image is taller than the button, centre-crop vertically.
        if img_h > self.size[1]:
            overflow = img_h - int(self.size[1])
            crop[1] = overflow // 2
            crop[3] = img_h - overflow // 2
            img_h = crop[3] - crop[1]
            yi = 0

        self.text.crop = crop
        self.text.position = [xi, yi]
        # Stable dimensions: derived from font metrics, not from text.size.
        self.text.size = [max(img_w, 1), max(img_h, 1)]

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
    BLINK_PERIOD = 0.53

    def __init__(
        self,
        value: str,
        width: int = 400,
        height: int = 300,
        left: int = 10,
        top: int = 10,
        align: WindowAlignment = WindowAlignment.FREE,
        theme: Theme | None = None,
        on_change: Callable | None = None,
        multiline: bool = False,
        placeholder: str = "",
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
        multiline -- If True, enables multi-line text editing
        placeholder -- Placeholder text shown when value is empty and unfocused
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
        self._raw_on_change = on_change
        self.on_change = self._safe_on_change
        self._value = value
        self._placeholder = placeholder
        self.text = Text(
            position=[0, 0, 0],
            size=[width, height],
            label=self._value,
            color=self.theme.text_color,
        )
        self.add_child("label", self.text)
        self._cursor = -1
        self._blink_on = True
        self._last_blink = time.time()
        self._scroll_x = 0
        self._scroll_y = 0
        self._cursor_rect = Rectangle([0, 0], [2, 10])
        self._cursor_rect.material.color = self.theme.highlight_color
        self._cursor_rect.material.opacity = 1.0
        self._cursor_rect.hide()
        self.add_child("_cursor", self._cursor_rect)

    def _safe_on_change(self, text: str) -> None:
        if callable(self._raw_on_change):
            try:
                self._raw_on_change(text)
            except Exception:
                logger.exception("EditBox on_change callback failed")

    def _on_keypress(self, instr: str) -> None:
        new = self._value[: self._cursor] + instr + self._value[self._cursor :]
        self._cursor = self._cursor + len(instr)
        self.value = new

    def _focus(self) -> None:
        self._cursor = len(self._value)
        self._blink_on = True
        self._last_blink = time.time()
        self._init = False

    def cursor_left(self) -> None:
        """Move the cursor to the left"""
        if self._cursor > 0:
            self._cursor -= 1
            self._init = False

    def cursor_right(self) -> None:
        """Move the cursor to the right"""
        if self._cursor < len(self._value):
            self._cursor += 1
            self._init = False

    def backspace(self) -> None:
        """Delete the character before the cursor"""
        if self._cursor == 0:
            return
        self.value = self._value[: self._cursor - 1] + self._value[self._cursor :]
        self.cursor_left()

    def delete(self) -> None:
        """Delete the character after the cursor (forward delete)"""
        if self._cursor >= len(self._value):
            return
        self.value = self._value[: self._cursor] + self._value[self._cursor + 1 :]

    def home(self) -> None:
        """Move cursor to the start of the text"""
        self._cursor = 0
        self._scroll_x = 0
        self._init = False

    def end(self) -> None:
        """Move cursor to the end of the text"""
        self._cursor = len(self._value)
        self._init = False

    def select_all(self) -> None:
        """Select all text (sets cursor to end for Ctrl+A).
        The value is preserved; use in combination with paste/replace."""
        self._cursor = len(self._value)

    def paste(self, text: str) -> None:
        """Insert text at the cursor position

        Keyword arguments:
        text -- Text to paste at cursor position
        """
        if not text:
            return
        self._on_keypress(text)

    def _find_closest_char(self, target_x: float, line_text: str) -> int:
        """Find the character index closest to target_x pixels within line_text."""
        if self.font is None:
            return len(line_text)
        best_pos = 0
        best_dist = abs(text_size(line_text[:0], self.font)[0] - target_x)
        for i in range(1, len(line_text) + 1):
            w, _ = text_size(line_text[:i], self.font)
            dist = abs(w - target_x)
            if dist < best_dist:
                best_dist = dist
                best_pos = i
        return best_pos

    def set_cursor_from_global(self, global_x: int, global_y: int) -> None:
        """Set the cursor position based on a global mouse click coordinate.

        Keyword arguments:
        global_x -- Global X coordinate of the click
        global_y -- Global Y coordinate of the click
        """
        if self.font is None:
            return
        if self._model_matrix is None or len(self._model_matrix) == 0:
            return
        mm = self._model_matrix[3]
        local_x = global_x - mm[0] - 3
        local_y = global_y - mm[1] - 3

        text_str = self._value
        if not text_str:
            self._cursor = 0
            self._init = False
            return

        if self.multiline:
            lines = text_str.split("\n")
            line_h = text_size("Ag", self.font)[1]
            if line_h <= 0:
                line_h = 16
            target_line = max(0, min(len(lines) - 1, int(local_y / line_h)))
            pos = 0
            for i in range(target_line):
                pos += len(lines[i]) + 1
            col = self._find_closest_char(local_x, lines[target_line])
            self._cursor = pos + col
        else:
            self._cursor = self._find_closest_char(local_x, text_str)
        self._init = False

    def _update_scroll(self) -> None:
        """Adjust scroll offsets so the cursor stays visible."""
        if self.font is None:
            return
        vis_w = max(self.size[0] - 6, 10)
        cursor_x = text_size(self._value[: self._cursor], self.font)[0]

        if cursor_x < self._scroll_x:
            self._scroll_x = max(0, cursor_x - 4)
        elif cursor_x > self._scroll_x + vis_w:
            self._scroll_x = cursor_x - vis_w + 4

    def _exit(self) -> None:
        self._cursor = -1
        self.on_change(self._value)
        self._init = False

    def draw(self, **kwargs: Any) -> None:
        """Create the editbox polygons and texture along with cropping"""
        super().draw()
        focused = self._cursor > -1
        if focused:
            label = self._value
            self.text.color = self.theme.text_color
        elif self._value == "" and self._placeholder:
            label = self._placeholder
            c = self.theme.text_color
            self.text.color = [c[0] * 0.4, c[1] * 0.4, c[2] * 0.4]
        else:
            label = self._value
            self.text.color = self.theme.text_color
        self.text.label = label
        if self.multiline:
            self.text.wrap(self.size[0])
        tsz = list(self.text.text_size)
        x = 3
        y = (self.size[1] - tsz[1]) / 2
        y = max(y, 0)
        y += 4
        self.text.position = [x, y]
        crop = [0, 0, tsz[0], tsz[1] + 4]

        if tsz[0] > self.size[0]:
            self._update_scroll()
            crop[0] = self._scroll_x
            crop[2] = self._scroll_x + self.size[0] - 2

        if tsz[1] > self.size[1]:
            crop[1] = tsz[1] - self.size[1] + 4
            crop[3] = tsz[1]

        self.text.crop = crop
        self.text._init = False
        self.text._init_text = False

        # Update cursor rectangle position and size
        if focused and self.font is not None:
            cursor_x = max(
                1,
                text_size(self._value[: self._cursor], self.font)[0]
                - self._scroll_x
                + 1,
            )
            font_h = text_size("Ag", self.font)[1]
            cursor_h = max(font_h, 10)
            cursor_y = (self.size[1] - cursor_h) / 2 + 3
            self._cursor_rect.position = [cursor_x, max(0, int(cursor_y))]
            self._cursor_rect.size = [2, cursor_h]
            self._cursor_rect._init = False
            self._cursor_rect.draw()
            self._cursor_rect.material.color = self.theme.highlight_color
        else:
            self._cursor_rect.hide()

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: np.ndarray | None = None,
        _primitive: int | None = None,
    ) -> None:
        """Override render to handle cursor blinking each frame."""
        now = time.time()
        if self._cursor > -1:
            if now - self._last_blink > self.BLINK_PERIOD:
                self._blink_on = not self._blink_on
                self._last_blink = now
            if self._blink_on:
                self._cursor_rect.show()
            else:
                self._cursor_rect.hide()
        else:
            self._cursor_rect.hide()
        super().render(lit, shader, parent_matrix, _primitive)

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
        self._scroll_x = 0
        self._scroll_y = 0
        self.refresh()
        self._init = False

    @property
    def placeholder(self) -> str:
        """Return the placeholder text shown when value is empty."""
        return self._placeholder

    @placeholder.setter
    def placeholder(self, text: str) -> None:
        """Set the placeholder text."""
        self._placeholder = text
        self._init = False


# ---------------------------------------------------------------------------
# Pre-built UI theme presets matching the built-in SceneTheme presets.
# Import the SceneTheme presets and derive coherent UI palettes from them.
# ---------------------------------------------------------------------------
from payton.scene.theme import (
    THEME_BLENDER,
    THEME_GAMEENGINE,
    THEME_STUDIO,
)

UI_THEME_BLENDER: Theme = Theme.from_scene_theme(THEME_BLENDER)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_BLENDER`."""

UI_THEME_STUDIO: Theme = Theme.from_scene_theme(THEME_STUDIO)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_STUDIO`."""

UI_THEME_GAMEENGINE: Theme = Theme.from_scene_theme(THEME_GAMEENGINE)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_GAMEENGINE`."""
