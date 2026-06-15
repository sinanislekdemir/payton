"""Basic Window and UI Elements Support

This is a simple support but works without much hassle.
I tried to make the whole mechanism as basic as possible.
Anyone without any UI coding experience should be able to get started
with the basic stuff"""

from enum import Enum
from typing import Any, Callable, Dict, List, Optional, cast

import numpy as np
from PIL import Image, ImageDraw

from payton.math.vector import Vector3D
from payton.scene.geometry.base import Object
from payton.scene.geometry.mesh import Mesh
from payton.scene.gui.base import Shape2D, Text
from payton.scene.shader import Shader
from payton.scene.theme import SceneTheme

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
        _primitive: Optional[int] = None,
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

    def click(self, x: int, y: int) -> Optional[Mesh]:
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
        crop: List[int] = [0, 0, img_w, img_h]

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


# ---------------------------------------------------------------------------
# Pre-built UI theme presets matching the built-in SceneTheme presets.
# Import the SceneTheme presets and derive coherent UI palettes from them.
# ---------------------------------------------------------------------------
from payton.scene.theme import THEME_BLENDER, THEME_GAMEENGINE, THEME_STUDIO  # noqa: E402

UI_THEME_BLENDER: Theme = Theme.from_scene_theme(THEME_BLENDER)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_BLENDER`."""

UI_THEME_STUDIO: Theme = Theme.from_scene_theme(THEME_STUDIO)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_STUDIO`."""

UI_THEME_GAMEENGINE: Theme = Theme.from_scene_theme(THEME_GAMEENGINE)
"""UI theme that pairs with :data:`~payton.scene.theme.THEME_GAMEENGINE`."""
