import time
from dataclasses import dataclass
from typing import Any

from payton.scene.gui.base import Hud, Rectangle, Text

DETAIL_MINIMAL = 0
DETAIL_STANDARD = 1
DETAIL_VERBOSE = 2

LINE_HEIGHT = 18
PANEL_PADDING = 6
PANEL_WIDTH = 300


@dataclass
class FrameStats:
    fps: float = 0.0
    frame_time_ms: float = 0.0
    draw_calls: int = 0
    triangles: int = 0
    objects: int = 0
    shadow_passes: int = 0
    vram_mb: float = 0.0
    gpu_time_ms: float = 0.0
    physics_time_ms: float = 0.0


draw_calls: int = 0
triangles: int = 0
shadow_passes: int = 0
physics_time_ms: float = 0.0

_object_vram: dict[int, int] = {}
_texture_vram: dict[int, int] = {}

_vram_dirty: bool = True
_vram_cached: int = 0


def register_object_vram(obj_id: int, size_bytes: int) -> None:
    global _vram_dirty
    _object_vram[obj_id] = size_bytes
    _vram_dirty = True


def unregister_object_vram(obj_id: int) -> None:
    global _vram_dirty
    _object_vram.pop(obj_id, None)
    _vram_dirty = True


def register_texture_vram(tex_id: int, size_bytes: int) -> None:
    global _vram_dirty
    _texture_vram[tex_id] = size_bytes
    _vram_dirty = True


def unregister_texture_vram(tex_id: int) -> None:
    global _vram_dirty
    _texture_vram.pop(tex_id, None)
    _vram_dirty = True


def _compute_vram() -> int:
    global _vram_dirty, _vram_cached
    if _vram_dirty:
        _vram_cached = sum(_object_vram.values()) + sum(_texture_vram.values())
        _vram_dirty = False
    return _vram_cached


def estimate_vram_mb() -> float:
    return _compute_vram() / (1024.0 * 1024.0)


def reset_frame_counters() -> None:
    global draw_calls, triangles, shadow_passes, physics_time_ms
    draw_calls = 0
    triangles = 0
    shadow_passes = 0
    physics_time_ms = 0.0


class Profiler(Hud):
    def __init__(
        self,
        position: tuple[int, int] = (10, 10),
        width: int = PANEL_WIDTH,
        **kwargs: Any,
    ) -> None:
        super().__init__(width=width, height=600, **kwargs)
        self._profiler_position: tuple[int, int] = position
        self._panel_width: int = width
        self._detail_level: int = DETAIL_MINIMAL
        self._stats: FrameStats = FrameStats()

        self._frame_times: list[float] = []
        self._fps_smoothed: float = 0.0
        self._last_text_update: float = 0.0
        self._prev_labels: list[str] = ["", "", "", "", ""]

        self._bg: Rectangle | None = None
        self._text_title: Text | None = None
        self._text_row1: Text | None = None
        self._text_row2: Text | None = None
        self._text_row3: Text | None = None
        self._text_row4: Text | None = None
        self._text_row5: Text | None = None
        self._all_texts: list[Text] = []

        self._needs_layout: bool = True
        self._needs_text_update: bool = True

        self._build_panel()

    def cycle(self) -> None:
        if not self._visible:
            self._detail_level = DETAIL_MINIMAL
            self.show()
            self._needs_layout = True
            return
        self._detail_level += 1
        if self._detail_level > DETAIL_VERBOSE:
            self.hide()
            return
        self._needs_layout = True

    def _build_panel(self) -> None:
        px, py = self._profiler_position

        bg_w = self._panel_width
        bg_h = self._compute_panel_height()

        self._bg = Rectangle(
            position=[px, py, -1],
            size=[bg_w, bg_h],
            opacity=0.7,
        )
        self._bg.material.color = [0.05, 0.05, 0.06, 1.0]
        self.add_child("_bg", self._bg)

        text_w = self._panel_width - PANEL_PADDING * 2
        txt_y = py + PANEL_PADDING

        self._text_title = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="-- Payton Profiler --",
            color=[0.6, 0.6, 0.7, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_title", self._text_title)

        txt_y += LINE_HEIGHT
        self._text_row1 = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="",
            color=[0.9, 0.9, 0.9, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_row1", self._text_row1)

        txt_y += LINE_HEIGHT
        self._text_row2 = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="",
            color=[0.9, 0.9, 0.9, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_row2", self._text_row2)

        txt_y += LINE_HEIGHT
        self._text_row3 = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="",
            color=[0.9, 0.9, 0.9, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_row3", self._text_row3)

        txt_y += LINE_HEIGHT
        self._text_row4 = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="",
            color=[0.9, 0.9, 0.9, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_row4", self._text_row4)

        txt_y += LINE_HEIGHT
        self._text_row5 = Text(
            position=[px + PANEL_PADDING, txt_y, -1],
            size=[text_w, LINE_HEIGHT],
            label="",
            color=[0.9, 0.9, 0.9, 1.0],
            bgcolor=[0.05, 0.05, 0.06, 0.0],
        )
        self.add_child("_row5", self._text_row5)

        self._all_texts = [
            self._text_row1,
            self._text_row2,
            self._text_row3,
            self._text_row4,
            self._text_row5,
        ]
        self._apply_visibility()

    def _compute_panel_height(self) -> int:
        lines = self._visible_line_count()
        return lines * LINE_HEIGHT + PANEL_PADDING * 2

    def _visible_line_count(self) -> int:
        if self._detail_level == DETAIL_MINIMAL:
            return 1
        if self._detail_level == DETAIL_STANDARD:
            return 5
        return 6

    def _apply_visibility(self) -> None:
        if self._bg is None:
            return
        bg_h = self._compute_panel_height()
        self._bg.size = [self._panel_width, bg_h]

        px = self._profiler_position[0]
        py = self._profiler_position[1]
        base_y = py + PANEL_PADDING
        title_offset = 0

        if self._detail_level == DETAIL_MINIMAL:
            if self._text_title is not None:
                self._text_title.hide()
        else:
            if self._text_title is not None:
                self._text_title.show()
                self._text_title.position = [px + PANEL_PADDING, base_y, -1]
            title_offset = LINE_HEIGHT

        rows = [
            self._text_row1,
            self._text_row2,
            self._text_row3,
            self._text_row4,
            self._text_row5,
        ]

        for i, t in enumerate(rows):
            if t is None:
                continue
            y = base_y + title_offset + i * LINE_HEIGHT
            t.position = [px + PANEL_PADDING, y, -1]

        for i, t in enumerate(self._all_texts):
            if self._detail_level == DETAIL_MINIMAL:
                (t.show() if i == 0 else t.hide())
            elif self._detail_level == DETAIL_STANDARD:
                (t.show() if i < 4 else t.hide())
            else:
                t.show()

    def set_stats(self, stats: FrameStats) -> None:
        self._stats = stats
        now = time.perf_counter()
        if now - self._last_text_update >= 0.5:
            self._needs_text_update = True

    def update(self) -> None:
        if not self._visible:
            return
        if self._needs_layout:
            self._apply_visibility()
            self._needs_layout = False
        if self._needs_text_update:
            self._last_text_update = time.perf_counter()
            self._refresh_texts()
            self._needs_text_update = False

    def _compute_fps_color(self, fps: float) -> list[float]:
        if fps >= 55.0:
            return [0.3, 1.0, 0.3, 1.0]
        if fps >= 25.0:
            return [1.0, 0.7, 0.0, 1.0]
        return [1.0, 0.2, 0.2, 1.0]

    def _refresh_texts(self) -> None:
        s = self._stats
        assert self._text_row1 is not None
        assert self._text_row2 is not None
        assert self._text_row3 is not None
        assert self._text_row4 is not None
        assert self._text_row5 is not None

        labels = [
            f"FPS: {s.fps:5.1f}      Frame: {s.frame_time_ms:5.2f} ms",
            f"Draw Calls: {s.draw_calls:<5d}  Triangles: {s.triangles:>7,d}",
            f"Objects: {s.objects:<6d}   VRAM: ~{s.vram_mb:6.1f} MB",
            f"Shadows: {s.shadow_passes} pass  Physics: {s.physics_time_ms:5.2f} ms",
            f"GPU: {s.gpu_time_ms:5.2f} ms",
        ]

        rows = [
            self._text_row1,
            self._text_row2,
            self._text_row3,
            self._text_row4,
            self._text_row5,
        ]

        for i in range(5):
            if labels[i] != self._prev_labels[i]:
                rows[i].label = labels[i]
                self._prev_labels[i] = labels[i]

        fps_color = self._compute_fps_color(s.fps)
        self._text_row1.color = fps_color
