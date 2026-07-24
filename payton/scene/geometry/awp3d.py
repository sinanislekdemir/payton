"""
Animated Wavefront Pack format.

Animated Wavefront Pack 3D files (AWP3D) are simply ZIP
files with each frame packed inside.

Easiest way to generate those files is through using Blender Export
Add-On.
"""

import os
import time
from collections.abc import Callable
from typing import Any
from zipfile import ZipFile

import numpy as np

from payton.scene.geometry import Wavefront
from payton.scene.material import POINTS
from payton.scene.shader import DEFAULT_SHADER, PARTICLE_SHADER, Shader
from payton.tools.bar import progress

_BULLET = False
try:
    import pybullet

    _BULLET = True
except ModuleNotFoundError:
    _BULLET = False


class AWP3D(Wavefront):
    """
    AWP3D File Format Loader.

    This file format is quite like MD2 but supports high-poly animations.
    Simply, each awp3d file is a zip file with every frame of an anination
    is exported as Wavefront and packed.

    AWP3D loads the zip file and parses each Wavefront. Then loads them as
    key frames in Payton
    """

    def __init__(
        self,
        filename: str = "",
        fps: int = 30,
        progress_callback: Callable[[int, int], None] | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize AWP3D.

        Keyword arguments:
        filename -- File name to load
        fps -- Frames per second
        progress_callback -- Optional callback receiving (current, total) during loading
        """
        super().__init__(**kwargs)
        self.frames: list[Wavefront] = []
        self._frame: int = 0
        self.num_frames: int = 0
        self._time: float = 0
        self._path: str = ""
        self._loop: bool = False
        self._from_frame = 0
        self._to_frame = 0
        self._active_framecount = 0
        self.fps = fps
        self._frame_period = 1.0 / fps
        self.animate = True
        self._time = 0.0
        self.animations: dict[str, tuple[int, int]] = {}
        self._needs_update = True

        if os.path.exists(filename):
            self.load_file(filename, progress_callback=progress_callback)

    def start(self) -> None:
        """Manually start animation."""
        self.animate = True

    def pause(self) -> None:
        """Pause animation."""
        self.animate = False

    def stop(self) -> None:
        """Stop animation."""
        self.animate = False
        self._time = 0.0

    def set_range(self, from_frame: int, to_frame: int) -> None:
        """Set animation frame range.

        Keyword arguments:
        from_frame -- Starting frame number
        to_frame -- Ending frame number
        """
        if from_frame >= self.num_frames:
            raise ValueError("Frame range error - from_frame out of bounds")
        if to_frame >= self.num_frames:
            raise ValueError("Frame range error - to_frame out of bounds")
        if from_frame < 0 or to_frame < 0:
            raise ValueError("Frame range error - Negative value defined")
        self._from_frame = from_frame
        self._to_frame = to_frame
        self._active_framecount = to_frame - from_frame
        self._frame_period = 1.0 / self.fps

    def set_animation(self, name: str, from_frame: int, to_frame: int) -> None:
        """Create an animation definition.

        Keyword arguments:
        name -- Name of the animation
        from_frame -- Animation starting frame number
        to_frame -- Animation ending frame number
        """
        self.animations[name] = (from_frame, to_frame)

    def run_animation(self, name: str) -> None:
        """Run the selected animation.

        Keyword arguments:
        name -- Name of the desired animation
        """
        if name not in self.animations:
            raise ValueError("Animation not defined")
        self.set_range(self.animations[name][0], self.animations[name][1])

    def build(self) -> bool:
        if len(self.frames) == 0:
            return False
        if len(self.frames[0]._model_matrix) > 0:
            self.frames[0].build()
            self._build_collision_shape()
            self._needs_update = False
        return True

    def load_file(
        self,
        filename: str,
        progress_callback: Callable[[int, int], None] | None = None,
    ) -> bool:
        """Load file into system.

        Keyword arguments:
        filename -- File name to load
        progress_callback -- Optional callback receiving (current, total) during loading
        """
        if not os.path.exists(filename):
            raise ValueError(f"File not found: {filename}")
        self._path = os.path.dirname(os.path.abspath(filename))
        archive = ZipFile(filename, "r")
        files = archive.namelist()
        max_frame = max([int(f.split(".")[0]) for f in files]) + 1
        min_frame = min(int(f.split(".")[0]) for f in files)
        total = max_frame - 1
        for f in range(min_frame, max_frame):
            if progress_callback is not None:
                progress_callback(f, total)
            else:
                progress(f, total)
            wobj = Wavefront()
            wobj.path = self._path
            data = archive.read(f"{f}.obj").decode("utf-8")
            material = archive.read(f"{f}.mtl").decode("utf-8")
            wobj.load_material(material)
            wobj.load(data)
            self.frames.append(wobj)
        self._active_framecount = max_frame - min_frame
        self.num_frames = max_frame - min_frame
        archive.close()
        return True

    def begin_incremental_load(self, filename: str) -> int:
        """Prepare for incremental loading. Returns the total frame count.

        Call load_next_frame() repeatedly, then finish_incremental_load() when done.

        Keyword arguments:
        filename -- File name to load
        """
        if not os.path.exists(filename):
            raise ValueError(f"File not found: {filename}")
        self._path = os.path.dirname(os.path.abspath(filename))
        self._load_archive = ZipFile(filename, "r")
        files = self._load_archive.namelist()
        self._load_current = min(int(f.split(".")[0]) for f in files)
        self._load_max = max([int(f.split(".")[0]) for f in files]) + 1
        self._load_min = self._load_current
        return self._load_max - self._load_min

    def load_next_frame(
        self, progress_callback: Callable[[int, int], None] | None = None
    ) -> bool:
        """Load the next frame during incremental loading.

        Returns True if there are more frames to load, False when complete.

        Keyword arguments:
        progress_callback -- Optional callback receiving (current, total) during loading
        """
        if self._load_current >= self._load_max:
            return False
        total = self._load_max - 1
        if progress_callback is not None:
            progress_callback(self._load_current, total)
        wobj = Wavefront()
        wobj.path = self._path
        data = self._load_archive.read(f"{self._load_current}.obj").decode("utf-8")
        material = self._load_archive.read(f"{self._load_current}.mtl").decode("utf-8")
        wobj.load_material(material)
        wobj.load(data)
        self.frames.append(wobj)
        self._load_current += 1
        return self._load_current < self._load_max

    def finish_incremental_load(self) -> bool:
        """Finalize incremental loading. Must be called after all frames are loaded."""
        self._active_framecount = self._load_max - self._load_min
        self.num_frames = self._load_max - self._load_min
        self._load_archive.close()
        return True

    def render(
        self,
        lit: bool,
        shader: Shader,
        parent_matrix: np.ndarray | None = None,
        _primitive: int | None = None,
    ) -> None:
        """Render AWP3D."""
        if not self._visible:
            return
        self.update_matrix(parent_matrix=parent_matrix)
        self.track()
        if self._needs_update:
            self.build()

        if not self.animate:
            self.frames[0].render(lit, shader, self._model_matrix)
            return

        if self._time == 0:
            self._time = time.time()

        if time.time() - self._time >= self._frame_period:
            self._frame = (self._frame + 1) % self._active_framecount
            self._time = time.time()
        render_frame = self._frame + self._from_frame
        self.frames[render_frame].render(lit, shader, self._model_matrix)
        if _BULLET:
            self._build_constraints()

    def toggle_wireframe(self) -> None:
        """Toggle wireframe view of the Object."""
        d = (self.material.display + 1) % 3

        for mat in self.materials.values():
            mat.display = d
            mat.particle_size = 0.01

        self.shader = PARTICLE_SHADER if d == POINTS else DEFAULT_SHADER
        self.refresh()
        for frame in self.frames:
            frame.toggle_wireframe()

    def _create_collision_shape(self) -> None:
        width = self.frames[0].bounding_box[1][0] - self.frames[0].bounding_box[0][0]
        depth = self.frames[0].bounding_box[1][1] - self.frames[0].bounding_box[0][1]
        height = self.frames[0].bounding_box[1][2] - self.frames[0].bounding_box[0][2]
        self._bullet_shape_id = pybullet.createCollisionShape(
            pybullet.GEOM_BOX, halfExtents=[width, depth, height]
        )

    @property
    def physics(self) -> bool:
        return True
