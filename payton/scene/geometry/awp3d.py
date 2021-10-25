"""
Animated Wavefront Pack format.

Animated Wavefront Pack 3D files (AWP3D) are simply ZIP
files with each frame packed inside.

Easiest way to generate those files is through using Blender Export
Add-On.
"""
import os
import time
from typing import Any, Dict, List, Optional, Tuple
from zipfile import ZipFile

import numpy as np

from payton.scene.geometry import Wavefront
from payton.scene.material import POINTS
from payton.scene.shader import DEFAULT_SHADER, PARTICLE_SHADER, Shader
from payton.tools.bar import progress


class AWP3D(Wavefront):
    """
    AWP3D File Format Loader.

    This file format is quite like MD2 but supports high-poly animations.
    Simply, each awp3d file is a zip file with every frame of an anination
    is exported as Wavefront and packed.

    AWP3D loads the zip file and parses each Wavefront. Then loads them as
    key frames in Payton
    """

    def __init__(self, filename: str = "", fps: int = 30, **kwargs: Any) -> None:
        """Initialize AWP3D."""
        super().__init__(**kwargs)
        self.frames: List[Wavefront] = []
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
        self.animations: Dict[str, Tuple[int, int]] = {}

        if os.path.exists(filename):
            self.load_file(filename)

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
            raise BaseException("Frame range error - from_frame out of bounds")
        if to_frame >= self.num_frames:
            raise BaseException("Frame range error - to_frame out of bounds")
        if from_frame < 0 or to_frame < 0:
            raise BaseException("Frame range error - Negative value defined")
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
            raise BaseException("Animation not defined")
        self.set_range(self.animations[name][0], self.animations[name][1])

    def load_file(self, filename: str) -> bool:
        """Load file into system.

        NOTE: This takes a while, that's why there is a loading
        indicator here. If it is annoying for you, ping me thus
        I can remove ;)

        Keyword arguments:
        filename -- File name to load
        """
        if not os.path.exists(filename):
            raise BaseException(f"File not found: {filename}")
        self._path = os.path.dirname(os.path.abspath(filename))
        archive = ZipFile(filename, 'r')
        files = archive.namelist()
        max_frame = max([int(f.split('.')[0]) for f in files]) + 1
        min_frame = min(int(f.split('.')[0]) for f in files)
        for f in range(min_frame, max_frame):
            progress(f, max_frame - 1)
            wobj = Wavefront()
            wobj.path = self._path
            data = archive.read(f"{f}.obj").decode('utf-8')
            material = archive.read(f"{f}.mtl").decode('utf-8')
            wobj.load_material(material)
            wobj.load(data)
            self.frames.append(wobj)
        self._active_framecount = max_frame - min_frame
        self.num_frames = max_frame - min_frame
        archive.close()
        return True

    def render(
        self, lit: bool, shader: Shader, parent_matrix: Optional[np.ndarray] = None, _primitive: int = None
    ) -> None:
        """Render AWP3D."""
        if not self._visible:
            return
        self.update_matrix(parent_matrix=parent_matrix)
        self.track()

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
