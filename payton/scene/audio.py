"""3D spatial audio engine for Payton.

Uses miniaudio for cross-platform audio playback with zero system dependencies
and numpy for 3D spatial processing (distance attenuation + stereo panning).
"""

import logging
import math
from collections import OrderedDict
from collections.abc import Generator
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Optional

import miniaudio
import numpy as np

from payton.math.vector import Vector3D

if TYPE_CHECKING:
    from payton.scene.geometry.base import Object

logger = logging.getLogger(__name__)

SOUND_CACHE_SIZE = 64


@dataclass
class AudioListener:
    """Position and orientation of the virtual ears.

    Updated each frame from the active camera's transform.
    """

    position: Vector3D = field(default_factory=lambda: [0.0, 0.0, 0.0])
    forward: Vector3D = field(default_factory=lambda: [0.0, 1.0, 0.0])
    up: Vector3D = field(default_factory=lambda: [0.0, 0.0, 1.0])


class AudioSource:
    """A playable sound instance with 3D spatial properties.

    Created by :meth:`AudioEngine.play_sound` and returned by
    :meth:`Object.playSound` / :meth:`Scene.playSound`.  Use ``.stop()``,
    ``.pause()``, ``.resume()`` and adjust ``.volume`` / ``.pitch`` at runtime.
    """

    def __init__(
        self,
        data: np.ndarray,
        position: Vector3D,
        obj: Optional["Object"],
        volume: float,
        pitch: float,
        looping: bool,
        rolloff: float,
        one_shot: bool,
        engine: "AudioEngine",
    ) -> None:
        self.data = data
        self.position = list(position)
        self._object = obj
        self.volume = volume
        self.pitch = pitch
        self.looping = looping
        self.rolloff = rolloff
        self._one_shot = one_shot
        self._engine = engine
        self._playhead: int = 0
        self._paused = False
        self._done = False

    @property
    def state(self) -> str:
        """Current playback state: ``"playing"``, ``"paused"`` or ``"stopped"``."""
        if self._done:
            return "stopped"
        if self._paused:
            return "paused"
        return "playing"

    def stop(self) -> None:
        """Stop playback and remove this source from the engine."""
        self._done = True
        self._engine._remove_source(self)

    def pause(self) -> None:
        """Pause playback.  Call :meth:`resume` to continue."""
        self._paused = True

    def resume(self) -> None:
        """Resume playback after :meth:`pause`."""
        self._paused = False


class AudioEngine:
    """Central audio mixer and playback device manager.

    Owns a :class:`miniaudio.PlaybackDevice` that pulls mixed stereo audio
    from a callback generator running on miniaudio's internal audio thread.

    Parameters
    ----------
    sample_rate : int
        Output sample rate in Hz.  Default 44100.
    buffersize_msec : int
        Audio buffer size in milliseconds.  Lower = less latency but higher
        CPU usage.  Default 80.
    cache_size : int
        Maximum number of decoded sound files to keep in memory.  Default 64.
    """

    def __init__(
        self,
        sample_rate: int = 44100,
        buffersize_msec: int = 80,
        cache_size: int = SOUND_CACHE_SIZE,
    ) -> None:
        self.sample_rate = sample_rate
        self.listener = AudioListener()
        self.master_volume: float = 1.0
        self._sources: list[AudioSource] = []
        self._sound_cache: OrderedDict[str, np.ndarray] = OrderedDict()
        self._cache_size = cache_size
        self._device: miniaudio.PlaybackDevice | None = None
        self._generator: Generator[memoryview, int, None] | None = None
        self._running = False

    def start(self) -> None:
        """Open the audio device and begin the mixing loop.

        Safe to call multiple times; subsequent calls are no-ops if the
        device is already running.
        """
        if self._running:
            return
        self._device = miniaudio.PlaybackDevice(
            output_format=miniaudio.SampleFormat.FLOAT32,
            nchannels=2,
            sample_rate=self.sample_rate,
            buffersize_msec=80,
        )
        self._generator = self._audio_generator()
        next(self._generator)
        self._device.start(self._generator)
        self._running = True

    def stop(self) -> None:
        """Close the audio device and stop all playback."""
        if not self._running:
            return
        self._sources.clear()
        if self._device is not None:
            self._device.stop()
            self._device.close()
            self._device = None
        self._generator = None
        self._running = False

    def play_sound(
        self,
        file: str,
        position: Vector3D | None = None,
        obj: Optional["Object"] = None,
        loop: bool = False,
        volume: float = 1.0,
        pitch: float = 1.0,
    ) -> AudioSource:
        """Decode a sound file (caching it) and start playback.

        Parameters
        ----------
        file : str
            Path to a WAV, MP3, FLAC or Ogg Vorbis file.
        position : Vector3D or None
            World position for the source.  Ignored if *obj* is provided.
        obj : Object or None
            If given, the source tracks this object's world position.
        loop : bool
            Whether to loop the sound indefinitely.
        volume : float
            Volume multiplier (0.0–1.0).
        pitch : float
            Playback speed factor.  1.0 is normal speed.

        Returns
        -------
        AudioSource
            Handle for controlling the sound instance.
        """
        if file not in self._sound_cache:
            try:
                decoded = miniaudio.decode_file(
                    file,
                    nchannels=1,
                    sample_rate=self.sample_rate,
                    output_format=miniaudio.SampleFormat.FLOAT32,
                )
            except miniaudio.DecodeError:
                logger.error(f"Failed to decode audio file: {file}")
                return AudioSource(
                    np.array([], dtype=np.float32),
                    position or [0.0, 0.0, 0.0],
                    obj,
                    volume,
                    pitch,
                    loop,
                    1.0,
                    True,
                    self,
                )
            samples = np.array(decoded.samples, dtype=np.float32)
            if len(self._sound_cache) >= self._cache_size:
                self._sound_cache.popitem(last=False)
            self._sound_cache[file] = samples
        data = self._sound_cache[file]
        pos = position if position is not None else [0.0, 0.0, 0.0]
        one_shot = not loop
        source = AudioSource(data, pos, obj, volume, pitch, loop, 1.0, one_shot, self)
        self._sources.append(source)
        return source

    def _remove_source(self, source: AudioSource) -> None:
        """Remove a source from the active list."""
        try:
            self._sources.remove(source)
        except ValueError:
            pass

    def stop_all_from(self, obj: Optional["Object"]) -> None:
        """Stop all sources associated with *obj* (or all unowned sources if None)."""
        for source in self._sources:
            if source._object is obj:
                source._done = True

    def _get_source_position(self, source: AudioSource) -> tuple[float, float, float]:
        if source._object is not None:
            m = source._object.matrix
            return (m[3][0], m[3][1], m[3][2])
        return (source.position[0], source.position[1], source.position[2])

    def _compute_spatial_gains(
        self, sx: float, sy: float, sz: float
    ) -> tuple[float, float]:
        lx, ly, lz = self.listener.position
        fx, fy, fz = self.listener.forward
        ux, uy, uz = self.listener.up

        dx = sx - lx
        dy = sy - ly
        dz = sz - lz
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        attenuation = 1.0 / (1.0 + dist * dist)

        if dist < 0.001:
            return (attenuation, attenuation)

        dx /= dist
        dy /= dist
        dz /= dist

        # Right direction = forward × up
        rx = fy * uz - fz * uy
        ry = fz * ux - fx * uz
        rz = fx * uy - fy * ux
        rlen = math.sqrt(rx * rx + ry * ry + rz * rz)
        if rlen > 0.0:
            rx /= rlen
            ry /= rlen
            rz /= rlen

        pan = dx * rx + dy * ry + dz * rz
        if pan < -1.0:
            pan = -1.0
        elif pan > 1.0:
            pan = 1.0

        angle = (pan + 1.0) * math.pi * 0.25
        left = math.cos(angle) * attenuation
        right = math.sin(angle) * attenuation
        return (left, right)

    def _audio_generator(self) -> Generator[memoryview, int, None]:
        """Callback generator for miniaudio PlaybackDevice.

        This runs on miniaudio's internal audio thread.  It receives the
        required frame count via ``.send()`` and yields mixed stereo audio
        as a memoryview of float32 samples.
        """
        required_frames: int = yield memoryview(b"")

        while True:
            if required_frames <= 0:
                required_frames = yield memoryview(b"")
                continue

            # Mix into a stereo (nchannels=2) float32 buffer
            output = np.zeros((required_frames, 2), dtype=np.float32)

            # Snapshot active sources (minimizes lock contention)
            sources = list(self._sources)
            for source in sources:
                if source._done:
                    continue
                if source._paused:
                    continue
                if len(source.data) == 0:
                    continue

                plen = len(source.data)
                remaining = plen - source._playhead
                if remaining <= 0:
                    if source.looping:
                        source._playhead = 0
                        remaining = plen
                    else:
                        source._done = True
                        continue

                take = min(required_frames, remaining)

                # Apply pitch: step through source at pitch rate
                if source.pitch != 1.0 and source.pitch > 0.0:
                    src_indices = (
                        source._playhead
                        + np.arange(take, dtype=np.float64) * source.pitch
                    )
                    src_indices = np.clip(src_indices, 0, plen - 1).astype(np.int64)
                    chunk = source.data[src_indices]
                else:
                    chunk = source.data[source._playhead : source._playhead + take]

                source._playhead += int(take * source.pitch)

                # Compute 3D spatial gains
                sx, sy, sz = self._get_source_position(source)
                left_gain, right_gain = self._compute_spatial_gains(sx, sy, sz)
                left_gain *= source.volume * self.master_volume
                right_gain *= source.volume * self.master_volume

                if take < required_frames:
                    output[:take, 0] += chunk[:take] * left_gain
                    output[:take, 1] += chunk[:take] * right_gain
                else:
                    output[:, 0] += chunk[:required_frames] * left_gain
                    output[:, 1] += chunk[:required_frames] * right_gain

            # Clean up finished one-shot sources
            for source in sources:
                if source._done and source._one_shot:
                    try:
                        self._sources.remove(source)
                    except ValueError:
                        pass

            # Convert to bytes for miniaudio
            result = np.ascontiguousarray(output, dtype=np.float32)
            required_frames = yield memoryview(result.tobytes())
