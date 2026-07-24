from collections.abc import Sequence
from typing import Any

import numpy as np
from OpenGL.GL import (
    GL_CLAMP_TO_EDGE,
    GL_COMPARE_REF_TO_TEXTURE,
    GL_DEPTH_COMPONENT,
    GL_DEPTH_COMPONENT24,
    GL_FLOAT,
    GL_FRAMEBUFFER,
    GL_LEQUAL,
    GL_LINEAR,
    GL_NONE,
    GL_TEXTURE_COMPARE_FUNC,
    GL_TEXTURE_COMPARE_MODE,
    GL_TEXTURE_CUBE_MAP,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_X,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Y,
    GL_TEXTURE_CUBE_MAP_NEGATIVE_Z,
    GL_TEXTURE_CUBE_MAP_POSITIVE_X,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Y,
    GL_TEXTURE_CUBE_MAP_POSITIVE_Z,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TEXTURE_WRAP_R,
    GL_TEXTURE_WRAP_S,
    GL_TEXTURE_WRAP_T,
    glBindFramebuffer,
    glBindTexture,
    glDeleteFramebuffers,
    glDeleteTextures,
    glDrawBuffer,
    glGenFramebuffers,
    glGenTextures,
    glReadBuffer,
    glTexImage2D,
    glTexParameteri,
)
from pyrr import matrix44

from payton.math.vector import Vector3D

_CUBEMAP_FACES = [
    (GL_TEXTURE_CUBE_MAP_POSITIVE_X, [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
    (GL_TEXTURE_CUBE_MAP_NEGATIVE_X, [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
    (GL_TEXTURE_CUBE_MAP_POSITIVE_Y, [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
    (GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]),
    (GL_TEXTURE_CUBE_MAP_POSITIVE_Z, [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]),
    (GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, [0.0, 0.0, -1.0], [0.0, -1.0, 0.0]),
]


class Light:
    def __init__(
        self,
        position: Vector3D | None = None,
        color: Vector3D | None = None,
        cast_shadows: bool = True,
        shadow_far: float = 100.0,
        shadow_bias: float = 0.005,
        **kwargs: Any,
    ):
        """Initialize light.

        Light in Payton is a point light radiating in all directions.

        Keyword arguments:
        position -- Position of the light in space
        color -- Color of the light source
        cast_shadows -- Whether this light casts shadows (default True)
        shadow_far -- Far plane distance for the shadow cubemap (default 100.0)
        shadow_bias -- Depth bias for shadow comparison (default 0.005)
        """
        self._position = [10.0, 7.0, 6.0] if position is None else position
        self._color = [1.0, 1.0, 1.0] if color is None else color
        self._position_np: np.ndarray = np.array(
            list(self._position), dtype=np.float32
        )
        self._color_np: np.ndarray = np.array(list(self._color), dtype=np.float32)

        self.active: bool = True
        self.cast_shadows: bool = cast_shadows
        self.shadow_far: float = shadow_far
        self.shadow_bias: float = shadow_bias

        self._shadow_cubemap_tex: int = 0
        self._shadow_cubemap_fbo: int = 0
        self._shadow_face_size: int = 0
        self._shadow_dirty: bool = True

    @property
    def position(self) -> Vector3D:
        """Return the position of the light."""
        return self._position

    @position.setter
    def position(self, position: Vector3D) -> None:
        """Set the position of the light.

        Keyword arguments:
        position -- Position in space
        """
        self._position = position
        self._position_np = np.array(self.position, dtype=np.float32)
        self._shadow_dirty = True

    def to_dict(self) -> dict[str, Any]:
        """Convert the light into dictionary."""
        return {
            "position": self.position,
            "color": self.color,
            "active": self.active,
        }

    @property
    def color(self) -> Vector3D:
        """Return the light color."""
        return self._color

    @color.setter
    def color(self, color: Vector3D) -> None:
        """Set the light color.

        Keyword arguments:
        color -- Color of the light
        """
        self._color = color
        self._color_np = np.array(self._color, dtype=np.float32)

    def init_shadow_cubemap(self, face_size: int) -> None:
        """Create the shadow cubemap FBO and texture.

        Keyword arguments:
        face_size -- Width/height in pixels for each cubemap face
        """
        if face_size <= 0:
            return
        self.free_shadow_cubemap()
        self._shadow_face_size = face_size

        self._shadow_cubemap_tex = glGenTextures(1)
        glBindTexture(GL_TEXTURE_CUBE_MAP, self._shadow_cubemap_tex)

        for i in range(6):
            glTexImage2D(
                GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
                0,
                GL_DEPTH_COMPONENT24,
                face_size,
                face_size,
                0,
                GL_DEPTH_COMPONENT,
                GL_FLOAT,
                None,
            )

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE)
        glTexParameteri(
            GL_TEXTURE_CUBE_MAP, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE
        )
        glTexParameteri(
            GL_TEXTURE_CUBE_MAP, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL
        )

        self._shadow_cubemap_fbo = glGenFramebuffers(1)
        glBindFramebuffer(GL_FRAMEBUFFER, self._shadow_cubemap_fbo)
        glDrawBuffer(GL_NONE)
        glReadBuffer(GL_NONE)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindTexture(GL_TEXTURE_CUBE_MAP, 0)

    def free_shadow_cubemap(self) -> None:
        """Release shadow cubemap resources."""
        if self._shadow_cubemap_tex > 0:
            glDeleteTextures([self._shadow_cubemap_tex])
            self._shadow_cubemap_tex = 0
        if self._shadow_cubemap_fbo > 0:
            glDeleteFramebuffers(1, [self._shadow_cubemap_fbo])
            self._shadow_cubemap_fbo = 0
        self._shadow_face_size = 0

    def shadow_face_matrices(self) -> Sequence[Sequence[Sequence[float]]]:
        """Return tuple of (target, up) vectors for the six cubemap faces."""
        return _CUBEMAP_FACES

    def shadow_projection(self) -> np.ndarray:
        """Return the perspective projection matrix for shadow cubemap faces."""
        return matrix44.create_perspective_projection_matrix(
            90.0,
            1.0,
            0.1,
            self.shadow_far,
            dtype=np.float32,
        )
