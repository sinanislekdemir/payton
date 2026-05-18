"""payton.scene.theme

Scene visual themes for Payton.

Three built-in presets are provided:

* :data:`THEME_BLENDER`  – neutral mid-gray viewport
* :data:`THEME_STUDIO`   – warm charcoal with vignette (default)
* :data:`THEME_STUDIO`   – deep blue-black with vignette, warm key light
* :data:`THEME_GAMEENGINE` – sky-blue gradient with horizon glow,
  differentiated major/minor grid lines, warm sunlight
"""

from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class SceneTheme:
    """Visual configuration applied to a :class:`~payton.scene.scene.Scene`.

    All colour values are in linear RGB(A) space with components in [0, 1].

    Attributes
    ----------
    background_top_color : list[float]
        RGBA colour at the top of the background gradient.
    background_bottom_color : list[float]
        RGBA colour at the bottom of the background gradient.
    background_mode : int
        Shader rendering mode for the background:
        ``0`` – clean smooth gradient (Blender-style),
        ``1`` – gradient with radial vignette (Studio),
        ``2`` – gradient with horizon glow band (Game-engine).
    grid_color : list[float]
        RGB colour for minor grid lines.
    grid_major_color : list[float] or None
        RGB colour for major grid lines.  ``None`` disables major-line
        differentiation.
    grid_major_interval : int
        Every *n*-th grid line is drawn as a major line.
    light_position : list[float]
        Default XYZ position of the scene's first light.
    light_color : list[float]
        Default RGB colour of the scene's first light.
    """

    background_top_color: List[float] = field(
        default_factory=lambda: [0.0, 0.0, 0.0, 1.0]
    )
    background_bottom_color: List[float] = field(
        default_factory=lambda: [0.0, 0.1, 0.2, 1.0]
    )
    background_mode: int = 0
    grid_color: List[float] = field(default_factory=lambda: [0.35, 0.35, 0.35])
    grid_major_color: Optional[List[float]] = None
    grid_major_interval: int = 5
    light_position: List[float] = field(default_factory=lambda: [10.0, 7.0, 6.0])
    light_color: List[float] = field(default_factory=lambda: [1.0, 1.0, 1.0])


# ---------------------------------------------------------------------------
# Preset themes
# ---------------------------------------------------------------------------

THEME_BLENDER = SceneTheme(
    # Mid-gray matching the Blender 4.x default viewport
    background_top_color=[0.40, 0.40, 0.40, 1.0],
    background_bottom_color=[0.30, 0.30, 0.30, 1.0],
    background_mode=0,
    grid_color=[0.50, 0.50, 0.50],
    grid_major_color=[0.65, 0.65, 0.65],
    grid_major_interval=5,
    light_position=[7.5, 5.0, 5.0],
    light_color=[1.0, 1.0, 1.0],
)
"""Neutral mid-gray viewport — matches the Blender 4.x default look."""

THEME_STUDIO = SceneTheme(
    # Warm charcoal — like a photography studio with a gradient sweep
    background_top_color=[0.14, 0.14, 0.18, 1.0],
    background_bottom_color=[0.22, 0.20, 0.25, 1.0],
    background_mode=1,
    grid_color=[0.28, 0.26, 0.32],
    grid_major_color=[0.40, 0.38, 0.46],
    grid_major_interval=5,
    light_position=[10.0, 5.0, 8.0],
    light_color=[1.0, 0.95, 0.85],
)
"""Warm charcoal background with a radial vignette and a warm key light."""

THEME_GAMEENGINE = SceneTheme(
    # Bright sky-blue horizon — reminiscent of Unreal / Unity editor
    background_top_color=[0.38, 0.55, 0.78, 1.0],
    background_bottom_color=[0.18, 0.28, 0.48, 1.0],
    background_mode=2,
    grid_color=[0.32, 0.38, 0.50],
    grid_major_color=[0.50, 0.58, 0.72],
    grid_major_interval=5,
    light_position=[15.0, 10.0, 12.0],
    light_color=[1.0, 0.97, 0.88],
)
"""Sky-blue gradient with a horizon glow, differentiated grid lines and
warm sunlight — reminiscent of Unreal Engine / Unity editor viewports."""
