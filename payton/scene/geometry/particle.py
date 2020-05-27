from typing import Any, Dict, List, Optional

from payton.scene.geometry import Object
from payton.scene.material import BASE_PARTICLE, DEFAULT, POINTS
from payton.scene.shader import PARTICLE_SHADER


class ParticleSystem(Object):
    def __init__(self, particle_scale: float = 1.0, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # Buffer to keep the state of all particles,
        # hitting update will switch buffer into particles
        self.material.texture = BASE_PARTICLE
        self.particle_scale = particle_scale
        self._previous_camera_position: Optional[List[float]] = None
        self.meta: List[Dict[str, Any]] = []
        self.shader = PARTICLE_SHADER

    def toggle_wireframe(self) -> None:
        pass

    def track(self) -> bool:
        return False

    def add(self, vertex: List[float], **meta: Dict[str, Any]):
        self._vertices.append(vertex)
        i = len(self._vertices) - 1
        self._indices.append([i])
        self.materials[DEFAULT]._indices.append([i])
        self.materials[DEFAULT].display = POINTS
        self.meta.append(meta)
        self.refresh()
