"""Physics backend resolver.

Payton ships with a small, dependency-free physics engine.  When the optional
``pybullet`` package is installed, Payton uses it instead.  You can force the
built-in engine at any time with ``Scene(use_internal_physics=True)``.

``physics_client`` is kept for backward compatibility and is the connected
PyBullet client when ``pybullet`` is available, otherwise ``None``.
"""

from payton.scene.internal_physics import (
    COLLISION_AUTO,
    COLLISION_BOX,
    COLLISION_CAPSULE,
    COLLISION_SHAPES,
    COLLISION_SPHERE,
    InternalPhysicsWorld,
)

__all__ = [
    "COLLISION_AUTO",
    "COLLISION_BOX",
    "COLLISION_CAPSULE",
    "COLLISION_SHAPES",
    "COLLISION_SPHERE",
    "InternalPhysicsWorld",
    "PhysicsException",
    "physics_client",
    "pybullet_available",
]

physics_client = None
pybullet_available = False

try:
    import pybullet

    print("Bullet Physics enabled")
    physics_client = pybullet.connect(pybullet.DIRECT)
    pybullet_available = True
except ModuleNotFoundError:
    print("Bullet Physics is not installed, using the built-in physics engine")


class PhysicsException(Exception):
    """Raise this exception when needed."""
