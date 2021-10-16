"""Physics resolver."""
physics_client = None

try:
    import pybullet

    print("Bullet Physics enabled")
    physics_client = pybullet.connect(pybullet.DIRECT)
except ModuleNotFoundError:
    print("Bullet Physics is not installed, continue without Physics support")


class PhysicsException(Exception):
    """Raise this exception when needed."""

    pass
