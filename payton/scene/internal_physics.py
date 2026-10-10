"""Built-in physics engine for Payton.

Payton runs physics through Bullet (``pybullet``) when it is installed and
falls back to this dependency-free engine otherwise.  You can also force the
built-in engine with ``Scene(use_internal_physics=True)``.

The engine is intentionally small and readable: it simulates *solid*, convex
rigid bodies with a sequential-impulse solver (Erin Catto's GDC 2007 approach,
as used by Box2D and Randy Gaul's qu3e).  Key design points:

* **Persistent contacts** -- a manifold is kept per overlapping body pair and
  re-used between steps.
* **Warm starting** -- contact impulses are matched frame to frame and applied
  before solving, which is what keeps stacks stable and pushes deep overlap
  apart instead of freezing it.
* **Baumgarte stabilisation** with a penetration slop.
* A **sweep-and-prune** broadphase and per-body sleeping.

The solver design is adapted from **qu3e** by Randy Gaul (zlib licence) and the
sequential-impulse method of Erin Catto / Box2D; this is an altered Python
re-implementation, not the original software.  See ``THIRD_PARTY_NOTICES.md``.

Supported collision shapes are:

* :class:`BoxShape`    -- orientable box (``Cube``)
* :class:`SphereShape` -- sphere (``Sphere``)
* :class:`CapsuleShape`-- capsule, aligned along the object's local Z axis
* :class:`PlaneShape`  -- infinite plane (``Plane``)

Any other mesh can opt into a simple approximation through
:attr:`payton.scene.geometry.base.Object.collision_approximation` (``"box"``,
``"sphere"`` or ``"capsule"``).

The public object parameters mirror PyBullet, so ``mass``,
``linear_velocity`` and :meth:`payton.scene.geometry.base.Object.change_dynamics`
drive either engine the same way.
"""

import logging
import math
import threading
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

import numpy as np

from payton.math.matrix import matrix_to_position_and_quaternion

if TYPE_CHECKING:
    from payton.scene.geometry.base import Object

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Collision approximation kinds (used by Object.collision_approximation)
# ---------------------------------------------------------------------------
COLLISION_AUTO = "auto"
COLLISION_BOX = "box"
COLLISION_SPHERE = "sphere"
COLLISION_CAPSULE = "capsule"
COLLISION_SHAPES = (
    COLLISION_AUTO,
    COLLISION_BOX,
    COLLISION_SPHERE,
    COLLISION_CAPSULE,
)

# ---------------------------------------------------------------------------
# Sensible defaults so a newcomer can just set ``mass`` and go
# ---------------------------------------------------------------------------
DEFAULT_GRAVITY: tuple[float, float, float] = (0.0, 0.0, -9.8)
DEFAULT_TIME_STEP: float = 1.0 / 120.0
DEFAULT_SOLVER_ITERATIONS: int = 20
DEFAULT_RESTITUTION: float = 0.0
DEFAULT_FRICTION: float = 0.5
DEFAULT_LINEAR_DAMPING: float = 0.04
DEFAULT_ANGULAR_DAMPING: float = 0.1

# Solver tuning
# Baumgarte stabilisation: fraction of the penetration (beyond the slop) turned
# into a separating velocity each step, so the solver actively pushes overlap
# apart.  The resulting speed is clamped -- for deep overlap ``beta * pen / dt``
# would otherwise be tens of m/s and fling bodies around.
_BAUMGARTE = 0.2
_MAX_BIAS_VELOCITY = 2.0
_PENETRATION_SLOP = 0.005
_RESTITUTION_THRESHOLD = 1.0
_SLEEP_LINEAR_THRESHOLD = 0.05
_SLEEP_ANGULAR_THRESHOLD = 0.05
_SLEEP_TIME = 0.5
_MAX_SUBSTEPS = 5
_EPSILON = 1e-9

# Stop iterating the sequential-impulse solver once the largest impulse change
# in a whole sweep drops below this (its units are impulse, i.e. mass * speed).
_SOLVER_EPSILON = 1e-4

Vector = np.ndarray
Matrix3 = np.ndarray
Vec3 = tuple[float, float, float]

ZERO3: Vector = np.zeros(3)

# The eight vertex sign combinations of a unit box, used to build box corners
# with a single broadcast/matmul instead of a Python triple loop.
_CORNER_SIGNS = np.array(
    [[sx, sy, sz] for sx in (-1.0, 1.0) for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)],
    dtype=np.float64,
)


def _dot(a: Vector, b: Vector) -> float:
    """Fast dot product for two 3-vectors."""
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


# Scalar (tuple-based) 3-vector helpers for the box-box narrowphase.  NumPy is
# ideal for whole-array maths, but the box-box SAT touches only a few 3-vectors
# per pair and NumPy's per-operation dispatch dwarfs the arithmetic there, so
# plain tuples are both faster and allocation-free.
def _v_dot(u: Vec3, v: Vec3) -> float:
    """Dot product of two scalar 3-vectors."""
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def _v_cross(u: Vec3, v: Vec3) -> Vec3:
    """Cross product of two scalar 3-vectors."""
    return (
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    )


def _v_norm(u: Vec3) -> float:
    """Euclidean length of a scalar 3-vector."""
    return math.sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2])


# ---------------------------------------------------------------------------
# Small vector / quaternion helpers (kept local so the module is self-contained)
# ---------------------------------------------------------------------------
def _cross_batch(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Cross product for ``(N, 3)`` arrays.

    Hand-written because :func:`numpy.cross` routes through ``moveaxis`` /
    ``normalize_axis_tuple`` and is far slower than three broadcast products.
    """
    out = np.empty_like(a)
    out[:, 0] = a[:, 1] * b[:, 2] - a[:, 2] * b[:, 1]
    out[:, 1] = a[:, 2] * b[:, 0] - a[:, 0] * b[:, 2]
    out[:, 2] = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    return out


def _quat_integrate_batch(quat: np.ndarray, omega: np.ndarray, dt: float) -> np.ndarray:
    """Integrate an ``(N, 4)`` quaternion array by an ``(N, 3)`` angular velocity."""
    wx, wy, wz = omega[:, 0], omega[:, 1], omega[:, 2]
    qx, qy, qz, qw = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    delta = np.empty_like(quat)
    delta[:, 0] = 0.5 * (wx * qw + wy * qz - wz * qy)
    delta[:, 1] = 0.5 * (-wx * qz + wy * qw + wz * qx)
    delta[:, 2] = 0.5 * (wx * qy - wy * qx + wz * qw)
    delta[:, 3] = 0.5 * (-wx * qx - wy * qy - wz * qz)
    result = quat + delta * dt
    norm = np.linalg.norm(result, axis=1, keepdims=True)
    norm[norm < _EPSILON] = 1.0
    return result / norm


def _quat_normalize(quat: Vector) -> Vector:
    """Return *quat* as a unit quaternion ``[x, y, z, w]``."""
    norm = float(np.dot(quat, quat))
    if norm < _EPSILON:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return quat / math.sqrt(norm)


def _quat_to_matrix(quat: Vector) -> Matrix3:
    """Convert a quaternion to a 3x3 rotation matrix (``world = R @ local``)."""
    x, y, z, w = quat
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def _quat_multiply(left: Vector, right: Vector) -> Vector:
    """Multiply two quaternions ``[x, y, z, w]``."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return np.array(
        [
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
            lw * rw - lx * rx - ly * ry - lz * rz,
        ],
        dtype=np.float64,
    )


def _quat_integrate(quat: Vector, omega: Vector, dt: float) -> Vector:
    """Advance *quat* by the world-space angular velocity *omega* for *dt*."""
    if float(np.dot(omega, omega)) < _EPSILON:
        return quat
    spin = np.array([omega[0], omega[1], omega[2], 0.0], dtype=np.float64)
    delta = 0.5 * _quat_multiply(spin, quat) * dt
    return _quat_normalize(quat + delta)


# ---------------------------------------------------------------------------
# Shapes -- local-space collision geometry
# ---------------------------------------------------------------------------
@dataclass
class SphereShape:
    """A sphere centred on ``center`` (local space)."""

    radius: float = 0.5
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def local_half_extents(self) -> tuple[float, float, float]:
        """Half sizes of the local axis-aligned bounding box."""
        return (self.radius, self.radius, self.radius)


@dataclass
class BoxShape:
    """An oriented box centred on ``center`` (local space)."""

    half_extents: tuple[float, float, float] = (0.5, 0.5, 0.5)
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def local_half_extents(self) -> tuple[float, float, float]:
        """Half sizes of the local axis-aligned bounding box."""
        hx, hy, hz = self.half_extents
        return (abs(hx), abs(hy), abs(hz))


@dataclass
class CapsuleShape:
    """A capsule aligned along the object's local Z axis, centred on ``center``."""

    radius: float = 0.5
    half_height: float = 0.5
    center: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def local_half_extents(self) -> tuple[float, float, float]:
        """Half sizes of the local axis-aligned bounding box."""
        return (self.radius, self.radius, self.half_height + self.radius)

    def endpoints(self, position: Vector, rotation: Matrix3) -> tuple[Vector, Vector]:
        """World-space centres of the two spherical caps."""
        axis = rotation[:, 2] * self.half_height
        return position - axis, position + axis


@dataclass
class PlaneShape:
    """An infinite plane through the object origin with a local normal."""

    normal: tuple[float, float, float] = (0.0, 0.0, 1.0)

    def local_half_extents(self) -> tuple[float, float, float]:
        """Planes are infinite; return a very large box for broadphase use."""
        big = 1e9
        return (big, big, big)


Shape = SphereShape | BoxShape | CapsuleShape | PlaneShape


def shape_inverse_inertia(shape: Shape, mass: float) -> Vector:
    """Return the diagonal inverse inertia tensor for *shape* and *mass*.

    A non-positive *mass* yields a zero vector (a static, immovable body).
    """
    if mass <= 0:
        return ZERO3.copy()
    if isinstance(shape, SphereShape):
        inertia = np.full(3, 0.4 * mass * shape.radius * shape.radius)
    elif isinstance(shape, BoxShape):
        hx, hy, hz = shape.half_extents
        inertia = np.array(
            [
                mass * (hy * hy + hz * hz) / 3.0,
                mass * (hx * hx + hz * hz) / 3.0,
                mass * (hx * hx + hy * hy) / 3.0,
            ]
        )
    elif isinstance(shape, CapsuleShape):
        r = shape.radius
        total_height = 2.0 * (shape.half_height + r)
        perpendicular = mass * (3.0 * r * r + total_height * total_height) / 12.0
        axis = 0.5 * mass * r * r
        inertia = np.array([perpendicular, perpendicular, axis])
    else:
        return ZERO3.copy()
    return np.array(
        [
            1.0 / max(inertia[0], _EPSILON),
            1.0 / max(inertia[1], _EPSILON),
            1.0 / max(inertia[2], _EPSILON),
        ],
        dtype=np.float64,
    )


# ---------------------------------------------------------------------------
# Rigid body
# ---------------------------------------------------------------------------
class RigidBody:
    """Dynamic state of a single :class:`~payton.scene.geometry.base.Object`.

    The engine keeps one body per collidable object.  Contacts are stored on the
    body itself (so they survive between steps, enabling warm starting).
    """

    _next_id = 0

    def __init__(
        self,
        obj: "Object",
        shape: Shape,
        position: Vector,
        orientation: Vector,
        mass: float,
        linear_velocity: Vector,
        restitution: float = DEFAULT_RESTITUTION,
        friction: float = DEFAULT_FRICTION,
        linear_damping: float = DEFAULT_LINEAR_DAMPING,
        angular_damping: float = DEFAULT_ANGULAR_DAMPING,
    ) -> None:
        """Create a body from an object's current parameters.

        ``position`` is the object's local origin.  Collision shapes may carry a
        local ``center`` offset (e.g. a box built from an off-centre mesh), so
        the body stores its centre of mass as ``position + R @ center`` and the
        world converts back when writing the transform onto the object.
        """
        self.obj = obj
        self.shape = shape
        self.mass = float(mass)
        self.static = self.mass <= 0
        self.orientation = _quat_normalize(np.asarray(orientation, dtype=np.float64))
        self.linear_velocity = np.asarray(linear_velocity, dtype=np.float64)
        self.angular_velocity = ZERO3.copy()
        self.restitution = float(restitution)
        self.friction = float(friction)
        self.linear_damping = float(linear_damping)
        self.angular_damping = float(angular_damping)
        self.inv_mass = 0.0 if self.static else 1.0 / self.mass
        self.inverse_inertia = shape_inverse_inertia(shape, self.mass)
        self.sleeping = False
        self.sleep_time = 0.0
        self.local_center = np.asarray(
            getattr(shape, "center", (0.0, 0.0, 0.0)), dtype=np.float64
        )
        self._half = np.array(shape.local_half_extents(), dtype=np.float64)
        self._rotation = np.identity(3)
        self._inv_inertia_world = np.zeros((3, 3))
        self._update_world_inertia()
        self.position = np.asarray(position, dtype=np.float64) + (
            self._rotation @ self.local_center
        )
        # Bookkeeping used by the world.
        self._id = RigidBody._next_id
        RigidBody._next_id += 1
        self._index = -1
        self._moved = False

    # -- derived state ----------------------------------------------------
    @property
    def rotation(self) -> Matrix3:
        """Current orientation as a 3x3 rotation matrix (cached)."""
        return self._rotation

    @property
    def half_extents(self) -> Vector:
        """Local half extents of the collision shape as a 3-vector (cached)."""
        return self._half

    def _update_world_inertia(self) -> None:
        """Refresh the cached rotation matrix and world inverse inertia."""
        self._rotation = _quat_to_matrix(self.orientation)
        if self.static:
            self._inv_inertia_world = np.zeros((3, 3))
            return
        self._inv_inertia_world = (
            self._rotation @ np.diag(self.inverse_inertia) @ self._rotation.T
        )

    def world_inv_inertia(self) -> Matrix3:
        """World-space inverse inertia tensor."""
        return self._inv_inertia_world

    # -- runtime editing --------------------------------------------------
    def wake(self) -> None:
        """Wake the body so it is simulated again."""
        self.sleeping = False
        self.sleep_time = 0.0

    def sleep(self) -> None:
        """Put the body to sleep and clear its velocities."""
        self.sleeping = True
        self.sleep_time = 0.0
        self.linear_velocity = ZERO3.copy()
        self.angular_velocity = ZERO3.copy()

    def set_mass(self, mass: float) -> None:
        """Change the body mass and update the derived inertia."""
        self.mass = float(mass)
        self.static = self.mass <= 0
        self.inv_mass = 0.0 if self.static else 1.0 / self.mass
        self.inverse_inertia = shape_inverse_inertia(self.shape, self.mass)
        self._update_world_inertia()
        self.wake()

    def set_velocity(self, linear_velocity: "Vector | list[float]") -> None:
        """Set the linear velocity and wake the body."""
        self.linear_velocity = np.asarray(linear_velocity, dtype=np.float64)
        self.wake()

    def sync_from_object(self) -> None:
        """Teleport the body to match its object's current transform."""
        position, quat = matrix_to_position_and_quaternion(self.obj.matrix)
        self.orientation = _quat_normalize(np.asarray(quat, dtype=np.float64))
        self._update_world_inertia()
        self.position = np.asarray(position, dtype=np.float64) + (
            self._rotation @ self.local_center
        )
        self.wake()


# ---------------------------------------------------------------------------
# Narrowphase collision detection
# ---------------------------------------------------------------------------
def _closest_point_on_segment(point: Vector, a: Vector, b: Vector) -> Vector:
    """Closest point to *point* on segment ``a``-``b``."""
    ab = b - a
    length_sq = float(np.dot(ab, ab))
    if length_sq < _EPSILON:
        return a.copy()
    t = float(np.dot(point - a, ab)) / length_sq
    return a + ab * max(0.0, min(1.0, t))


def _closest_points_between_segments(
    p1: Vector, q1: Vector, p2: Vector, q2: Vector
) -> tuple[Vector, Vector]:
    """Closest points between two segments (Ericson, Real-Time Collision Detection)."""
    d1 = q1 - p1
    d2 = q2 - p2
    r = p1 - p2
    a = float(np.dot(d1, d1))
    e = float(np.dot(d2, d2))
    f = float(np.dot(d2, r))

    if a < _EPSILON and e < _EPSILON:
        return p1.copy(), p2.copy()
    if a < _EPSILON:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
    else:
        c = float(np.dot(d1, r))
        if e < _EPSILON:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = float(np.dot(d1, d2))
            denom = a * e - b * b
            if denom > _EPSILON:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))
    return p1 + d1 * s, p2 + d2 * t


def _closest_point_on_obb(
    point: Vector, center: Vector, rotation: Matrix3, half: Vector
) -> tuple[Vector, bool]:
    """Closest point on an oriented box and whether *point* is inside it."""
    local = rotation.T @ (point - center)
    clamped = np.clip(local, -half, half)
    inside = bool(np.all(np.abs(local) <= half))
    return center + rotation @ clamped, inside


def _box_corners(center: Vector, rotation: Matrix3, half: Vector) -> np.ndarray:
    """Return the eight world-space corners of an oriented box as an ``(8, 3)``."""
    return center + (_CORNER_SIGNS * half) @ rotation.T


def _sphere_sphere(a: RigidBody, b: RigidBody, ra: float, rb: float) -> list[Any]:
    """Sphere-sphere contact (``a`` and ``b`` are spheres)."""
    delta = b.position - a.position
    distance = float(np.linalg.norm(delta))
    if distance >= ra + rb:
        return []
    normal = delta / distance if distance > _EPSILON else np.array([0.0, 0.0, 1.0])
    point = a.position + normal * ra
    return [(normal, point, ra + rb - distance)]


def _sphere_box(a: RigidBody, b: RigidBody, radius: float) -> list[Any]:
    """Sphere (*a*) against box (*b*) contact; normal points sphere -> box."""
    center = b.position
    rotation = b.rotation
    half = b.half_extents
    closest, inside = _closest_point_on_obb(a.position, center, rotation, half)
    if inside:
        local = rotation.T @ (a.position - center)
        diff = half - np.abs(local)
        axis = int(np.argmin(diff))
        sign = 1.0 if local[axis] >= 0 else -1.0
        face_normal = rotation[:, axis] * sign
        local_face = local.copy()
        local_face[axis] = sign * half[axis]
        point = center + rotation @ local_face
        return [(-face_normal, point, radius + float(diff[axis]))]
    delta = a.position - closest
    distance = float(np.linalg.norm(delta))
    if distance >= radius:
        return []
    normal = -(delta / distance) if distance > _EPSILON else np.array([0.0, 0.0, 1.0])
    point = a.position + normal * radius
    return [(normal, point, radius - distance)]


def _sphere_capsule(a: RigidBody, b: RigidBody, radius: float) -> list[Any]:
    """Sphere (*a*) against capsule (*b*); normal points sphere -> capsule."""
    shape = b.shape
    assert isinstance(shape, CapsuleShape)
    start, end = shape.endpoints(b.position, b.rotation)
    closest = _closest_point_on_segment(a.position, start, end)
    delta = closest - a.position
    distance = float(np.linalg.norm(delta))
    if distance >= radius + shape.radius:
        return []
    normal = delta / distance if distance > _EPSILON else np.array([0.0, 0.0, 1.0])
    point = a.position + normal * radius
    return [(normal, point, radius + shape.radius - distance)]


def _capsule_capsule(a: RigidBody, b: RigidBody) -> list[Any]:
    """Capsule-capsule contact; normal points a -> b."""
    shape_a = a.shape
    shape_b = b.shape
    assert isinstance(shape_a, CapsuleShape)
    assert isinstance(shape_b, CapsuleShape)
    a0, a1 = shape_a.endpoints(a.position, a.rotation)
    b0, b1 = shape_b.endpoints(b.position, b.rotation)
    ca, cb = _closest_points_between_segments(a0, a1, b0, b1)
    delta = cb - ca
    distance = float(np.linalg.norm(delta))
    if distance >= shape_a.radius + shape_b.radius:
        return []
    normal = delta / distance if distance > _EPSILON else np.array([0.0, 0.0, 1.0])
    point = (ca + cb) * 0.5
    return [(normal, point, shape_a.radius + shape_b.radius - distance)]


def _capsule_box(a: RigidBody, b: RigidBody) -> list[Any]:
    """Capsule (*a*) against box (*b*); normal points capsule -> box.

    The closest point on the box to the capsule's core segment is found with a
    ternary search (the distance function is convex in the segment parameter),
    then the capsule radius is subtracted.  The search runs on plain Python
    floats: doing it with NumPy 3-vectors would allocate a dozen tiny arrays per
    probe and dominate the whole step for capsule-heavy scenes.
    """
    shape = a.shape
    assert isinstance(shape, CapsuleShape)
    start, end = shape.endpoints(a.position, a.rotation)
    radius = shape.radius
    center = b.position
    rotation = b.rotation
    half = b.half_extents

    # Box frame / segment as scalars, hoisted out of the search loop.
    cx, cy, cz = float(center[0]), float(center[1]), float(center[2])
    ax0, ay0, az0 = float(rotation[0, 0]), float(rotation[1, 0]), float(rotation[2, 0])
    ax1, ay1, az1 = float(rotation[0, 1]), float(rotation[1, 1]), float(rotation[2, 1])
    ax2, ay2, az2 = float(rotation[0, 2]), float(rotation[1, 2]), float(rotation[2, 2])
    hx, hy, hz = abs(float(half[0])), abs(float(half[1])), abs(float(half[2]))
    sx, sy, sz = float(start[0]), float(start[1]), float(start[2])
    ex, ey, ez = float(end[0]), float(end[1]), float(end[2])
    dx, dy, dz = ex - sx, ey - sy, ez - sz

    def sq_distance(t: float) -> float:
        px = sx + dx * t - cx
        py = sy + dy * t - cy
        pz = sz + dz * t - cz
        # Local coordinates: dot with the world axes (columns of the rotation).
        lx = ax0 * px + ay0 * py + az0 * pz
        ly = ax1 * px + ay1 * py + az1 * pz
        lz = ax2 * px + ay2 * py + az2 * pz
        qx = min(hx, max(lx, -hx))
        qy = min(hy, max(ly, -hy))
        qz = min(hz, max(lz, -hz))
        return (lx - qx) ** 2 + (ly - qy) ** 2 + (lz - qz) ** 2

    low, high = 0.0, 1.0
    for _ in range(40):
        m1 = low + (high - low) / 3.0
        m2 = high - (high - low) / 3.0
        if sq_distance(m1) < sq_distance(m2):
            high = m2
        else:
            low = m1
    closest = start + (end - start) * ((low + high) * 0.5)
    on_box, inside = _closest_point_on_obb(closest, center, rotation, half)
    delta = on_box - closest
    distance = float(np.linalg.norm(delta))
    if inside or distance >= radius or distance < _EPSILON:
        return []
    normal = delta / distance
    point = closest + normal * radius
    return [(normal, point, radius - distance)]


def _obb_overlap(
    t: Vec3, axis: Vec3, a_axes: list[Vec3], ha: Vec3, b_axes: list[Vec3], hb: Vec3
) -> float:
    """Projection overlap of two boxes along *axis* (negative means separated)."""
    ra = (
        ha[0] * abs(_v_dot(a_axes[0], axis))
        + ha[1] * abs(_v_dot(a_axes[1], axis))
        + ha[2] * abs(_v_dot(a_axes[2], axis))
    )
    rb = (
        hb[0] * abs(_v_dot(b_axes[0], axis))
        + hb[1] * abs(_v_dot(b_axes[1], axis))
        + hb[2] * abs(_v_dot(b_axes[2], axis))
    )
    return ra + rb - abs(_v_dot(t, axis))


def _box_axes(rotation: Matrix3) -> list[Vec3]:
    """The three world-space box axes (columns of the rotation matrix)."""
    return [
        (float(rotation[0, 0]), float(rotation[1, 0]), float(rotation[2, 0])),
        (float(rotation[0, 1]), float(rotation[1, 1]), float(rotation[2, 1])),
        (float(rotation[0, 2]), float(rotation[1, 2]), float(rotation[2, 2])),
    ]


def _support_face_vertices(
    center: Vec3, axes: list[Vec3], half: Vec3, axis_index: int, sign: float
) -> list[Vec3]:
    """The four world-space corners of the face along *axis_index*."""
    hx, hy, hz = half
    (a0, a1, a2) = axes
    cx, cy, cz = center
    vertices: list[Vec3] = []
    if axis_index == 0:
        fx = sign * hx
        for sy, sz in ((-hy, -hz), (-hy, hz), (hy, -hz), (hy, hz)):
            vertices.append(
                (
                    cx + a0[0] * fx + a1[0] * sy + a2[0] * sz,
                    cy + a0[1] * fx + a1[1] * sy + a2[1] * sz,
                    cz + a0[2] * fx + a1[2] * sy + a2[2] * sz,
                )
            )
    elif axis_index == 1:
        fy = sign * hy
        for sx, sz in ((-hx, -hz), (-hx, hz), (hx, -hz), (hx, hz)):
            vertices.append(
                (
                    cx + a0[0] * sx + a1[0] * fy + a2[0] * sz,
                    cy + a0[1] * sx + a1[1] * fy + a2[1] * sz,
                    cz + a0[2] * sx + a1[2] * fy + a2[2] * sz,
                )
            )
    else:
        fz = sign * hz
        for sx, sy in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)):
            vertices.append(
                (
                    cx + a0[0] * sx + a1[0] * sy + a2[0] * fz,
                    cy + a0[1] * sx + a1[1] * sy + a2[1] * fz,
                    cz + a0[2] * sx + a1[2] * sy + a2[2] * fz,
                )
            )
    return vertices


def _clip_polygon(points: list[Vec3], plane_normal: Vec3, offset: float) -> list[Vec3]:
    """Clip a convex polygon keeping the region ``dot(n, p) <= offset``."""
    if not points:
        return []
    result: list[Vec3] = []
    count = len(points)
    for i in range(count):
        current = points[i]
        following = points[(i + 1) % count]
        d_current = _v_dot(plane_normal, current) - offset
        d_following = _v_dot(plane_normal, following) - offset
        if d_current <= 0.0:
            result.append(current)
        if (d_current < 0.0 < d_following) or (d_following < 0.0 < d_current):
            t = d_current / (d_current - d_following)
            result.append(
                (
                    current[0] + (following[0] - current[0]) * t,
                    current[1] + (following[1] - current[1]) * t,
                    current[2] + (following[2] - current[2]) * t,
                )
            )
    return result


def _edge_segment(
    center: Vec3, axes: list[Vec3], half: Vec3, edge_index: int, toward: Vec3
) -> tuple[Vec3, Vec3]:
    """The box edge parallel to *edge_index* that faces *toward*."""
    if edge_index == 0:
        k1, k2 = 1, 2
    elif edge_index == 1:
        k1, k2 = 0, 2
    else:
        k1, k2 = 0, 1
    ox = oy = oz = 0.0
    for k in (k1, k2):
        sign = 1.0 if _v_dot(axes[k], toward) >= 0.0 else -1.0
        scale = half[k] * sign
        ox += axes[k][0] * scale
        oy += axes[k][1] * scale
        oz += axes[k][2] * scale
    base = (center[0] + ox, center[1] + oy, center[2] + oz)
    dx = axes[edge_index][0] * half[edge_index]
    dy = axes[edge_index][1] * half[edge_index]
    dz = axes[edge_index][2] * half[edge_index]
    return (
        (base[0] - dx, base[1] - dy, base[2] - dz),
        (base[0] + dx, base[1] + dy, base[2] + dz),
    )


def _box_box(a: RigidBody, b: RigidBody) -> list[Any]:
    """Oriented box against oriented box (SAT plus reference-face clipping)."""
    a_axes = _box_axes(a.rotation)
    b_axes = _box_axes(b.rotation)
    ha: Vec3 = (
        float(a.half_extents[0]),
        float(a.half_extents[1]),
        float(a.half_extents[2]),
    )
    hb: Vec3 = (
        float(b.half_extents[0]),
        float(b.half_extents[1]),
        float(b.half_extents[2]),
    )
    a_pos: Vec3 = (float(a.position[0]), float(a.position[1]), float(a.position[2]))
    b_pos: Vec3 = (float(b.position[0]), float(b.position[1]), float(b.position[2]))
    t = (b_pos[0] - a_pos[0], b_pos[1] - a_pos[1], b_pos[2] - a_pos[2])

    best_overlap = math.inf
    best_kind = "face"
    best_ref = "a"
    best_index = 0
    best_sign = 1.0

    def consider(axis: Vec3, kind: str, ref: str, index: int) -> bool:
        nonlocal best_overlap, best_kind, best_ref, best_index, best_sign
        length = _v_norm(axis)
        if length < 1e-8:
            return True
        axis = (axis[0] / length, axis[1] / length, axis[2] / length)
        sign = 1.0 if _v_dot(axis, t) >= 0.0 else -1.0
        oriented = (axis[0] * sign, axis[1] * sign, axis[2] * sign)
        overlap = _obb_overlap(t, oriented, a_axes, ha, b_axes, hb)
        if overlap < 0.0:
            return False
        # Use a small tolerance so a cross-product axis that merely duplicates a
        # face axis (a common degenerate case for aligned boxes) never wins a
        # floating-point tie and produces a wrong, destabilising normal.
        if overlap < best_overlap - 1e-6:
            best_overlap = overlap
            best_kind = kind
            best_ref = ref
            best_index = index
            best_sign = sign
        return True

    a0, a1, a2 = a_axes
    b0, b1, b2 = b_axes
    if not consider(a0, "face", "a", 0):
        return []
    if not consider(a1, "face", "a", 1):
        return []
    if not consider(a2, "face", "a", 2):
        return []
    if not consider(b0, "face", "b", 0):
        return []
    if not consider(b1, "face", "b", 1):
        return []
    if not consider(b2, "face", "b", 2):
        return []
    if not consider(_v_cross(a0, b0), "edge", "a", 0):
        return []
    if not consider(_v_cross(a0, b1), "edge", "a", 1):
        return []
    if not consider(_v_cross(a0, b2), "edge", "a", 2):
        return []
    if not consider(_v_cross(a1, b0), "edge", "a", 3):
        return []
    if not consider(_v_cross(a1, b1), "edge", "a", 4):
        return []
    if not consider(_v_cross(a1, b2), "edge", "a", 5):
        return []
    if not consider(_v_cross(a2, b0), "edge", "a", 6):
        return []
    if not consider(_v_cross(a2, b1), "edge", "a", 7):
        return []
    if not consider(_v_cross(a2, b2), "edge", "a", 8):
        return []

    if best_kind == "edge":
        i = best_index // 3
        j = best_index % 3
        a0, a1 = _edge_segment(a_pos, a_axes, ha, i, t)
        b0, b1 = _edge_segment(b_pos, b_axes, hb, j, (-t[0], -t[1], -t[2]))
        ca, cb = _closest_points_between_segments(
            np.asarray(a0), np.asarray(a1), np.asarray(b0), np.asarray(b1)
        )
        delta = cb - ca
        distance = float(np.linalg.norm(delta))
        if distance > _EPSILON:
            normal: Any = tuple(delta / distance)
        else:
            normal = (
                a_axes[i][0] * best_sign,
                a_axes[i][1] * best_sign,
                a_axes[i][2] * best_sign,
            )
        midpoint = tuple((ca + cb) * 0.5)
        return [(normal, midpoint, best_overlap)]

    if best_ref == "a":
        ref_center, ref_axes, ref_half = a_pos, a_axes, ha
        inc_center, inc_axes, inc_half = b_pos, b_axes, hb
        ref_normal = (
            a_axes[best_index][0] * best_sign,
            a_axes[best_index][1] * best_sign,
            a_axes[best_index][2] * best_sign,
        )
        flip = False
    else:
        ref_center, ref_axes, ref_half = b_pos, b_axes, hb
        inc_center, inc_axes, inc_half = a_pos, a_axes, ha
        # Reference normal points from the reference box towards the other box.
        ref_normal = (
            b_axes[best_index][0] * (-best_sign),
            b_axes[best_index][1] * (-best_sign),
            b_axes[best_index][2] * (-best_sign),
        )
        flip = True
    inv_len = 1.0 / _v_norm(ref_normal)
    ref_normal = (
        ref_normal[0] * inv_len,
        ref_normal[1] * inv_len,
        ref_normal[2] * inv_len,
    )
    face_offset = _v_dot(ref_normal, ref_center) + ref_half[best_index]

    d0 = _v_dot(inc_axes[0], ref_normal)
    d1 = _v_dot(inc_axes[1], ref_normal)
    d2 = _v_dot(inc_axes[2], ref_normal)
    inc_index = 0
    inc_abs = abs(d0)
    if abs(d1) > inc_abs:
        inc_index = 1
        inc_abs = abs(d1)
    if abs(d2) > inc_abs:
        inc_index = 2
    inc_sign = -1.0 if (d0, d1, d2)[inc_index] > 0.0 else 1.0
    polygon = _support_face_vertices(
        inc_center, inc_axes, inc_half, inc_index, inc_sign
    )

    if best_index == 0:
        clip_axes = (1, 2)
    elif best_index == 1:
        clip_axes = (0, 2)
    else:
        clip_axes = (0, 1)
    for k in clip_axes:
        side = ref_axes[k]
        center_offset = _v_dot(side, ref_center)
        polygon = _clip_polygon(polygon, side, center_offset + ref_half[k])
        polygon = _clip_polygon(
            polygon, (-side[0], -side[1], -side[2]), -center_offset + ref_half[k]
        )
        if len(polygon) < 3:
            break

    # Keep at most the four most extreme points so each pair contributes a
    # compact manifold (faster and numerically calmer than long collinear rows).
    if len(polygon) > 4:
        count = len(polygon)
        cx = sum(p[0] for p in polygon) / count
        cy = sum(p[1] for p in polygon) / count
        cz = sum(p[2] for p in polygon) / count
        polygon = sorted(
            polygon,
            key=lambda p: -((p[0] - cx) ** 2 + (p[1] - cy) ** 2 + (p[2] - cz) ** 2),
        )[:4]

    contacts: list[Any] = []
    out_normal = (
        (-ref_normal[0], -ref_normal[1], -ref_normal[2]) if flip else ref_normal
    )
    for point in polygon:
        separation = _v_dot(ref_normal, point) - face_offset
        if separation <= 1e-4:
            contacts.append((out_normal, point, max(-separation, 0.0)))
    if not contacts:
        contacts.append((out_normal, inc_center, best_overlap))
    return contacts


def _plane_contacts(plane: RigidBody, other: RigidBody) -> list[Any]:
    """Contacts between an infinite *plane* and *other*.

    Normals point from the plane towards *other*.
    """
    shape = plane.shape
    assert isinstance(shape, PlaneShape)
    normal = plane.rotation @ np.asarray(shape.normal)
    norm = float(np.linalg.norm(normal))
    if norm < _EPSILON:
        return []
    normal = normal / norm
    origin = plane.position
    contacts: list[Any] = []

    def add_sphere(center: Vector, radius: float) -> None:
        signed = _dot(normal, center - origin)
        if signed < radius:
            point = center - normal * radius
            contacts.append((normal, point, radius - signed))

    other_shape = other.shape
    if isinstance(other_shape, SphereShape):
        add_sphere(other.position, other_shape.radius)
    elif isinstance(other_shape, CapsuleShape):
        start, end = other_shape.endpoints(other.position, other.rotation)
        add_sphere(start, other_shape.radius)
        add_sphere(end, other_shape.radius)
    elif isinstance(other_shape, BoxShape):
        corners = _box_corners(other.position, other.rotation, other.half_extents)
        signed = corners @ normal - float(np.dot(normal, origin))
        below = signed < 0.0
        for corner, penetration in zip(corners[below], -signed[below]):
            contacts.append((normal, corner, float(penetration)))
    return contacts


def _flip(contacts: list[Any]) -> list[Any]:
    """Reverse contact normals (used when swapping the two bodies)."""
    return [(-normal, point, penetration) for normal, point, penetration in contacts]


def _collide_convex(a: RigidBody, b: RigidBody) -> list[Any]:
    """Dispatch convex-convex contacts; normals point a -> b."""
    sa = a.shape
    sb = b.shape
    if isinstance(sa, BoxShape) and isinstance(sb, BoxShape):
        return _box_box(a, b)
    if isinstance(sa, SphereShape) and isinstance(sb, SphereShape):
        return _sphere_sphere(a, b, sa.radius, sb.radius)
    if isinstance(sa, SphereShape) and isinstance(sb, BoxShape):
        return _sphere_box(a, b, sa.radius)
    if isinstance(sa, BoxShape) and isinstance(sb, SphereShape):
        return _flip(_sphere_box(b, a, sb.radius))
    if isinstance(sa, SphereShape) and isinstance(sb, CapsuleShape):
        return _sphere_capsule(a, b, sa.radius)
    if isinstance(sa, CapsuleShape) and isinstance(sb, SphereShape):
        return _flip(_sphere_capsule(b, a, sb.radius))
    if isinstance(sa, CapsuleShape) and isinstance(sb, CapsuleShape):
        return _capsule_capsule(a, b)
    if isinstance(sa, CapsuleShape) and isinstance(sb, BoxShape):
        return _capsule_box(a, b)
    if isinstance(sa, BoxShape) and isinstance(sb, CapsuleShape):
        return _flip(_capsule_box(b, a))
    return []


def _collide(a: RigidBody, b: RigidBody) -> list[Any]:
    """Return raw contacts between two bodies (normals point a -> b)."""
    sa = a.shape
    sb = b.shape
    if isinstance(sa, PlaneShape) and isinstance(sb, PlaneShape):
        return []
    if isinstance(sa, PlaneShape):
        return _plane_contacts(a, b)
    if isinstance(sb, PlaneShape):
        return _flip(_plane_contacts(b, a))
    return _collide_convex(a, b)


# ---------------------------------------------------------------------------
# Persistent contacts
#
# Unlike the old engine -- which rebuilt every contact and discarded all
# impulses each step -- the world keeps a manifold per overlapping body pair.
# Contact points are matched to the previous step and their accumulated
# impulses are reused ("warm starting"), which is what lets stacks and deeply
# overlapping piles settle instead of sinking.  The core algorithms are
# inspired by Randy Gaul's qu3e / Erin Catto's sequential-impulse solver.
# ---------------------------------------------------------------------------
_AABB_MARGIN = 0.05
_WARM_START_DISTANCE_SQ = 0.02 * 0.02


class _Contact:
    """A single contact point with its accumulated impulse."""

    __slots__ = (
        "normal_impulse",
        "penetration",
        "position",
        "t0_impulse",
        "t1_impulse",
    )

    def __init__(self, position: Vector, penetration: float) -> None:
        self.position = np.asarray(position, dtype=np.float64)
        self.penetration = float(penetration)
        self.normal_impulse = 0.0
        self.t0_impulse = 0.0
        self.t1_impulse = 0.0


class _Manifold:
    """Persistent contact constraint between two bodies."""

    __slots__ = ("a", "b", "contacts", "friction", "normal", "restitution", "t0", "t1")

    def __init__(self, a: RigidBody, b: RigidBody) -> None:
        self.a = a
        self.b = b
        self.normal = np.array([0.0, 0.0, 1.0])
        self.t0 = np.array([1.0, 0.0, 0.0])
        self.t1 = np.array([0.0, 1.0, 0.0])
        self.contacts: list[_Contact] = []
        self.friction = 0.0
        self.restitution = 0.0

    def refresh(self) -> None:
        """(Re)derive material properties from the two bodies."""
        a, b = self.a, self.b
        self.friction = math.sqrt(max(a.friction, 0.0) * max(b.friction, 0.0))
        self.restitution = max(a.restitution, b.restitution)


def _tangent_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return two orthonormal tangent directions for each normal in ``(m, 3)``."""
    absn = np.abs(normal)
    axis = np.argmin(absn, axis=1)
    up = np.zeros_like(normal)
    up[np.arange(normal.shape[0]), axis] = 1.0
    t0 = _cross_batch(normal, up)
    t0 /= np.maximum(np.linalg.norm(t0, axis=1, keepdims=True), _EPSILON)
    t1 = _cross_batch(normal, t0)
    return t0, t1


def _color_contacts(
    a_idx: np.ndarray, b_idx: np.ndarray, movable: np.ndarray
) -> list[np.ndarray]:
    """Greedy-color contacts so no two in a color share a *movable* body.

    Within a color no movable body appears in more than one contact, so the
    whole color can be solved with vectorised array maths and direct scatter
    (immovable bodies may repeat -- a zero-mass impulse is a no-op).
    """
    contacts = a_idx.size
    a_list = a_idx.tolist()
    b_list = b_idx.tolist()
    adjacency: dict[int, list[int]] = {}
    for c in range(contacts):
        ac = a_list[c]
        if movable[ac]:
            adjacency.setdefault(ac, []).append(c)
        bc = b_list[c]
        if movable[bc]:
            adjacency.setdefault(bc, []).append(c)

    color_of = [0] * contacts
    for c in range(contacts):
        used = 0
        for other in adjacency.get(a_list[c], ()):
            if other < c:
                used |= 1 << color_of[other]
        for other in adjacency.get(b_list[c], ()):
            if other < c:
                used |= 1 << color_of[other]
        color_of[c] = ((~used) & (used + 1)).bit_length() - 1

    color_count = max(color_of) + 1 if color_of else 0
    color_arr = np.asarray(color_of, dtype=np.intp)
    order = np.argsort(color_arr, kind="stable")
    boundaries = np.searchsorted(color_arr[order], np.arange(color_count + 1))
    return [order[boundaries[k] : boundaries[k + 1]] for k in range(color_count)]


def _solve_color(
    data: tuple[Any, ...],
    linear: np.ndarray,
    angular: np.ndarray,
    normal_impulse: np.ndarray,
    t0_impulse: np.ndarray,
    t1_impulse: np.ndarray,
) -> float:
    """Solve one color in parallel; return the largest impulse applied."""
    (
        ia,
        ib,
        n,
        t0,
        t1,
        ra,
        rb,
        im_a,
        im_b,
        mass_n,
        mass_t0,
        mass_t1,
        bias,
        friction,
        ang_n_a,
        ang_n_b,
        ang_t0_a,
        ang_t0_b,
        ang_t1_a,
        ang_t1_b,
        color,
    ) = data
    if ia.size == 0:
        return 0.0

    va = linear[ia]
    vb = linear[ib]
    wa = angular[ia]
    wb = angular[ib]
    max_impulse = 0.0

    # --- friction (two tangential directions) ---
    rv = (vb + _cross_batch(wb, rb)) - (va + _cross_batch(wa, ra))
    for t, tmass, ang_a, ang_b, imp in (
        (t0, mass_t0, ang_t0_a, ang_t0_b, t0_impulse),
        (t1, mass_t1, ang_t1_a, ang_t1_b, t1_impulse),
    ):
        vt = np.einsum("mi,mi->m", rv, t)
        lam = -vt / tmass
        limit = friction * normal_impulse[color]
        old = imp[color]
        new = np.clip(old + lam, -limit, limit)
        lam = new - old
        imp[color] = new
        max_impulse = max(max_impulse, float(np.max(np.abs(lam))))
        va -= im_a * lam[:, None] * t
        wa -= lam[:, None] * ang_a
        vb += im_b * lam[:, None] * t
        wb += lam[:, None] * ang_b

    # --- normal ---
    rv = (vb + _cross_batch(wb, rb)) - (va + _cross_batch(wa, ra))
    vn = np.einsum("mi,mi->m", rv, n)
    lam = (-vn + bias) / mass_n
    old = normal_impulse[color]
    new = np.maximum(old + lam, 0.0)
    lam = new - old
    normal_impulse[color] = new
    max_impulse = max(max_impulse, float(np.max(np.abs(lam))))
    va -= im_a * lam[:, None] * n
    wa -= lam[:, None] * ang_n_a
    vb += im_b * lam[:, None] * n
    wb += lam[:, None] * ang_n_b

    linear[ia] = va
    linear[ib] = vb
    angular[ia] = wa
    angular[ib] = wb
    return max_impulse


# ---------------------------------------------------------------------------
# Simulation world
# ---------------------------------------------------------------------------
class InternalPhysicsWorld:
    """A small, dependency-free rigid-body simulation.

    Contacts persist between steps and their impulses are warm started, so
    resting stacks stay put and deep overlap is pushed apart instead of being
    frozen in place.

    Parameters
    ----------
    gravity : tuple of float, optional
        Gravity vector.  Defaults to ``(0, 0, -9.8)``.
    time_step : float, optional
        Fixed simulation step in seconds.  Defaults to ``1 / 120``.
    solver_iterations : int, optional
        Sequential-impulse iterations per step.  Defaults to 20.
    enable_sleeping : bool, optional
        Put resting bodies to sleep to save work.  Defaults to True.
    """

    def __init__(
        self,
        gravity: tuple[float, float, float] = DEFAULT_GRAVITY,
        time_step: float = DEFAULT_TIME_STEP,
        solver_iterations: int = DEFAULT_SOLVER_ITERATIONS,
        enable_sleeping: bool = True,
    ) -> None:
        self.gravity = np.asarray(gravity, dtype=np.float64)
        self.time_step = float(time_step)
        self.solver_iterations = max(1, int(solver_iterations))
        self.enable_sleeping = enable_sleeping
        self.bodies: list[RigidBody] = []
        self._manifolds: dict[tuple[int, int], _Manifold] = {}
        self._lock = threading.Lock()
        self._accumulator = 0.0
        self._warned_constraints = False

    # -- membership -------------------------------------------------------
    def add_object(self, obj: "Object") -> RigidBody | None:
        """Register *obj* and return its body (or ``None`` if not collidable)."""
        shape = obj._physics_shape()
        if shape is None:
            return None
        position, quat = matrix_to_position_and_quaternion(obj.matrix)
        dynamics = getattr(obj, "_bullet_dynamics", {}) or {}
        body = RigidBody(
            obj=obj,
            shape=shape,
            position=np.asarray(position, dtype=np.float64),
            orientation=np.asarray(quat, dtype=np.float64),
            mass=obj.mass,
            linear_velocity=np.asarray(obj.linear_velocity, dtype=np.float64),
            restitution=float(dynamics.get("restitution", DEFAULT_RESTITUTION)),
            friction=float(dynamics.get("lateralFriction", DEFAULT_FRICTION)),
            linear_damping=float(dynamics.get("linearDamping", DEFAULT_LINEAR_DAMPING)),
            angular_damping=float(
                dynamics.get("angularDamping", DEFAULT_ANGULAR_DAMPING)
            ),
        )
        with self._lock:
            self.bodies.append(body)
            self._reindex()
        obj._internal_body = body
        constraints = getattr(obj, "_bullet_constraints", [])
        if constraints and not self._warned_constraints:
            self._warned_constraints = True
            logger.warning(
                "constraint_point/point-to-point joints are a PyBullet-only "
                "feature and are ignored by the built-in physics engine."
            )
        return body

    def remove_object(self, obj: "Object") -> None:
        """Unregister *obj* if it was previously added."""
        with self._lock:
            self.bodies = [body for body in self.bodies if body.obj is not obj]
            self._reindex()
            for key in [
                k
                for k, m in self._manifolds.items()
                if m.a.obj is obj or m.b.obj is obj
            ]:
                del self._manifolds[key]
        if getattr(obj, "_internal_body", None) is not None:
            obj._internal_body = None

    def clear(self) -> None:
        """Remove every body from the world."""
        with self._lock:
            for body in self.bodies:
                body.obj._internal_body = None
            self.bodies = []
            self._manifolds.clear()
        self._accumulator = 0.0

    def _reindex(self) -> None:
        for i, body in enumerate(self.bodies):
            body._index = i

    # -- simulation -------------------------------------------------------
    def step(self, dt: float | None = None) -> None:
        """Advance the simulation by *dt* seconds using fixed sub-steps."""
        if dt is None:
            dt = self.time_step
        self._accumulator += dt
        substeps = 0
        while self._accumulator >= self.time_step and substeps < _MAX_SUBSTEPS:
            self._step_once(self.time_step)
            self._accumulator -= self.time_step
            substeps += 1
        if substeps == _MAX_SUBSTEPS:
            self._accumulator = 0.0
        self._sync_transforms()

    # -- broadphase -------------------------------------------------------
    def _broadphase(self, bodies: list[RigidBody]) -> list[tuple[int, int]]:
        """Sweep-and-prune broadphase on fattened world AABBs."""
        count = len(bodies)
        if count < 2:
            return []
        lower = np.empty((count, 3), dtype=np.float64)
        upper = np.empty((count, 3), dtype=np.float64)
        for i, body in enumerate(bodies):
            if isinstance(body.shape, PlaneShape):
                lower[i] = -1e9
                upper[i] = 1e9
                continue
            extents = np.abs(body.rotation) @ body.half_extents + _AABB_MARGIN
            lower[i] = body.position - extents
            upper[i] = body.position + extents

        order = np.argsort(lower[:, 0])
        pairs: list[tuple[int, int]] = []
        for a in range(count):
            i = int(order[a])
            imax_x = upper[i, 0]
            for b in range(a + 1, count):
                j = int(order[b])
                if lower[j, 0] > imax_x:
                    break
                if lower[j, 1] > upper[i, 1] or upper[j, 1] < lower[i, 1]:
                    continue
                if lower[j, 2] > upper[i, 2] or upper[j, 2] < lower[i, 2]:
                    continue
                if bodies[i].static and bodies[j].static:
                    continue
                pairs.append((i, j) if i < j else (j, i))
        return pairs

    # -- contact persistence ---------------------------------------------
    def _update_manifolds(
        self, pairs: list[tuple[int, int]], bodies: list[RigidBody]
    ) -> None:
        live: set[tuple[int, int]] = set()
        for i, j in pairs:
            a, b = bodies[i], bodies[j]
            key = (a._id, b._id) if a._id < b._id else (b._id, a._id)
            live.add(key)
            manifold = self._manifolds.get(key)
            if manifold is None:
                manifold = _Manifold(a, b)
                self._manifolds[key] = manifold
            self._update_manifold(manifold)
        for key in [k for k in self._manifolds if k not in live]:
            del self._manifolds[key]

    def _update_manifold(self, manifold: _Manifold) -> None:
        """Re-run narrowphase and carry accumulated impulses to matching points."""
        old = manifold.contacts
        raw = _collide(manifold.a, manifold.b)
        new_contacts: list[_Contact] = []
        # Keep the normal sign stable for a given separating axis.  For a very
        # symmetric overlap (two crossing bars at the same height) the SAT sign
        # is ambiguous and flips frame to frame; a flipped normal would make a
        # warm-started impulse push the wrong way.  If instead the *axis*
        # changed, the old impulses are meaningless, so cold-start this step.
        prev_normal = manifold.normal
        consistent = True
        if raw:
            new_normal = np.asarray(raw[0][0], dtype=np.float64)
            alignment = float(np.dot(new_normal, prev_normal))
            if old and alignment < 0.0:
                new_normal = -new_normal
                alignment = -alignment
            consistent = not old or alignment > 0.9
            manifold.normal = new_normal
        for normal, point, penetration in raw:
            contact = _Contact(point, penetration)
            best_dist = _WARM_START_DISTANCE_SQ
            best: _Contact | None = None
            for previous in old:
                d = float(np.sum((previous.position - contact.position) ** 2))
                if d < best_dist:
                    best_dist = d
                    best = previous
            if best is not None and consistent:
                contact.normal_impulse = best.normal_impulse
                contact.t0_impulse = best.t0_impulse
                contact.t1_impulse = best.t1_impulse
            new_contacts.append(contact)
        manifold.contacts = new_contacts
        manifold.refresh()

    # -- waking -----------------------------------------------------------
    def _wake_contacts(self, bodies: list[RigidBody]) -> None:
        """Propagate motion into sleeping neighbours through the contact graph."""
        active: list[RigidBody] = []
        for body in bodies:
            if body.static or body.sleeping:
                continue
            speed = float(np.dot(body.linear_velocity, body.linear_velocity))
            spin = float(np.dot(body.angular_velocity, body.angular_velocity))
            if (
                speed > _SLEEP_LINEAR_THRESHOLD**2 * 4.0
                or spin > _SLEEP_ANGULAR_THRESHOLD**2 * 4.0
            ):
                active.append(body)
        if not active:
            return
        adjacency: dict[int, list[RigidBody]] = {}
        for manifold in self._manifolds.values():
            adjacency.setdefault(manifold.a._id, []).append(manifold.b)
            adjacency.setdefault(manifold.b._id, []).append(manifold.a)
        stack = list(active)
        woken: set[int] = set()
        while stack:
            body = stack.pop()
            for other in adjacency.get(body._id, ()):
                if other.sleeping and not other.static and other._id not in woken:
                    other.wake()
                    woken.add(other._id)
                    stack.append(other)

    # -- solver -----------------------------------------------------------
    def _step_once(self, dt: float) -> None:
        with self._lock:
            bodies = list(self.bodies)
        count = len(bodies)
        if count == 0:
            return
        self._reindex()
        pairs = self._broadphase(bodies)
        self._update_manifolds(pairs, bodies)
        self._wake_contacts(bodies)
        self._solve(dt, bodies)

    def _solve(self, dt: float, bodies: list[RigidBody]) -> None:
        static = np.array([b.static for b in bodies], dtype=bool)
        sleeping = np.array([b.sleeping for b in bodies], dtype=bool)
        position = np.array([b.position for b in bodies], dtype=np.float64)
        linear = np.array([b.linear_velocity for b in bodies], dtype=np.float64)
        angular = np.array([b.angular_velocity for b in bodies], dtype=np.float64)
        quat = np.array([b.orientation for b in bodies], dtype=np.float64)
        inv_mass = np.array([b.inv_mass for b in bodies], dtype=np.float64)
        inv_inertia = np.array(
            [b.world_inv_inertia() for b in bodies], dtype=np.float64
        )
        restitution = np.array([b.restitution for b in bodies], dtype=np.float64)
        friction = np.array([b.friction for b in bodies], dtype=np.float64)
        linear_damping = np.array([b.linear_damping for b in bodies], dtype=np.float64)
        angular_damping = np.array(
            [b.angular_damping for b in bodies], dtype=np.float64
        )
        sleep_time = np.array([b.sleep_time for b in bodies], dtype=np.float64)

        dynamic = ~static & ~sleeping
        if dynamic.any():
            linear[dynamic] += self.gravity * dt
            linear[dynamic] *= np.clip(
                1.0 - linear_damping[dynamic, None] * dt, 0.0, 1.0
            )
            angular[dynamic] *= np.clip(
                1.0 - angular_damping[dynamic, None] * dt, 0.0, 1.0
            )

        self._solve_contacts(
            dt,
            static,
            sleeping,
            inv_mass,
            inv_inertia,
            restitution,
            friction,
            position,
            linear,
            angular,
        )

        # Integrate positions and orientations.
        if dynamic.any():
            position[dynamic] += linear[dynamic] * dt
            quat[dynamic] = _quat_integrate_batch(quat[dynamic], angular[dynamic], dt)

        # Sleep test (per body; motion propagates through the contact graph).
        if self.enable_sleeping:
            linear_sq = np.einsum("ij,ij->i", linear, linear)
            angular_sq = np.einsum("ij,ij->i", angular, angular)
            still = (
                dynamic
                & (linear_sq < _SLEEP_LINEAR_THRESHOLD**2)
                & (angular_sq < _SLEEP_ANGULAR_THRESHOLD**2)
            )
            sleep_time = np.where(still, sleep_time + dt, 0.0)
            sleeping = sleeping | ((~static) & (sleep_time >= _SLEEP_TIME))
            linear[sleeping] = 0.0
            angular[sleeping] = 0.0

        # Write state back onto the bodies that moved (or changed sleep state).
        for i, body in enumerate(bodies):
            if body.static:
                continue
            if not dynamic[i] and body.sleeping == bool(sleeping[i]):
                continue
            body.position = position[i].copy()
            body.linear_velocity = linear[i].copy()
            body.angular_velocity = angular[i].copy()
            body.orientation = quat[i].copy()
            body.sleeping = bool(sleeping[i])
            body.sleep_time = float(sleep_time[i])
            body._moved = True
            body._update_world_inertia()

    def _solve_contacts(
        self,
        dt: float,
        static: np.ndarray,
        sleeping: np.ndarray,
        inv_mass: np.ndarray,
        inv_inertia: np.ndarray,
        restitution: np.ndarray,
        friction: np.ndarray,
        position: np.ndarray,
        linear: np.ndarray,
        angular: np.ndarray,
    ) -> None:
        a_list: list[int] = []
        b_list: list[int] = []
        normals: list[np.ndarray] = []
        points: list[np.ndarray] = []
        penetrations: list[float] = []
        n_imp: list[float] = []
        t0_imp: list[float] = []
        t1_imp: list[float] = []
        owners: list[_Contact] = []
        for manifold in self._manifolds.values():
            if not manifold.contacts:
                continue
            ia = manifold.a._index
            ib = manifold.b._index
            if (static[ia] or sleeping[ia]) and (static[ib] or sleeping[ib]):
                continue
            for contact in manifold.contacts:
                a_list.append(ia)
                b_list.append(ib)
                normals.append(manifold.normal)
                points.append(contact.position)
                penetrations.append(contact.penetration)
                n_imp.append(contact.normal_impulse)
                t0_imp.append(contact.t0_impulse)
                t1_imp.append(contact.t1_impulse)
                owners.append(contact)
        if not owners:
            return

        a_idx = np.array(a_list, dtype=np.intp)
        b_idx = np.array(b_list, dtype=np.intp)
        normal = np.array(normals, dtype=np.float64)
        point = np.array(points, dtype=np.float64)
        penetration = np.array(penetrations, dtype=np.float64)
        normal_impulse = np.array(n_imp, dtype=np.float64)
        t0_impulse = np.array(t0_imp, dtype=np.float64)
        t1_impulse = np.array(t1_imp, dtype=np.float64)

        eff_mass = np.where(sleeping, 0.0, inv_mass)
        eff_inertia = np.where(sleeping[:, None, None], 0.0, inv_inertia)
        im_a = eff_mass[a_idx]
        im_b = eff_mass[b_idx]
        ia_w = eff_inertia[a_idx]
        ib_w = eff_inertia[b_idx]
        ra = point - position[a_idx]
        rb = point - position[b_idx]

        t0, t1 = _tangent_basis(normal)

        def effective_mass(axis: np.ndarray) -> np.ndarray:
            cra = _cross_batch(ra, axis)
            crb = _cross_batch(rb, axis)
            return (
                im_a
                + im_b
                + np.einsum("mi,mi->m", cra, np.einsum("mij,mj->mi", ia_w, cra))
                + np.einsum("mi,mi->m", crb, np.einsum("mij,mj->mi", ib_w, crb))
            )

        ang_n_a = np.einsum("mij,mj->mi", ia_w, _cross_batch(ra, normal))
        ang_n_b = np.einsum("mij,mj->mi", ib_w, _cross_batch(rb, normal))
        ang_t0_a = np.einsum("mij,mj->mi", ia_w, _cross_batch(ra, t0))
        ang_t0_b = np.einsum("mij,mj->mi", ib_w, _cross_batch(rb, t0))
        ang_t1_a = np.einsum("mij,mj->mi", ia_w, _cross_batch(ra, t1))
        ang_t1_b = np.einsum("mij,mj->mi", ib_w, _cross_batch(rb, t1))
        mass_n = effective_mass(normal)
        mass_t0 = effective_mass(t0)
        mass_t1 = effective_mass(t1)

        bias = np.minimum(
            _BAUMGARTE / dt * np.maximum(penetration - _PENETRATION_SLOP, 0.0),
            _MAX_BIAS_VELOCITY,
        )
        contact_restitution = np.maximum(restitution[a_idx], restitution[b_idx])
        contact_friction = np.sqrt(
            np.maximum(friction[a_idx], 0.0) * np.maximum(friction[b_idx], 0.0)
        )

        # Warm start: apply the impulses carried over from the previous step.
        impulse = (
            normal * normal_impulse[:, None]
            + t0 * t0_impulse[:, None]
            + t1 * t1_impulse[:, None]
        )
        np.add.at(linear, a_idx, -im_a[:, None] * impulse)
        np.add.at(
            angular, a_idx, -np.einsum("mij,mj->mi", ia_w, _cross_batch(ra, impulse))
        )
        np.add.at(linear, b_idx, im_b[:, None] * impulse)
        np.add.at(
            angular, b_idx, np.einsum("mij,mj->mi", ib_w, _cross_batch(rb, impulse))
        )

        # Restitution bias, measured after warm starting.
        va = linear[a_idx]
        vb = linear[b_idx]
        wa = angular[a_idx]
        wb = angular[b_idx]
        rv = (vb + _cross_batch(wb, rb)) - (va + _cross_batch(wa, ra))
        vn = np.einsum("mi,mi->m", rv, normal)
        bias = bias + np.where(
            vn < -_RESTITUTION_THRESHOLD, contact_restitution * (-vn), 0.0
        )

        movable = eff_mass > 0.0
        colors = _color_contacts(a_idx, b_idx, movable)
        color_data = [
            (
                a_idx[color],
                b_idx[color],
                normal[color],
                t0[color],
                t1[color],
                ra[color],
                rb[color],
                im_a[color, None],
                im_b[color, None],
                mass_n[color],
                mass_t0[color],
                mass_t1[color],
                bias[color],
                contact_friction[color],
                ang_n_a[color],
                ang_n_b[color],
                ang_t0_a[color],
                ang_t0_b[color],
                ang_t1_a[color],
                ang_t1_b[color],
                color,
            )
            for color in colors
        ]

        for _ in range(self.solver_iterations):
            max_delta = 0.0
            for data in color_data:
                max_delta = max(
                    max_delta,
                    _solve_color(
                        data, linear, angular, normal_impulse, t0_impulse, t1_impulse
                    ),
                )
            if max_delta <= _SOLVER_EPSILON:
                break

        for k, contact in enumerate(owners):
            contact.normal_impulse = float(normal_impulse[k])
            contact.t0_impulse = float(t0_impulse[k])
            contact.t1_impulse = float(t1_impulse[k])

    # -- transform sync ---------------------------------------------------
    def _sync_transforms(self) -> None:
        """Copy body transforms onto their scene objects for the moved bodies."""
        with self._lock:
            bodies = list(self.bodies)
        for body in bodies:
            if not body._moved:
                continue
            # body.position is the centre of mass; the object origin is offset
            # back by the shape's local center.
            obj_position = body.position - body.rotation @ body.local_center
            body.obj._apply_physics_transform(obj_position, body.rotation)
            body._moved = False
