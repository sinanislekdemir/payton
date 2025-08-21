"""Capsule module."""

import math
from typing import Any, List, Optional

from payton.scene.geometry.mesh import Mesh
from payton.scene.material import DEFAULT

_BULLET = False
try:
    import importlib.util
    _BULLET = importlib.util.find_spec("pybullet") is not None
except (ModuleNotFoundError, ImportError):
    _BULLET = False


class Capsule(Mesh):
    """Capsule class - a cylinder with hemispherical caps."""

    def __init__(
        self,
        radius: Optional[float] = None,
        radius_top: Optional[float] = None,
        radius_bottom: Optional[float] = None,
        height: float = 1.0,
        meridians: int = 12,
        parallels: int = 6,
        **kwargs: Any,
    ) -> None:
        """Define a Capsule with given parameters.

        Keyword arguments:
        radius -- Uniform radius for both ends (for backward compatibility)
        radius_top -- Radius of the top hemisphere and top of cylinder
        radius_bottom -- Radius of the bottom hemisphere and bottom of cylinder
        height -- Height of the cylindrical part (total height will be height + radius_top + radius_bottom)
        meridians -- Number of vertical segments around the circumference
        parallels -- Number of horizontal segments in each hemisphere
        
        Note: If radius is provided, it overrides radius_top and radius_bottom.
        If radius is None, radius_top and radius_bottom default to 0.5.
        """
        super().__init__(**kwargs)
        
        # Handle radius parameters with backward compatibility
        if radius is not None:
            self.radius_top = radius
            self.radius_bottom = radius
            self.radius = radius  # Keep for backward compatibility
        else:
            self.radius_top = radius_top if radius_top is not None else 0.5
            self.radius_bottom = radius_bottom if radius_bottom is not None else 0.5
            self.radius = max(self.radius_top, self.radius_bottom)  # For collision shape
            
        self.height: float = height
        self.meridians: int = meridians
        self.parallels: int = parallels
        self.build_capsule()

    def build_capsule(self) -> bool:
        """Build the capsule based on the initial parameters."""
        self.clear_triangles()
        
        # Build the cylindrical body
        self._build_cylinder_body()
        
        # Build the top hemisphere
        self._build_hemisphere(top=True)
        
        # Build the bottom hemisphere
        self._build_hemisphere(top=False)
        
        return True

    def _build_cylinder_body(self) -> None:
        """Build the cylindrical body of the capsule (can be tapered if radii differ)."""
        step_angle = math.radians(360.0 / self.meridians)
        u_step = 1.0 / self.meridians
        h_2 = self.height / 2.0
        indices = len(self._vertices)

        for i in range(self.meridians):
            # Current and next angle
            angle1 = step_angle * i
            angle2 = step_angle * (i + 1)
            
            # Calculate vertices for tapered cylinder
            # Bottom (use radius_bottom)
            x1_bot = self.radius_bottom * math.cos(angle1)
            y1_bot = self.radius_bottom * math.sin(angle1)
            x2_bot = self.radius_bottom * math.cos(angle2)
            y2_bot = self.radius_bottom * math.sin(angle2)
            
            # Top (use radius_top)
            x1_top = self.radius_top * math.cos(angle1)
            y1_top = self.radius_top * math.sin(angle1)
            x2_top = self.radius_top * math.cos(angle2)
            y2_top = self.radius_top * math.sin(angle2)
            
            # UV coordinates
            u1 = u_step * i
            u2 = u_step * (i + 1)
            
            # Create quad as two triangles
            # Bottom vertices
            v1 = [x1_bot, y1_bot, -h_2]
            v2 = [x2_bot, y2_bot, -h_2]
            # Top vertices
            v3 = [x1_top, y1_top, h_2]
            v4 = [x2_top, y2_top, h_2]
            
            # Calculate normals for tapered surface correctly
            def calculate_frustum_normal(angle: float) -> List[float]:
                """Calculate the normal for a frustum surface at given angle."""
                # Direction vectors
                cos_a, sin_a = math.cos(angle), math.sin(angle)
                
                # Surface tangent vectors
                # Circumferential direction (tangent to circle)
                tangent_u = [-sin_a, cos_a, 0.0]
                
                # Axial direction (from bottom to top, accounting for radius change)
                tangent_v = [
                    (self.radius_top - self.radius_bottom) * cos_a,
                    (self.radius_top - self.radius_bottom) * sin_a,
                    self.height
                ]
                
                # Cross product gives surface normal (tangent_u × tangent_v)
                normal = [
                    tangent_u[1] * tangent_v[2] - tangent_u[2] * tangent_v[1],
                    tangent_u[2] * tangent_v[0] - tangent_u[0] * tangent_v[2],
                    tangent_u[0] * tangent_v[1] - tangent_u[1] * tangent_v[0]
                ]
                
                # Normalize
                length = math.sqrt(sum(n**2 for n in normal))
                if length > 0:
                    normal = [n / length for n in normal]
                else:
                    # Fallback to radial direction for degenerate case
                    normal = [cos_a, sin_a, 0.0]
                
                return normal
            
            # Calculate normals at the two angles
            normal1 = calculate_frustum_normal(angle1)
            normal2 = calculate_frustum_normal(angle2)
            
            # Add vertices
            self._vertices.extend([v1, v2, v3, v4])
            
            # Add texture coordinates
            self._texcoords.extend([[u1, 0.0], [u2, 0.0], [u1, 1.0], [u2, 1.0]])
            
            # Add normals (same normal for top and bottom vertices at each angle)
            self._normals.extend([normal1, normal2, normal1, normal2])
            
            # Add triangles (quad = 2 triangles)
            self._indices.append([indices, indices + 1, indices + 2])
            self._indices.append([indices + 1, indices + 3, indices + 2])
            self.materials[DEFAULT]._indices.append([indices, indices + 1, indices + 2])
            self.materials[DEFAULT]._indices.append([indices + 1, indices + 3, indices + 2])
            
            indices += 4

    def _build_hemisphere(self, top: bool = True) -> None:
        """Build a hemisphere cap.
        
        Args:
            top: If True, build top hemisphere, otherwise bottom hemisphere
        """
        # Use appropriate radius for this hemisphere
        radius = self.radius_top if top else self.radius_bottom
        
        step_angle = math.radians(360.0 / self.meridians)
        step_height = math.radians(90.0 / self.parallels)  # Only 90 degrees for hemisphere
        u_step = 1.0 / self.meridians
        v_step = 1.0 / self.parallels
        h_2 = self.height / 2.0
        indices = len(self._vertices)
        
        # Determine the z offset and angle range based on top/bottom
        z_offset = h_2 if top else -h_2
        start_angle = 0.0 if top else math.radians(90.0)
        angle_multiplier = 1 if top else -1
        
        for i in range(self.parallels):
            for j in range(self.meridians):
                # Calculate the four vertices of this segment
                h1 = start_angle + angle_multiplier * step_height * i
                h2 = start_angle + angle_multiplier * step_height * (i + 1)
                a1 = step_angle * j
                a2 = step_angle * (j + 1)
                
                # First vertex
                x1 = radius * math.sin(h1) * math.cos(a1)
                y1 = radius * math.sin(h1) * math.sin(a1)
                z1 = radius * math.cos(h1) * angle_multiplier + z_offset
                
                # Second vertex
                x2 = radius * math.sin(h2) * math.cos(a1)
                y2 = radius * math.sin(h2) * math.sin(a1)
                z2 = radius * math.cos(h2) * angle_multiplier + z_offset
                
                # Third vertex
                x3 = radius * math.sin(h2) * math.cos(a2)
                y3 = radius * math.sin(h2) * math.sin(a2)
                z3 = radius * math.cos(h2) * angle_multiplier + z_offset
                
                # Fourth vertex
                x4 = radius * math.sin(h1) * math.cos(a2)
                y4 = radius * math.sin(h1) * math.sin(a2)
                z4 = radius * math.cos(h1) * angle_multiplier + z_offset
                
                # Texture coordinates
                u1 = u_step * j
                u2 = u_step * j
                u3 = u_step * (j + 1)
                u4 = u_step * (j + 1)
                
                v1 = v_step * i
                v2 = v_step * (i + 1)
                v3 = v_step * (i + 1)
                v4 = v_step * i
                
                # Calculate normals (pointing outward from center)
                def normalize(vec: List[float]) -> List[float]:
                    length = math.sqrt(sum(v**2 for v in vec))
                    return [v / length if length > 0 else 0 for v in vec]
                
                n1 = normalize([x1, y1, z1 - z_offset])
                n2 = normalize([x2, y2, z2 - z_offset])
                n3 = normalize([x3, y3, z3 - z_offset])
                n4 = normalize([x4, y4, z4 - z_offset])
                
                # Add vertices
                self._vertices.extend([[x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]])
                
                # Add texture coordinates
                self._texcoords.extend([[u1, v1], [u2, v2], [u3, v3], [u4, v4]])
                
                # Add normals
                self._normals.extend([n1, n2, n3, n4])
                
                # Add triangles
                if top:
                    self._indices.append([indices, indices + 1, indices + 2])
                    self._indices.append([indices, indices + 2, indices + 3])
                    self.materials[DEFAULT]._indices.append([indices, indices + 1, indices + 2])
                    self.materials[DEFAULT]._indices.append([indices, indices + 2, indices + 3])
                else:
                    # Reverse winding for bottom hemisphere
                    self._indices.append([indices, indices + 2, indices + 1])
                    self._indices.append([indices, indices + 3, indices + 2])
                    self.materials[DEFAULT]._indices.append([indices, indices + 2, indices + 1])
                    self.materials[DEFAULT]._indices.append([indices, indices + 3, indices + 2])
                
                indices += 4

    def _create_collision_shape(self) -> None:
        """Create bullet physics collision shape for the capsule.
        
        Note: Bullet Physics doesn't support tapered capsules natively,
        so we use the larger radius for a conservative collision shape.
        """
        if _BULLET:
            import pybullet

            # Use the larger radius for collision detection
            collision_radius = max(self.radius_top, self.radius_bottom)
            self._bullet_shape_id = pybullet.createCollisionShape(
                pybullet.GEOM_CAPSULE, radius=collision_radius, height=self.height
            )
