"""Ragdoll Object with proper BVH animation support.

Simple, clean implementation following bvh.py guidelines.
"""

import math
from typing import Any, Dict, List, Optional
from dataclasses import dataclass, field
from time import time

from payton.scene.geometry.base import Object
from payton.scene.geometry.capsule import Capsule


@dataclass
class Joint:
    """Represents a joint in the BVH hierarchy - following bvh.py pattern."""
    name: str
    offset: List[float]  # [x, y, z] offset from parent
    channels: List[str]  # Rotation/position channels
    children: List['Joint'] = field(default_factory=list)
    parent: Optional['Joint'] = None
    is_end_site: bool = False


class RagDoll(Object):
    """Clean ragdoll implementation following bvh.py guidelines."""
    
    def __init__(self, **kwargs: Any) -> None:
        """Initialize the ragdoll."""
        super().__init__(**kwargs)
        
        # BVH data
        self.root_joint: Optional[Joint] = None
        self.joints: Dict[str, Joint] = {}
        
        # Animation data - support multiple animations
        self.animations: Dict[str, Dict[str, Any]] = {}  # animation_name -> {frame_data, frame_time, frames}
        self.current_animation: Optional[str] = None
        
        # Visual objects (capsules for bones)
        self.joint_objects: Dict[str, Object] = {}
        
        # Animation state
        self._current_frame = 0
        self._animate = True
        self._time = 0.0
        self._animation_start_time = 0.0  # Reference time for animation
    
    @property
    def animate(self) -> bool:
        return self._animate
    
    @animate.setter
    def animate(self, value: bool) -> None:
        self._animate = value
    
    def load_bvh(self, animation_name: str, file_path: str) -> None:
        """Load BVH file with animation name following bvh.py pattern."""
        try:
            with open(file_path, 'r') as f:
                content = f.read()
            
            # Parse hierarchy and motion sections
            lines = content.strip().split('\n')
            motion_index = -1
            
            for i, line in enumerate(lines):
                if line.strip() == 'MOTION':
                    motion_index = i
                    break
            
            if motion_index == -1:
                raise ValueError("No MOTION section found")
            
            # Parse hierarchy
            hierarchy_lines = lines[1:motion_index]  # Skip first HIERARCHY line
            new_root_joint = self._parse_joint(hierarchy_lines, 0)[0]
            
            # If we already have a skeleton, validate it matches
            if self.root_joint is not None:
                if not self._validate_skeleton_match(self.root_joint, new_root_joint):
                    raise ValueError(f"Animation '{animation_name}' skeleton does not match existing skeleton")
            else:
                # First animation - store the skeleton
                self.root_joint = new_root_joint
                
                # Build visual hierarchy only on first load
                self._build_hierarchy()
            
            # Parse motion data
            motion_lines = lines[motion_index + 1:]
            frames = int(motion_lines[0].split()[1])
            frame_time = float(motion_lines[1].split()[2])
            
            # Parse frame data
            frame_data = []
            for i in range(2, len(motion_lines)):
                if motion_lines[i].strip():
                    values = [float(x) for x in motion_lines[i].split()]
                    frame_data.append(values)
            
            # Store animation data
            self.animations[animation_name] = {
                'frame_data': frame_data,
                'frame_time': frame_time,
                'frames': frames
            }
            
            print(f"Loaded BVH animation '{animation_name}': {frames} frames, {frame_time}s per frame")
            
            # If this is the first animation, make it current
            if self.current_animation is None:
                self.current_animation = animation_name
                
                # Apply first frame to get proper initial bone lengths and positions
                if frame_data:
                    self.set_keyframe(0)
            
        except Exception as e:
            print(f"Error loading BVH: {e}")
            raise
    
    def _parse_joint(self, lines: List[str], index: int) -> tuple[Joint, int]:
        """Parse joint from lines starting at index."""
        line = lines[index].strip()
        
        if line.startswith('ROOT') or line.startswith('JOINT'):
            joint_name = line.split()[1]
        elif line.startswith('End Site'):
            joint_name = "End Site"
        else:
            raise ValueError(f"Expected joint definition, got: {line}")
        
        joint = Joint(name=joint_name, offset=[0, 0, 0], channels=[])
        joint.is_end_site = line.startswith('End Site')
        
        if not joint.is_end_site:
            self.joints[joint_name] = joint
        
        index += 1
        
        # Expect opening brace
        if not lines[index].strip() == '{':
            raise ValueError(f"Expected '{{', got: {lines[index]}")
        index += 1
        
        # Parse joint contents
        while index < len(lines):
            line = lines[index].strip()
            
            if line == '}':
                index += 1
                break
            elif line.startswith('OFFSET'):
                parts = line.split()
                # Convert BVH Y-up to Payton Z-up by swapping Y and Z coordinates
                joint.offset = [float(parts[1]), float(parts[3]), float(parts[2])]
            elif line.startswith('CHANNELS'):
                parts = line.split()
                channel_count = int(parts[1])
                joint.channels = parts[2:2 + channel_count]
            elif line.startswith('JOINT') or line.startswith('End Site'):
                child_joint, index = self._parse_joint(lines, index)
                child_joint.parent = joint
                joint.children.append(child_joint)
                continue
            
            index += 1
        
        return joint, index
    
    def _validate_skeleton_match(self, existing_joint: Joint, new_joint: Joint) -> bool:
        """Validate that two joint hierarchies match - only check structure, not exact values."""
        # Check joint name
        if existing_joint.name != new_joint.name:
            return False
        
        # Check is_end_site flag
        if existing_joint.is_end_site != new_joint.is_end_site:
            return False
        
        # Check number of children
        if len(existing_joint.children) != len(new_joint.children):
            return False
        
        # Recursively check children (only structure, not values)
        for existing_child, new_child in zip(existing_joint.children, new_joint.children):
            if not self._validate_skeleton_match(existing_child, new_child):
                return False
        
        return True
    
    def change_animation(self, animation_name: str) -> None:
        """Switch to a different loaded animation."""
        if animation_name not in self.animations:
            raise ValueError(f"Animation '{animation_name}' not found. Available animations: {list(self.animations.keys())}")
        
        self.current_animation = animation_name
        self._animation_start_time = time()  # Set to current time instead of 0.0
        self._current_frame = 0
        
        # Apply first frame of new animation immediately
        if self.animations[animation_name]['frame_data']:
            self.set_keyframe(0)
        
        print(f"Switched to animation: {animation_name}")
    
    def get_available_animations(self) -> List[str]:
        """Get list of available animation names."""
        return list(self.animations.keys())
    
    @property
    def frame_data(self) -> List[List[float]]:
        """Get frame data for current animation."""
        if self.current_animation is None:
            return []
        return self.animations[self.current_animation]['frame_data']
    
    @property
    def frame_time(self) -> float:
        """Get frame time for current animation."""
        if self.current_animation is None:
            return 0.0
        return self.animations[self.current_animation]['frame_time']
    
    def _build_hierarchy(self) -> None:
        """Build Payton object hierarchy for visualization."""
        if not self.root_joint:
            return
        
        # Clear existing
        self.joint_objects.clear()
        
        # Build from root - create root joint but don't make capsules for it
        self._build_joint_objects(self.root_joint, None, skip_root=False)
        
        # Create capsules for bones (will skip root joint connections)
        self._create_bone_capsules(self.root_joint)
    
    def _build_joint_objects(self, joint: Joint, parent_obj: Optional[Object], skip_root: bool = False) -> None:
        """Build joint objects recursively."""
        if joint.is_end_site:
            return
        
        # Create joint object (even for root, so hierarchy works correctly)
        joint_obj = Object(name=f"joint_{joint.name}")
        joint_obj.position = joint.offset.copy()
        
        # Add to hierarchy
        if parent_obj:
            parent_obj.add_child(joint.name, joint_obj)
        else:
            self.add_child(joint.name, joint_obj)
        
        self.joint_objects[joint.name] = joint_obj
        
        # Process children
        for child in joint.children:
            self._build_joint_objects(child, joint_obj)
    
    def _create_bone_capsules(self, joint: Joint) -> None:
        """Create capsules for bones and head-shaped capsules for heads."""
        for child in joint.children:
            if not child.is_end_site:
                # Check if child is a Head joint
                if child.name.lower() == "head":
                    self._create_head_for_joint(joint, child)
                else:
                    self._create_capsule_for_bone(joint, child)
        
        # Recurse
        for child in joint.children:
            if not child.is_end_site:
                self._create_bone_capsules(child)
    
    def _create_capsule_for_bone(self, parent_joint: Joint, child_joint: Joint) -> None:
        """Create realistic tapered capsule between two joints."""
        # Skip if root joint (per requirement)
        if not parent_joint.parent:
            return
        
        # Calculate bone length
        offset = child_joint.offset
        length = math.sqrt(offset[0]**2 + offset[1]**2 + offset[2]**2)
        
        if length < 0.01:
            return
        
        # Determine bone type and create appropriate tapered capsule
        bone_type = self._classify_bone_type(parent_joint.name, child_joint.name)
        radius_parent, radius_child = self._get_bone_radii(bone_type, length)
        
        # Create tapered capsule - thicker at joints, thinner in middle
        capsule = Capsule(
            radius_top=radius_child,    # Radius at child joint
            radius_bottom=radius_parent,  # Radius at parent joint  
            height=length
        )
        
        # Position at bone midpoint
        capsule.position = [offset[0] * 0.5, offset[1] * 0.5, offset[2] * 0.5]
        
        # Orient along bone
        self._orient_capsule(capsule, offset, length)
        
        # Add to parent joint
        if parent_joint.name in self.joint_objects:
            self.joint_objects[parent_joint.name].add_child(
                f"bone_to_{child_joint.name}", capsule
            )
    
    def _create_head_for_joint(self, parent_joint: Joint, head_joint: Joint) -> None:
        """Create capsule-shaped head for head joint - bigger top, smaller jaw."""
        # Skip if root joint (per requirement)
        if not parent_joint.parent:
            return
        
        # Calculate "neck" length to size head appropriately
        offset = head_joint.offset
        neck_length = math.sqrt(offset[0]**2 + offset[1]**2 + offset[2]**2)
        
        if neck_length < 0.01:
            return
        
        # First, create a small capsule for the neck bone
        neck_radius = neck_length * 0.05  # Thinner than regular bones
        neck_capsule = Capsule(radius=neck_radius, height=neck_length)
        
        # Position neck at midpoint between parent and head joint
        neck_capsule.position = [offset[0] * 0.5, offset[1] * 0.5, offset[2] * 0.5]
        
        # Orient neck along bone
        self._orient_capsule(neck_capsule, offset, neck_length)
        
        # Add neck to parent joint
        if parent_joint.name in self.joint_objects:
            self.joint_objects[parent_joint.name].add_child(
                f"neck_to_{head_joint.name}", neck_capsule
            )
        
        # Now create the head capsule - tapered for realistic head shape
        # Head height should be proportional to neck length
        head_height = neck_length * 1.5  # Head height
        head_radius_top = neck_length * 0.7    # Smaller top (crown)
        head_radius_bottom = neck_length * 0.9  # Larger bottom (jaw area)
        
        head_capsule = Capsule(
            radius_top=head_radius_top,     # Smaller at crown
            radius_bottom=head_radius_bottom, # Larger at jaw
            height=head_height
        )
        
        # Position head at the head joint location (end of neck)
        # Offset slightly upward so the bottom of head capsule connects to neck
        head_position = offset.copy()
        head_position[0] += head_height * 0.7  # Move up slightly (Z is up in Payton)
        head_capsule.position = head_position
        
        # Orient head capsule vertically (Z-up)
        # Default capsule is horizontal, so we need to rotate it to be vertical
        head_capsule.rotate_around_y(math.radians(90))  # Rotate to make it vertical
        
        # Add head to parent joint (so it moves with neck)
        if parent_joint.name in self.joint_objects:
            self.joint_objects[parent_joint.name].add_child(
                f"head_{head_joint.name}", head_capsule
            )
    
    def _classify_bone_type(self, parent_name: str, child_name: str) -> str:
        """Classify bone type based on joint names for realistic tapering."""
        # Convert to lowercase for easier matching
        parent_lower = parent_name.lower()
        child_lower = child_name.lower()
        
        # Major limb bones (thick)
        if any(bone in parent_lower for bone in ['shoulder', 'hip', 'spine']):
            return 'major'
        if any(bone in child_lower for bone in ['shoulder', 'hip', 'spine']):
            return 'major'
        
        # Arm bones
        if 'arm' in parent_lower or 'arm' in child_lower:
            if 'forearm' in parent_lower or 'forearm' in child_lower:
                return 'forearm'  # Tapered (thick at elbow, thin at wrist)
            else:
                return 'upperarm'  # Reverse taper (thick at shoulder, thin at elbow)
        
        # Leg bones  
        if 'leg' in parent_lower or 'leg' in child_lower:
            if 'upleg' in parent_lower or 'upleg' in child_lower:
                return 'upperleg'  # Thick at hip, thinner at knee
            else:
                return 'lowerleg'  # Thick at knee, thinner at ankle
        
        # Hand/finger bones (small and thin)
        if any(part in parent_lower or part in child_lower 
               for part in ['hand', 'finger', 'thumb']):
            return 'finger'
        
        # Foot bones
        if any(part in parent_lower or part in child_lower 
               for part in ['foot', 'toe']):
            return 'foot'
        
        # Default to regular bone
        return 'regular'
    
    def _get_bone_radii(self, bone_type: str, length: float) -> tuple[float, float]:
        """Get appropriate radii for bone type (parent_radius, child_radius)."""
        base_radius = length * 0.08  # Slightly thinner than before
        
        if bone_type == 'major':
            # Major bones: thick and fairly uniform
            return (base_radius * 1.4, base_radius * 1.2)
        
        elif bone_type == 'upperarm':
            # Upper arm: thick at shoulder, thin at elbow
            return (base_radius * 1.3, base_radius * 0.8)
        
        elif bone_type == 'forearm':
            # Forearm: thick at elbow, thin at wrist
            return (base_radius * 0.9, base_radius * 0.6)
        
        elif bone_type == 'upperleg':
            # Upper leg: thick at hip, thinner at knee
            return (base_radius * 1.5, base_radius * 1.0)
        
        elif bone_type == 'lowerleg':
            # Lower leg: thick at knee, thin at ankle
            return (base_radius * 1.0, base_radius * 0.7)
        
        elif bone_type == 'finger':
            # Fingers: very small and thin
            return (base_radius * 0.4, base_radius * 0.3)
        
        elif bone_type == 'foot':
            # Foot bones: small but sturdy
            return (base_radius * 0.6, base_radius * 0.5)
        
        else:
            # Regular bones: slight taper
            return (base_radius * 1.1, base_radius * 0.9)
    
    def _orient_capsule(self, capsule: Object, direction: List[float], length: float) -> None:
        """Orient capsule along direction vector using all three rotation axes."""
        if length < 0.001:
            return
        
        # Normalize direction
        norm_dir = [direction[0]/length, direction[1]/length, direction[2]/length]
        
        # Use base.py rotation functions for complete orientation
        # Calculate Euler angles to align capsule Z-axis with bone direction
        
        # Calculate rotation around X-axis (pitch)
        angle_x = math.atan2(norm_dir[1], norm_dir[2])
        
        # Calculate rotation around Y-axis (yaw) 
        angle_y = -math.atan2(norm_dir[0], math.sqrt(norm_dir[1]**2 + norm_dir[2]**2))
        
        # For bones, we might also need rotation around Z-axis (roll) for proper orientation
        # This could be important for twisted bones or specific orientations
        angle_z = 0.0  # For now, no roll - but this is where we'd add it if needed
        
        # Apply rotations using base.py methods
        capsule.rotate_around_x(angle_x)
        capsule.rotate_around_y(angle_y)
        if abs(angle_z) > 0.001:
            capsule.rotate_around_z(angle_z)
    
    def set_keyframe(self, frame_number: int) -> None:
        """Set ragdoll to specific frame."""
        if not self.frame_data or frame_number >= len(self.frame_data) or not self.root_joint:
            return
        
        frame_data = self.frame_data[frame_number]
        self._apply_frame(self.root_joint, frame_data, 0)
        self._current_frame = frame_number
    
    def _apply_frame(self, joint: Joint, frame_data: List[float], channel_index: int) -> int:
        """Apply frame data to joint and children."""
        if joint.is_end_site:
            return channel_index
        
        # Get joint object
        joint_obj = self.joint_objects.get(joint.name)
        if not joint_obj:
            # Process children without applying transforms
            for child in joint.children:
                channel_index = self._apply_frame(child, frame_data, channel_index)
            return channel_index
        
        # CRITICAL FIX: Reset transformation matrix to identity before applying new frame
        # This prevents accumulation of rotations between frames
        joint_obj.matrix = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ]
        
        # Start with static offset
        position = joint.offset.copy()
        
        # Apply channels in order
        for channel in joint.channels:
            if channel_index >= len(frame_data):
                break
            
            value = frame_data[channel_index]
            
            if channel == "Xposition":
                position[0] = value
            elif channel == "Yposition":
                position[2] = value  # BVH Y -> Payton Z
            elif channel == "Zposition":
                position[1] = value  # BVH Z -> Payton Y
            elif channel == "Xrotation":
                joint_obj.rotate_around_x(math.radians(value))
            elif channel == "Yrotation":
                joint_obj.rotate_around_z(math.radians(value))  # BVH Y-rot -> Payton Z-rot
            elif channel == "Zrotation":
                joint_obj.rotate_around_y(math.radians(value))  # BVH Z-rot -> Payton Y-rot
            
            channel_index += 1
        
        # Set position directly - coordinate conversion handled by ragdoll rotation
        joint_obj.position = position
        
        # Process children
        for child in joint.children:
            channel_index = self._apply_frame(child, frame_data, channel_index)
        
        return channel_index
    
    def update_animation(self, delta_time: float) -> None:
        """Update animation over time with proper timing."""
        if not self._animate or not self.frame_data:
            return
        
        # Initialize animation start time on first update
        if self._animation_start_time == 0.0:
            self._animation_start_time = time()
            self._current_frame = 0
            return
        
        # Calculate total elapsed time since animation started
        current_time = time()
        elapsed_time = current_time - self._animation_start_time
        
        # Calculate total animation duration
        total_frames = len(self.frame_data)
        total_duration = total_frames * self.frame_time
        
        # Loop the animation by using modulo on elapsed time
        animation_time = elapsed_time % total_duration
        
        # Calculate current frame based on elapsed time
        frame_number = int(animation_time / self.frame_time)
        
        # Clamp to valid frame range
        frame_number = max(0, min(frame_number, total_frames - 1))
        
        # Only update if frame changed (avoid unnecessary updates)
        if frame_number != self._current_frame:
            self._current_frame = frame_number
            self.set_keyframe(frame_number)
    
    def render(self, lit: bool, shader, parent_matrix=None, _primitive=None) -> None:
        """Render with animation update."""
        if not self._visible:
            return
        
        # Update animation if enabled
        if self._animate:
            current_time = time()
            if self._time > 0:
                delta_time = current_time - self._time
                self.update_animation(delta_time)
            else:
                # First frame - just initialize
                self.update_animation(0.0)
            self._time = current_time
        
        # Let parent handle rendering
        super().render(lit, shader, parent_matrix, _primitive)
