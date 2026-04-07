#!/usr/bin/env python3

"""BVH Viewer - Interactive BVH animation viewer with playback controls."""

import argparse
import os
import sys
from time import time

import sdl2

from payton.scene import Scene
from payton.scene.controller import GUIController
from payton.scene.geometry import Plane
from payton.scene.geometry.ragdoll import RagDoll

# Define skin colors
SKIN_LIGHT = [0.96, 0.80, 0.69]  # Light skin tone
SKIN_MEDIUM = [0.87, 0.72, 0.53]  # Medium skin tone  
SKIN_TAN = [0.78, 0.65, 0.43]    # Tan skin tone
HEAD_COLOR = [0.94, 0.78, 0.65]  # Slightly different head color


class BVHController(GUIController):
    """Custom keyboard controller for BVH animation playback."""

    def __init__(self, ragdoll):
        super().__init__()
        self.ragdoll = ragdoll
        self.playback_speed = 1.0
        self.show_info = True

    def keyboard(self, event, scene):
        # Call parent to preserve other keyboard functionality
        super().keyboard(event, scene)
        
        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            
            if key == sdl2.SDLK_SPACE:
                # Toggle animation on/off
                self.ragdoll.animate = not self.ragdoll.animate
                status = "ON" if self.ragdoll.animate else "OFF"
                print(f"🎬 Animation: {status}")
                
            elif key == sdl2.SDLK_LEFT:
                # Previous frame
                if self.ragdoll.frame_data:
                    current_frame = max(0, self.ragdoll._current_frame - 1)
                    self.ragdoll.set_keyframe(current_frame)
                    print(f"⏮️  Frame: {current_frame + 1}/{len(self.ragdoll.frame_data)}")
                
            elif key == sdl2.SDLK_RIGHT:
                # Next frame
                if self.ragdoll.frame_data:
                    current_frame = min(len(self.ragdoll.frame_data) - 1, self.ragdoll._current_frame + 1)
                    self.ragdoll.set_keyframe(current_frame)
                    print(f"⏭️  Frame: {current_frame + 1}/{len(self.ragdoll.frame_data)}")
                
            elif key == sdl2.SDLK_UP:
                # Increase playback speed
                self.playback_speed = min(5.0, self.playback_speed + 0.25)
                self._update_animation_speed()
                print(f"⏫ Speed: {self.playback_speed}x")
                
            elif key == sdl2.SDLK_DOWN:
                # Decrease playback speed
                self.playback_speed = max(0.1, self.playback_speed - 0.25)
                self._update_animation_speed()
                print(f"⏬ Speed: {self.playback_speed}x")
                
            elif key == sdl2.SDLK_r:
                # Reset to beginning
                if self.ragdoll.frame_data:
                    self.ragdoll.set_keyframe(0)
                    self.ragdoll._animation_start_time = time()
                    print("🔄 Reset to frame 1")
                
            elif key == sdl2.SDLK_i:
                # Toggle info display
                self.show_info = not self.show_info
                if self.show_info:
                    self._print_info()
                else:
                    print("ℹ️  Info display: OFF")
                
            elif key == sdl2.SDLK_h:
                # Show help
                self._print_help()
        
        return True  # Return True to indicate the event was handled
    
    def _update_animation_speed(self):
        """Update the animation frame time based on playback speed."""
        if self.ragdoll.current_animation:
            original_frame_time = self.ragdoll.animations[self.ragdoll.current_animation]["frame_time"]
            # Modify the frame time to change speed (lower = faster)
            self.ragdoll.animations[self.ragdoll.current_animation]["frame_time"] = original_frame_time / self.playback_speed
    
    def _print_info(self):
        """Print current animation information."""
        if not self.ragdoll.current_animation:
            return
            
        anim_data = self.ragdoll.animations[self.ragdoll.current_animation]
        frames = anim_data["frames"]
        frame_time = anim_data["frame_time"]
        duration = frames * frame_time
        
        print("\n" + "="*50)
        print("ℹ️  BVH ANIMATION INFO")
        print("="*50)
        print(f"📁 Animation: {self.ragdoll.current_animation}")
        print(f"🎞️  Frames: {frames}")
        print(f"⏱️  Frame Time: {frame_time:.4f}s")
        print(f"⏳ Duration: {duration:.2f}s")
        print(f"🎬 Current Frame: {self.ragdoll._current_frame + 1}")
        print(f"⚡ Playback Speed: {self.playback_speed}x")
        print(f"▶️  Playing: {'Yes' if self.ragdoll.animate else 'No'}")
        print("="*50)
    
    def _print_help(self):
        """Print help information."""
        print("\n" + "="*50)
        print("🎮 BVH VIEWER CONTROLS")
        print("="*50)
        print("🎬 PLAYBACK:")
        print("  SPACE     - Toggle animation on/off")
        print("  LEFT/RIGHT- Previous/Next frame")
        print("  UP/DOWN   - Increase/Decrease speed")
        print("  R         - Reset to beginning")
        print()
        print("ℹ️  DISPLAY:")
        print("  I         - Toggle info display")
        print("  H         - Show this help")
        print()
        print("🎥 CAMERA:")
        print("  Mouse     - Rotate camera")
        print("  Wheel     - Zoom in/out")
        print("  C         - Toggle perspective/orthographic")
        print()
        print("🎨 VIEW:")
        print("  G         - Toggle grid")
        print("  W         - Toggle wireframe")
        print("  F2/F3     - Switch cameras")
        print()
        print("🚪 GENERAL:")
        print("  ESC       - Exit")
        print("="*50)


def apply_skin_colors(ragdoll):
    """Apply realistic skin colors to the ragdoll."""
    # Apply colors to different body parts
    for joint_name, joint_obj in ragdoll.joint_objects.items():
        # Get all children (bones/capsules) of this joint
        for child_name, child_obj in joint_obj.children.items():
            if child_name.startswith("head_"):
                # Head gets special color
                child_obj.material.color = HEAD_COLOR
            elif child_name.startswith("neck_"):
                # Neck gets head color too
                child_obj.material.color = HEAD_COLOR  
            elif "Leg" in joint_name or "Foot" in joint_name or "Toes" in joint_name:
                # Legs and feet get tan color
                child_obj.material.color = SKIN_TAN
            elif "Arm" in joint_name or "Hand" in joint_name:
                # Arms and hands get medium color
                child_obj.material.color = SKIN_MEDIUM
            else:
                # Body gets light color
                child_obj.material.color = SKIN_LIGHT


def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description="BVH Animation Viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python bvh_viewer.py animation.bvh
  python bvh_viewer.py /path/to/animation.bvh --scale 0.02
        """
    )
    
    parser.add_argument("bvh_file", help="Path to the BVH file to load")
    parser.add_argument("--scale", type=float, default=0.01, 
                       help="Scale factor for the ragdoll (default: 0.01)")
    parser.add_argument("--no-skin", action="store_true",
                       help="Disable skin color application")
    
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.bvh_file):
        print(f"❌ Error: BVH file not found: {args.bvh_file}")
        sys.exit(1)
    
    # Create scene
    scene = Scene()
    
    # Position camera
    scene.active_camera.position = [0, -5, 2]
    scene.active_camera.target = [0, 0, 1]
    
    # Create ragdoll
    ragdoll = RagDoll()
    
    print(f"📂 Loading BVH file: {args.bvh_file}")
    
    try:
        # Load the BVH file
        animation_name = os.path.splitext(os.path.basename(args.bvh_file))[0]
        ragdoll.load_bvh(animation_name, args.bvh_file)
        print("✅ BVH file loaded successfully!")
    except Exception as e:
        print(f"❌ Error loading BVH file: {e}")
        sys.exit(1)
    
    # Apply skin colors unless disabled
    if not args.no_skin:
        print("🎨 Applying skin colors...")
        apply_skin_colors(ragdoll)
    
    # Add custom BVH controller
    bvh_controller = BVHController(ragdoll)
    scene.controller.add_controller(bvh_controller)
    
    # Add to scene
    scene.add_object("ragdoll", ragdoll)
    
    # Add ground plane
    scene.add_object("ground", Plane(5, 5))
    
    # Scale the ragdoll
    ragdoll.scale(args.scale, args.scale, args.scale)
    print(f"📏 Scaled ragdoll by factor: {args.scale}")
    
    # Print initial info
    print("\n🎯 BVH VIEWER READY")
    print("="*50)
    print(f"📁 File: {os.path.basename(args.bvh_file)}")
    print(f"📏 Scale: {args.scale}")
    print(f"🎨 Skin colors: {'Enabled' if not args.no_skin else 'Disabled'}")
    
    # Show initial animation info
    bvh_controller._print_info()
    
    print("\n🎮 CONTROLS:")
    print("  SPACE - Toggle animation")
    print("  H     - Show full help")
    print("  ESC   - Exit")
    print("\nStarting viewer...")
    
    # Start scene
    scene.run()


if __name__ == "__main__":
    main()
