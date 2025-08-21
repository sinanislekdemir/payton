#!/usr/bin/env python3

"""Interactive ragdoll animation switcher with keyboard controls."""

import sdl2
import os

from payton.scene import Scene
from payton.scene.geometry import Plane
from payton.scene.geometry.ragdoll import RagDoll
from payton.scene.controller import GUIController

# Define skin colors
SKIN_LIGHT = [0.96, 0.80, 0.69]  # Light skin tone
SKIN_MEDIUM = [0.87, 0.72, 0.53]  # Medium skin tone  
SKIN_TAN = [0.78, 0.65, 0.43]    # Tan skin tone
HEAD_COLOR = [0.94, 0.78, 0.65]  # Slightly different head color


class AnimationController(GUIController):
    """Custom keyboard controller for animation switching."""
    
    def __init__(self, ragdoll):
        super().__init__()
        self.ragdoll = ragdoll
        
    def keyboard(self, event, scene):
        # Call parent to preserve other keyboard functionality
        super().keyboard(event, scene)
        
        if event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            animations = self.ragdoll.get_available_animations()
            
            if key == sdl2.SDLK_1 and len(animations) > 0:
                print(f"🎮 Switching to animation: {animations[0]}")
                self.ragdoll.change_animation(animations[0])
                
            elif key == sdl2.SDLK_2 and len(animations) > 1:
                print(f"🎮 Switching to animation: {animations[1]}")
                self.ragdoll.change_animation(animations[1])
                
            elif key == sdl2.SDLK_3 and len(animations) > 2:
                print(f"🎮 Switching to animation: {animations[2]}")
                self.ragdoll.change_animation(animations[2])
                
            elif key == sdl2.SDLK_SPACE:
                # Toggle animation on/off
                self.ragdoll.animate = not self.ragdoll.animate
                status = "ON" if self.ragdoll.animate else "OFF"
                print(f"🎮 Animation: {status}")
                
            elif key == sdl2.SDLK_h:
                # Show help
                print("\n🎮 KEYBOARD CONTROLS:")
                print("1, 2, 3 - Switch between animations")
                print("SPACE   - Toggle animation on/off")
                print("H       - Show this help")
                print(f"Available animations: {animations}")
                print(f"Current: {self.ragdoll.current_animation}")
        
        return True  # Return True to indicate the event was handled


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
    # Create scene
    scene = Scene()
    
    # Position camera
    scene.active_camera.position = [0, -5, 2]
    scene.active_camera.target = [0, 0, 1]
    
    # Create ragdoll
    ragdoll = RagDoll()
    
    # Load multiple animations - BVH files relative to script location
    walk_path = os.path.join(os.path.dirname(__file__), "dataset-1_walk_active_001.bvh")
    sad_path = os.path.join(os.path.dirname(__file__), "dataset-1_guide_sad_001.bvh")
    dance_path = os.path.join(os.path.dirname(__file__), "dataset-1_dance-long_normal_001.bvh")
    
    print("Loading animations...")
    
    # Load first animation (walk)
    ragdoll.load_bvh("walk", walk_path)
    print("✅ Loaded: walk")
    
    # Try to load second animation (sad)
    try:
        ragdoll.load_bvh("sad", sad_path)
        print("✅ Loaded: sad")
    except ValueError as e:
        print(f"❌ Failed to load sad animation: {e}")
    
    # Try to load third animation (dance)
    try:
        ragdoll.load_bvh("dance", dance_path)
        print("✅ Loaded: dance")
    except ValueError as e:
        print(f"❌ Failed to load dance animation: {e}")
    
    # Fill remaining slots with copies if needed
    available = ragdoll.get_available_animations()
    while len(available) < 3:
        if "walk" in available:
            ragdoll.load_bvh(f"walk_copy_{len(available)}", walk_path)
        available = ragdoll.get_available_animations()
    
    print(f"✅ Loaded animations: {ragdoll.get_available_animations()}")
    print(f"Current animation: {ragdoll.current_animation}")
    
    # Apply realistic skin colors
    print("🎨 Applying skin colors...")
    apply_skin_colors(ragdoll)
    
    # Add custom keyboard controller for animation switching
    animation_controller = AnimationController(ragdoll)
    scene.controller.add_controller(animation_controller)
    
    # Add to scene
    scene.add_object("ragdoll", ragdoll)
    
    # Add ground plane
    scene.add_object("ground", Plane(5, 5))

    # Scale to proper size
    ragdoll.scale(0.01, 0.01, 0.01)
    
    print("\n🎯 INTERACTIVE ANIMATION DEMO")
    print("================================")
    print("🎮 KEYBOARD CONTROLS:")
    print("1, 2, 3 - Switch between animations")
    print("SPACE   - Toggle animation on/off") 
    print("H       - Show help")
    print("ESC     - Exit")
    print("\n✅ Multi-animation ragdoll ready!")
    print("✅ Head visualization with capsule shape")
    print("✅ Realistic skin tone coloring")
    print("✅ Interactive keyboard controls")
    print("\nStarting scene...")
    
    # Start scene
    scene.run()


if __name__ == "__main__":
    main()
