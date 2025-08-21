#!/usr/bin/env python3

"""
Tapered Capsule Example

Demonstrates the new tapered capsule functionality where you can specify
different radii for the top and bottom ends of the capsule. This creates
more realistic shapes for bones, limbs, and other organic structures.

Features:
- Backward compatibility with regular capsules (single radius)
- Tapered capsules with radius_top and radius_bottom parameters
- Proper normal calculations for smooth lighting
- Physics-compatible collision shapes
"""

from payton.scene import Scene
from payton.scene.geometry.capsule import Capsule
from payton.scene.geometry import Plane

def main():
    # Create scene
    scene = Scene()
    
    # Position camera
    scene.active_camera.position = [0, -10, 3]
    scene.active_camera.target = [0, 0, 1]
    
    # Test 1: Regular capsule (backward compatibility)
    regular_capsule = Capsule(radius=0.5, height=2.0)
    regular_capsule.position = [-3, 0, 1]
    regular_capsule.material.color = [1.0, 0.0, 0.0]  # Red
    scene.add_object("regular", regular_capsule)
    
    # Test 2: Tapered capsule (smaller top, larger bottom)
    tapered_capsule1 = Capsule(
        radius_top=0.3,
        radius_bottom=0.7,
        height=2.0
    )
    tapered_capsule1.position = [0, 0, 1]
    tapered_capsule1.material.color = [0.0, 1.0, 0.0]  # Green
    scene.add_object("tapered1", tapered_capsule1)
    
    # Test 3: Reverse taper (larger top, smaller bottom)
    tapered_capsule2 = Capsule(
        radius_top=0.8,
        radius_bottom=0.2,
        height=2.0
    )
    tapered_capsule2.position = [3, 0, 1]
    tapered_capsule2.material.color = [0.0, 0.0, 1.0]  # Blue
    scene.add_object("tapered2", tapered_capsule2)
    
    # Test 4: Extreme taper (very thin to thick)
    extreme_capsule = Capsule(
        radius_top=0.1,
        radius_bottom=1.0,
        height=3.0
    )
    extreme_capsule.position = [6, 0, 1.5]
    extreme_capsule.material.color = [1.0, 1.0, 0.0]  # Yellow
    scene.add_object("extreme", extreme_capsule)
    
    # Add ground plane
    ground = Plane(12, 8)
    ground.material.color = [0.8, 0.8, 0.8]
    scene.add_object("ground", ground)
    
    print("🎯 TAPERED CAPSULE SHOWCASE")
    print("==========================")
    print("Red:    Regular capsule (radius=0.5) - backward compatibility")
    print("Green:  Tapered capsule (top=0.3, bottom=0.7) - smaller to larger")
    print("Blue:   Reverse taper (top=0.8, bottom=0.2) - larger to smaller") 
    print("Yellow: Extreme taper (top=0.1, bottom=1.0) - very dramatic taper")
    print()
    print("💡 Usage:")
    print("   Capsule(radius=0.5)                    # Regular capsule")
    print("   Capsule(radius_top=0.3, radius_bottom=0.7)  # Tapered capsule")
    print()
    print("🎨 Perfect for: bones, limbs, organic shapes, realistic ragdolls")
    print("\nPress ESC to exit")
    
    # Start scene
    scene.run()

if __name__ == "__main__":
    main()
