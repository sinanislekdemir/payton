import math
import random

import sdl2

from payton.math.functions import distance
from payton.scene import Scene
from payton.scene.controller import Controller
from payton.scene.geometry import Cube
from payton.scene.gui import info_box


class MinecraftApp(Scene):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.blocks = {}
        self.setup_scene()
        self.setup_player()
        self.setup_controls()
        self.create_clock("update", 0.01, self.update_game)

    def setup_scene(self):
        self.background.top_color = [0.5, 0.8, 1.0, 1.0]
        self.background.bottom_color = [0.1, 0.6, 0.1, 1.0]
        self.grid.visible = False
        
        # Terrain Generation
        # Area: 100x100
        # Height: up to 50
        print("Generating terrain...")
        for x in range(-50, 50):
            for y in range(-50, 50):
                # Simple heightmap using sin/cos for speed and waviness
                # You could use Perlin noise for better results
                h = int((math.sin(x / 10.0) + math.cos(y / 10.0)) * 5) + 5
                h = max(1, min(h, 49)) # Keep within bounds
                
                # Create the surface block
                self.add_block(x, y, h)
                
                # Fill downwards a bit so we don't see holes if we dig
                for d in range(1, 4):
                    if h - d >= 0:
                        self.add_block(x, y, h - d, visible=False)
        
        self.add_object("info", info_box(left=10, top=10, label="WASD: Move\nMouse: Look\nL-Click: Dig\nR-Click: Place\nESC: Quit"))

    def add_block(self, x, y, z, visible=True):
        if (x, y, z) in self.blocks:
            return
        
        cube = Cube()
        cube.position = [x, y, z]
        # Color based on height
        if z > 8:
            cube.material.color = [0.9, 0.9, 0.9] # Snow
        elif z > 4:
            cube.material.color = [0.5, 0.5, 0.5] # Rock
        else:
            cube.material.color = [0.2, 0.8, 0.2] # Grass

        name = f"blk_{x}_{y}_{z}"
        self.add_object(name, cube)
        self.blocks[(x, y, z)] = cube

    def setup_player(self):
        self.cameras[0].position = [0, 0, 15]
        self.cameras[0].target = [10, 10, 12]
        self.cameras[0].near = 0.1
        self.cameras[0].far = 200
        
        self.camera_yaw = 0.0
        self.camera_pitch = 0.0
        self.mouse_sensitivity = 0.2
        self.move_speed = 0.3
        
        # Lock mouse (basic implementation, SDL relative mouse mode ideally)
        sdl2.SDL_SetRelativeMouseMode(sdl2.SDL_TRUE)

    def setup_controls(self):
        # We'll use a custom controller to handle inputs
        self.controller.add_controller(PlayerController(self))
        # Hook up selection for digging
        self.on_select = self.dig_block

    def dig_block(self, selected_list):
        if not selected_list:
            return
        
        # Dig the first object hit
        obj = selected_list[0]
        # Find coordinates
        pos = obj.position
        coord = (int(round(pos[0])), int(round(pos[1])), int(round(pos[2])))
        
        if coord in self.blocks:
            del self.blocks[coord]
            # Remove from scene
            name = f"blk_{coord[0]}_{coord[1]}_{coord[2]}"
            if name in self.objects:
                del self.objects[name]
                
    def place_block(self):
        # Raycast from camera to find where to place
        # Scene.raycast_intersect(start, direction)
        
        # Get camera vectors
        # Camera direction is target - position
        cam = self.cameras[0]
        direction = [cam.target[0] - cam.position[0],
                     cam.target[1] - cam.position[1],
                     cam.target[2] - cam.position[2]]
        
        hit = self.raycast_intersect(cam.position, direction, box_only=False)
        
        if hit:
            obj, point = hit
            # Calculate placement coordinate
            # The point is on the surface. We need to find the adjacent integer coordinate.
            # Simple way: find the center of the hit cube, and see which face is closest to point.
            
            # This is an approximation
            pos = obj.position
            cx, cy, cz = int(round(pos[0])), int(round(pos[1])), int(round(pos[2]))
            
            px, py, pz = point
            
            # Determine face
            dx = px - cx
            dy = py - cy
            dz = pz - cz
            
            # Normal logic
            nx, ny, nz = 0, 0, 0
            # Use a small epsilon for float comparison on face
            if abs(dx - 0.5) < 0.1: nx = 1
            elif abs(dx + 0.5) < 0.1: nx = -1
            elif abs(dy - 0.5) < 0.1: ny = 1
            elif abs(dy + 0.5) < 0.1: ny = -1
            elif abs(dz - 0.5) < 0.1: nz = 1
            elif abs(dz + 0.5) < 0.1: nz = -1
            else:
                # Fallback to absolute max difference
                if abs(dx) > abs(dy) and abs(dx) > abs(dz):
                    nx = 1 if dx > 0 else -1
                elif abs(dy) > abs(dx) and abs(dy) > abs(dz):
                    ny = 1 if dy > 0 else -1
                else:
                    nz = 1 if dz > 0 else -1
                
            new_x, new_y, new_z = cx + nx, cy + ny, cz + nz
            self.add_block(new_x, new_y, new_z)

    def update_game(self, period, total):
        # Movement logic is handled in the controller updating variables, 
        # but actual position update can happen here to be smooth
        pass


class PlayerController(Controller):
    def __init__(self, app):
        super().__init__()
        self.app = app
        self.forward = 0
        self.right = 0
        self.up = 0
        
    def keyboard(self, event, scene):
        if event.type == sdl2.SDL_KEYDOWN:
            key = event.key.keysym.sym
            if key == sdl2.SDLK_w:
                self.forward = 1
            elif key == sdl2.SDLK_s:
                self.forward = -1
            elif key == sdl2.SDLK_a:
                self.right = -1
            elif key == sdl2.SDLK_d:
                self.right = 1
            elif key == sdl2.SDLK_ESCAPE:
                self.app.running = False
            
        elif event.type == sdl2.SDL_KEYUP:
            key = event.key.keysym.sym
            if key in [sdl2.SDLK_w, sdl2.SDLK_s]:
                self.forward = 0
            if key in [sdl2.SDLK_a, sdl2.SDLK_d]:
                self.right = 0

        # Apply movement
        if self.forward != 0 or self.right != 0:
            self.move_player()

    def mouse(self, event, scene):
        # Mouse look
        if event.type == sdl2.SDL_MOUSEMOTION:
            if sdl2.SDL_GetRelativeMouseMode() == sdl2.SDL_TRUE:
                xrel = event.motion.xrel
                yrel = event.motion.yrel
                
                self.app.camera_yaw += xrel * self.app.mouse_sensitivity
                self.app.camera_pitch -= yrel * self.app.mouse_sensitivity
                
                # Clamp pitch
                self.app.camera_pitch = max(-89, min(89, self.app.camera_pitch))
                
                self.update_camera_target()
        
        elif event.type == sdl2.SDL_MOUSEBUTTONDOWN:
            if event.button.button == sdl2.SDL_BUTTON_RIGHT:
                self.app.place_block()
            # Left click is handled by scene.on_select which triggers internally on click
                
    def update_camera_target(self):
        # Calculate new target from yaw/pitch
        rad_yaw = math.radians(self.app.camera_yaw)
        rad_pitch = math.radians(self.app.camera_pitch)
        
        # Spherical coords
        # x = r * cos(pitch) * sin(yaw)
        # y = r * cos(pitch) * cos(yaw) 
        # z = r * sin(pitch)
        # Note: Payton coordinates might be slightly different, let's test.
        # Usually Z is up in Payton.
        
        # Forward vector
        fx = math.sin(rad_yaw) * math.cos(rad_pitch)
        fy = math.cos(rad_yaw) * math.cos(rad_pitch)
        fz = math.sin(rad_pitch)
        
        cam = self.app.cameras[0]
        # Target is position + forward * distance
        cam.target = [cam.position[0] + fx * 10,
                      cam.position[1] + fy * 10,
                      cam.position[2] + fz * 10]

    def move_player(self):
        speed = self.app.move_speed
        
        # Calculate movement vectors based on Yaw only (don't fly up/down when walking)
        rad_yaw = math.radians(self.app.camera_yaw)
        
        # Forward vector on ground plane
        fx = math.sin(rad_yaw)
        fy = math.cos(rad_yaw)
        
        # Right vector (rotate forward by -90 deg)
        rx = math.sin(rad_yaw - math.pi/2)
        ry = math.cos(rad_yaw - math.pi/2)
        
        dx = (fx * self.forward + rx * self.right) * speed
        dy = (fy * self.forward + ry * self.right) * speed
        
        cam = self.app.cameras[0]
        new_x = cam.position[0] - dx # Checks direction
        new_y = cam.position[1] - dy
        new_z = cam.position[2]
        
        # Collision Check
        # Simple point collision
        cx_i = int(round(new_x))
        cy_i = int(round(new_y))
        cz_i = int(round(new_z - 1.5)) # Check feet range
        
        # Basic wall collision
        # If the target block exists and is close enough, don't move
        # For simple Minecraft, check if (floor(x), floor(y), floor(z)) is solid
        
        # Let's verify feet and head
        if (complex_collision(self.app, new_x, new_y, new_z)):
             # Blocked
             pass
        else:
            cam.position = [new_x, new_y, new_z]
            self.update_camera_target()

def complex_collision(app, x, y, z):
    # Check body radius?
    # Simple check: center point
    bx = int(round(x))
    by = int(round(y))
    # Eyes are at z. Feet at z-2 approx?
    # Let's assume camera is at approx 1.7m height
    feet_z = int(round(z - 1.5))
    head_z = int(round(z - 0.5))
    
    if (bx, by, feet_z) in app.blocks: return True
    if (bx, by, head_z) in app.blocks: return True
    return False

if __name__ == "__main__":
    app = MinecraftApp()
    app.run()
