import math
import os
from urllib import request

import sdl2

from payton.math.functions import add_vectors, distance, normalize_vector, scale_vector
from payton.scene import SHADOW_HIGH, Scene
from payton.scene.controller import BaseController
from payton.scene.geometry import Cube, Sphere, Wavefront
from payton.scene.geometry.awp3d import AWP3D

IDLE = "idle"
WALK = "walk"
RUN = "run"
JUMP = "jump"
THROW = "throw"


class GameController(BaseController):
    """Game control overrides for the scene controller."""

    def mouse(self, evemt, scene):
        pass

    def keyboard(self, event, scene):
        """Additional keyboard controls.

        Space = JUMP
        e     = Throw a ball
        """
        if event.type == sdl2.SDL_KEYDOWN:
            key = event.key.keysym.sym
            hero = scene.objects["hero"]
            if key == sdl2.SDLK_SPACE and hero.animation != JUMP:
                hero.animation = JUMP
                hero.jump = True
                hero._time = 0
                hero._frame = 0
                hero.run_animation(JUMP)
            if key == sdl2.SDLK_e and not hero.throw and not hero.out_of_ammo:
                hero.run_animation(THROW)
                hero.animation = THROW
                hero._time = 0
                hero._frame = 0
                hero.throw = True


class Hero(AWP3D):
    """Hero object."""

    def __init__(self, scene, *args, **kwargs):
        """Initialize the hero."""
        super().__init__(*args, **kwargs)
        self._target = self.position
        self.animation = IDLE
        self.scene = scene
        self.set_animation(IDLE, 0, 47)
        self.set_animation(WALK, 48, 73)
        self.set_animation(RUN, 74, 91)
        self.set_animation(JUMP, 92, 138)
        self.set_animation(THROW, 139, 232)
        self.direction = [0, 1, 0]
        self.throw = False
        self.thrown = False
        self.out_of_ammo = False
        self.jump = False

    def target(self, target_point):
        """Set target."""
        if self.throw or self.jump:
            return

        self.direct_to(target_point)
        dist = distance(self.position, target_point)
        if dist > 5:
            # we are running
            self.animation = RUN
            self.run_animation(RUN)
        if dist < 5:
            self.animation = WALK
            self.run_animation(WALK)

        self._target = target_point

    def time_forward(self, period):
        """Move in time."""
        distance_to_target = distance(self.position, self._target)

        if self.animation == RUN:
            self.forward(0.20)

        if self.animation == WALK:
            self.forward(0.07)

        if self.animation == JUMP:
            if self._frame > 13 and self._frame < 26:
                self.forward(0.2)
            if self._frame >= 45:
                self._target = self.position
                self.animation = IDLE
                self.run_animation(IDLE)
                self.jump = False
            return

        if self.animation == THROW:
            if self._frame > 37 and not self.thrown:
                self.thrown = True
                ball = self.scene.balls[self.scene.ball]
                ball.show()
                ball.set_position(self.position[0], self.position[1], 1.72, True)
                ball.mass = 2
                ball.linear_velocity = scale_vector(self.direction, 40)
                self.scene.ball += 1
                if self.scene.ball == 100:
                    self.out_of_ammo = True

            if self._frame < 92:
                return
            self.run_animation(IDLE)
            self.animation = IDLE
            self.throw = False
            self.thrown = False

        if distance_to_target < 0.1:
            self.run_animation(IDLE)
            self.animation = IDLE
            self.jump = False


class Game(Scene):
    """Main game wrapper."""

    def __init__(self, *args, **kwargs):
        """Initialize the scene."""
        super().__init__(*args, **kwargs)
        self.hero = Hero(self, filename="assets/thebot.awp3d")
        self.hero.run_animation(IDLE)
        self.hero.scene = self
        self.hero.mass = 0
        self.hero.position = [0, 0, -0.2]
        self.ground = Wavefront("./assets/map_ground.obj")  # Plane(width=20, height=20)
        self.ground.change_dynamics(restitution=1, lateralFriction=2.0, linearDamping=10, angularDamping=10)
        self.ground.mass = 0  # make it stationary
        self.ground.position = [0, 0, 0]
        self.cameras[0].target_object = self.hero
        self.lights[0].position = [-2.24655, 33.1663, 9.07499]
        self.active_camera.min_elevation = math.radians(30)
        self.active_camera.max_elevation = math.radians(60)
        self.active_camera.max_distance = 20

        self.ground_rest = Wavefront("./assets/map_rest.obj")
        self.ground_rest.change_dynamics(restitution=1)
        for mat in self.ground_rest.materials:
            self.ground_rest.materials[mat].culling = True
        self.ground_rest.mass = 0

        self._generate_ammo()

        self.add_object("hero", self.hero)
        self.add_object("ground", self.ground)
        self.add_object("rest", self.ground_rest)

        self.cursor = Sphere(radius=0.1)
        self.add_object("cursor", self.cursor)
        self.shadow_quality = SHADOW_HIGH
        self.controller.add_controller(GameController())
        self.grid.visible = False

        self.add_click_plane([0, 0, 0], [0, 0, 1], self._ground_click)
        self.create_clock("time", 0.03333, self.scene_clock, True)
        self.create_clock("zone_check", 0.05, self.control_objects, True)
        self.background.top_color = [0, 0, 0]
        self.background.bottom_color = [0, 0, 0]
        self.lights[0].color = [0.6, 0.6, 0.6]
        self._create_wall()

    def _create_wall(self):
        x = -5
        z = 1
        for i in range(10):
            for j in range(5):
                cube = Cube(width=1, depth=1, height=1)
                cube.material.texture = 'assets/cube.png'
                cube.mass = 1.0
                cube.position = [10, x, z]
                z += 1.01
                self.add_object(f"block_{i}_{j}", cube)
            x += 1
            z = 0.51

    def control_objects(self, period, total):
        """Control collisions."""
        dir_vec = normalize_vector(add_vectors(self.hero.direction, [0, 0, -1]))
        hits = self.raycast_intersect(
            [self.hero.position[0], self.hero.position[1], 1],
            dir_vec,
            box_only=False,
            exempt_objects=[],
        )
        if not hits or "block" in hits[0].name or "rest" in hits[0].name:
            self.hero.forward(-0.2)
            print("should back off")
            self.hero._target = self.hero.position
            self.hero.animation = IDLE
            self.hero.run_animation(IDLE)

    def _ground_click(self, cursor_position):
        self.cursor.position = cursor_position
        self.hero.target(cursor_position)

    def _generate_ammo(self):
        self.balls = [Sphere(radius=0.15, mass=0, parallels=6, meridians=6) for i in range(100)]
        for i, ball in enumerate(self.balls):
            ball.material.color = [1, 0, 0]
            ball.hide()
            ball.change_dynamics(restitution=0.8)
            self.add_object(f"ball_{i}", ball)

        self.ball = 0

    def scene_clock(self, period, total):
        """Animate the scene."""
        self.hero.time_forward(period)


if not os.path.exists('assets/thebot.awp3d'):
    print("Downloading the bot asset")
    print("The file is 123mb which can not fit into github")
    request.urlretrieve("https://www.islekdemir.com/thebot.awp3d", "assets/thebot.awp3d")
    print("Download complete")

game = Game()
game.run()
