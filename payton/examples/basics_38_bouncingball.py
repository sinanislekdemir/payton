from payton.scene import Scene
from payton.scene.geometry import Plane, Sphere

# Create a scene
game = Scene()
game.lights[0].position = [50, 50, 50]

# Create the ground
ground = Plane(10, 10)
game.add_object("ground", ground)

# Create the ball
ball = Sphere()
ball.position = [0, 0, 6]
game.add_object("ball", ball)

# Ball physics
velocity = [0.1, 0.08, 0.05]  # Initial velocity in [x, y, z]
gravity = -0.005  # Gravity effect


def bounce(period, total):
    global velocity
    pos = ball.position
    pos[0] += velocity[0]  # Move in x direction
    pos[1] += velocity[1]  # Move in y direction
    pos[2] += velocity[2]  # Move in z direction
    velocity[2] += gravity  # Apply gravity

    # Bounce on ground
    if pos[2] <= 0:
        pos[2] = 0
        velocity[2] = -velocity[2] * 0.9  # Energy loss

    # Bounce on walls
    if abs(pos[0]) >= 4.9:
        velocity[0] = -velocity[0]  # Reverse x direction
    if abs(pos[1]) >= 4.9:
        velocity[1] = -velocity[1]  # Reverse y direction

    ball.position = pos


game.create_clock("bounce", 0.02, bounce)
game.run(start_clocks=True)
