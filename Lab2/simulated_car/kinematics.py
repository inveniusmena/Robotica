import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1000, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car on a Road")

# Load road images
road_images = ["road1.png", "road2.png", "road3.png"]
current_road = 0

# Attempt to load the first road image
try:
    road = pygame.image.load(road_images[current_road])
except pygame.error as e:
    print(f"Error loading road image: {e}")
    pygame.quit()
    exit()

# Load car image
try:
    car = pygame.image.load("car.png")
except pygame.error as e:
    print(f"Error loading car image: {e}")
    pygame.quit()
    exit()

car_width, car_height = car.get_width(), car.get_height()
car_x, car_y = 50, HEIGHT // 2 + 23  # Starting position of the car
car_angle = 0
car_speed = 0

# Constants
L = 2.5  # Wheelbase in meters (adjust this based on your car)
ACCELERATION = 1
MAX_SPEED = 20
TURN_SPEED = 2
dt = 0.1  # Time step for simulation
FRICTION = 0.98  # Friction coefficient (reduce speed by 2% per update)

# Open a file to save detected positions
file = open("detected_positions.txt", "w")

# New Runge-Kutta function (provided by user)
def runge_kutta(f, state, dt):
    k1 = f(state)
    k2 = f([s + dt * 0.5 * k for s, k in zip(state, k1)])
    k3 = f([s + dt * 0.5 * k for s, k in zip(state, k2)])
    k4 = f([s + dt * k for s, k in zip(state, k3)])
    return [s + dt / 6 * (k1i + 2 * k2i + 2 * k3i + k4i) for s, k1i, k2i, k3i, k4i in zip(state, k1, k2, k3, k4)]

# Derivatives function for the car's motion
def car_derivatives(state):
    """
    Compute the car's derivatives based on its current state.
    state: [x, y, angle]
    """
    x, y, angle = state
    dx = car_speed * math.cos(angle)
    dy = car_speed * math.sin(angle)
    dtheta = (car_speed / L) * math.tan(math.radians(steering_angle))
    return [dx, dy, dtheta]

# Main loop
running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Input handling for steering and speed
    keys = pygame.key.get_pressed()

    # Steering angle input
    if keys[pygame.K_LEFT]:  # Turn left (counter-clockwise)
        steering_angle = TURN_SPEED
    elif keys[pygame.K_RIGHT]:  # Turn right (clockwise)
        steering_angle = -TURN_SPEED
    else:
        steering_angle = 0  # No steering change

    # Speed control (accelerate or decelerate)
    if keys[pygame.K_UP]:  # Move forward
        car_speed = min(car_speed + ACCELERATION, MAX_SPEED)
    elif keys[pygame.K_DOWN]:  # Move backward
        car_speed = max(car_speed - ACCELERATION, -MAX_SPEED)

    # Apply road friction to car speed
    car_speed *= FRICTION

    # Update car state using the new Runge-Kutta function
    car_x, car_y, car_angle = runge_kutta(car_derivatives, [car_x, car_y, car_angle], dt)

    # Ensure car_x and car_y are within valid range
    car_x = max(0, min(WIDTH, car_x))
    car_y = max(0, min(HEIGHT, car_y))

    # Check for end of road (when car reaches the right edge of the current road)
    road_width = road.get_width()  # Get the width of the current road image
    if car_x >= road_width:  # Reached the right edge of the screen
        current_road += 1
        if current_road < len(road_images):
            try:
                road = pygame.image.load(road_images[current_road])
            except pygame.error as e:
                print(f"Error loading road image: {e}")
                pygame.quit()
                exit()
            car_x = 0  # Reset car position to the left edge
        else:
            print("End of map, congrats!")
            running = False

    # Draw the background (road)
    screen.blit(road, (0, 0))

    # Rotate and draw the car
    rotated_car = pygame.transform.rotate(car, -math.degrees(car_angle))
    rect = rotated_car.get_rect(center=(car_x, car_y))
    screen.blit(rotated_car, rect.topleft)

    # Refresh screen
    pygame.display.flip()
    pygame.time.Clock().tick(60)

# Close the file after quitting
file.close()
pygame.quit()
