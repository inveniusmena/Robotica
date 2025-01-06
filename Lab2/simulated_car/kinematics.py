import pygame
import math
import numpy as np

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
SENSOR_RANGE = 450  # 30 meters in pixels
SENSOR_ANGLE = 20  # Sensor cone angle (degrees)
dt = 0.1  # Time step for simulation

# Open a file to save detected positions
file = open("detected_positions.txt", "w")

# Function for Runge-Kutta integration
def runge_kutta(car_x, car_y, car_angle, car_speed, steering_angle):
    """
    Fourth-order Runge-Kutta integration for the car's position and angle.
    """
    def derivatives(state, steering_angle):
        x, y, angle = state
        dx = car_speed * math.cos(angle)
        dy = car_speed * math.sin(angle)
        dtheta = (car_speed / L) * math.tan(math.radians(steering_angle))
        return dx, dy, dtheta

    # Initial state
    state = (car_x, car_y, car_angle)

    # Compute Runge-Kutta coefficients
    k1 = derivatives(state, steering_angle)
    k2 = derivatives(
        (state[0] + k1[0] * dt / 2, state[1] + k1[1] * dt / 2, state[2] + k1[2] * dt / 2),
        steering_angle,
    )
    k3 = derivatives(
        (state[0] + k2[0] * dt / 2, state[1] + k2[1] * dt / 2, state[2] + k2[2] * dt / 2),
        steering_angle,
    )
    k4 = derivatives(
        (state[0] + k3[0] * dt, state[1] + k3[1] * dt, state[2] + k3[2] * dt),
        steering_angle,
    )

    # Combine coefficients
    dx = (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]) / 6
    dy = (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]) / 6
    dtheta = (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]) / 6

    # Update state
    return state[0] + dx * dt, state[1] + dy * dt, state[2] + dtheta * dt


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
    else:
        car_speed *= 0.95  # Friction effect when no key is pressed

    # Update car state using Runge-Kutta integration
    car_x, car_y, car_angle = runge_kutta(car_x, car_y, car_angle, car_speed, steering_angle)

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
