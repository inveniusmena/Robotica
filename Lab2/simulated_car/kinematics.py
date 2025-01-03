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

# Main loop
running = True
while running:

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN or event.type == pygame.KEYUP:
            print(f"Key event: {event}")  # Debug print statement

    # Input handling for steering and speed
    keys = pygame.key.get_pressed()
    print(f"keys: {keys}")  # Debug print statement

    # Steering angle input
    if keys[pygame.K_LEFT]:  # Turn left (counter-clockwise)
        steering_angle = TURN_SPEED
    elif keys[pygame.K_RIGHT]:  # Turn right (clockwise)
        steering_angle = -TURN_SPEED
    else:
        steering_angle = 0  # No steering change
    print(f"steering_angle: {steering_angle}")  # Debug print statement

    # Speed control (accelerate or decelerate)
    if keys[pygame.K_UP]:  # Move forward
        car_speed = min(car_speed + ACCELERATION, MAX_SPEED)
    elif keys[pygame.K_DOWN]:  # Move backward
        car_speed = max(car_speed - ACCELERATION, -MAX_SPEED)
    else:
        car_speed *= 0.95  # Friction effect when no key is pressed
    print(f"car_speed: {car_speed}")  # Debug print statement

    # Update car angle (steering control)
    delta = math.radians(steering_angle)  # Steering angle in radians

    # Bicycle Model for car's motion
    car_x += car_speed * math.cos(car_angle) * dt  # x position update
    car_y += car_speed * math.sin(car_angle) * dt  # y position update
    car_angle += (car_speed / L) * math.tan(delta) * dt  # Heading angle update

    # Ensure car_x and car_y are within valid range
    car_x = max(0, min(WIDTH, car_x))
    car_y = max(0, min(HEIGHT, car_y))

    # Debug print statements for key variables
    print(f"car_x: {car_x}, car_y: {car_y}, car_angle: {car_angle}")

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

    # Calculate the front of the car (based on the current position and angle)
    car_front_x = car_x + car_width / 2 * math.cos(car_angle)
    car_front_y = car_y + car_height / 2 * math.sin(car_angle)

    # Define the color of the road lines (e.g., white lines)
    LINE_COLORS = [
        (250, 253, 253),  # #fafdfd
        (206, 213, 205),  # #ced5cd
        (140, 136, 129),  # #8c8881
        (229, 230, 229),  # #e5e6e5
    ]

    # Sensor simulation (anchored at the front of the car)
    sensor_points = []
    detected_lines = []  # To store the positions of detected lines

    for angle_offset in range(-SENSOR_ANGLE // 2, SENSOR_ANGLE // 2 + 1,
                              2):  # Steps within the cone
        sensor_angle = - math.radians(
            car_angle - angle_offset)  # Adjust for rotation
        for distance in range(1, SENSOR_RANGE,
                              5):  # Incremental steps along the ray
            sensor_x = int(car_front_x + distance * math.cos(sensor_angle))
            sensor_y = int(car_front_y - distance * math.sin(sensor_angle))

            # Ensure the sensor point is within screen bounds
            if 0 <= sensor_x < WIDTH and 0 <= sensor_y < HEIGHT:
                # Check the color of the pixel at the sensor point
                pixel_color = road.get_at((sensor_x, sensor_y))[
                              :3]  # Ignore alpha channel
                if pixel_color in LINE_COLORS:
                    detected_lines.append(
                        (sensor_x, sensor_y))  # Record detected line position
                    pygame.draw.circle(screen, (255, 0, 0),
                                       (sensor_x, sensor_y),
                                       3)  # Highlight detected point
                    break  # Stop the ray once a line is detected

            # Draw the sensor ray
            pygame.draw.line(screen, (0, 255, 0), (car_front_x, car_front_y),
                             (sensor_x, sensor_y), 1)

    # Draw detected lines (if any)
    for line_pos in detected_lines:
        pygame.draw.circle(screen, (255, 0, 0), line_pos,
                           5)  # Red dots for detected lines

    # Rotate and draw the car
    rotated_car = pygame.transform.rotate(car, -math.degrees(
        car_angle))  # Negative angle to match screen coordinates
    rect = rotated_car.get_rect(center=(car_x, car_y))
    screen.blit(rotated_car, rect.topleft)

    # Refresh screen
    pygame.display.flip()
    pygame.time.Clock().tick(60)

# Close the file after quitting
file.close()

pygame.quit()
