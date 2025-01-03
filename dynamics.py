import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1000, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car on a Road")

# Load road images
road_images = ["C:/Users/filom/Downloads/road1.png", "C:/Users/filom/Downloads/road2.png", "C:/Users/filom/Downloads/road3.png"] 
current_road = 0
road = pygame.image.load(road_images[current_road])

# Car attributes
car = pygame.image.load("C:/Users/filom/Downloads/car.png")
car_width, car_height = car.get_width(), car.get_height()
car_x, car_y = 50, HEIGHT // 2 + 23  # Starting position of the car
theta = 0 # Car angle

# Constants for dynamics
L = 1000  # Width in pixels ?
L = 2.5  # Width in meters
a = L*2/3
b = 1/3*L
MAX_V = 5 # Maximum velocity
MAX_STEER_ANGLE = 30  # Maximum steering angle in degrees
M = 1200  # Mass of the car in kg
J = 2000  # Moment of inertia about 
FA, FD = -10, 0 #Friction, driving force
tau_s, c_s = 1, 1 #constants for steering

# Simulation constants
DT = 0.1  # Time step in seconds

# Sensor constants
SENSOR_RANGE = 450  # 30 meters in pixels
SENSOR_ANGLE = 20   # Sensor cone angle (degrees)
 
# State variables: [x, y, theta, v_u, phi, u_2 ]
state = [car_x, car_y, math.radians(theta), 0, 0, 0]
# Helper function: Runge-Kutta integration
def runge_kutta(f, state, dt):
    #print("inside runge kutta fucntion", state)
    k1 = f(state)
    #print("k1:",k1)
    k2 = f([s + dt * 0.5 * k for s, k in zip(state, k1)])
    #print("k2:",k2)
    k3 = f([s + dt * 0.5 * k for s, k in zip(state, k2)])
    #print("k3:",k3)
    k4 = f([s + dt * k for s, k in zip(state, k3)])
    #print("k4:",k4)
    return [s + dt / 6 * (k1i + 2 * k2i + 2 * k3i + k4i) for s, k1i, k2i, k3i, k4i in zip(state, k1, k2, k3, k4)]

# Dynamics model
def car_dynamics(state):
    x, y, theta, v_u, phi, u_2 = state
    global FA, FD # Access global variables
    print(FD, phi)

    # Clamp steering angle
    phi = max(-math.radians(MAX_STEER_ANGLE), min(math.radians(MAX_STEER_ANGLE), phi))

    # Precomputed values
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)
    gamma = (cos_phi ** 2) * (L**2 * M + (M * (b)**2 + J) * (tan_phi ** 2))

    # Equations of motion (from Equation 2.23 in the PDF)
    dx = v_u * ( math.cos(theta) -b/L *math.tan(theta)*math.sin(theta))
    dy = v_u *( math.sin(theta) +b/L *math.tan(theta)*math.cos(theta))
    dtheta = v_u * tan_phi / L
    dphi = (1 / tau_s) * (phi + c_s * u_2)  # Steering dynamics
    dv_u = (v_u*(b**2*M+J)*tan_phi * dphi + L**2 * cos_phi**2) / gamma * (FD+FA)
    state[4] = 0
    state[5] = 0
    return [dx, dy, dtheta, dv_u, dphi, 0]

# Main loop
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Input handling
    keys = pygame.key.get_pressed()

    if keys[pygame.K_UP]:
        if state[5] > -5:
            state[5] -= 0.1  # Steer left
    elif keys[pygame.K_DOWN]:
        if state[5] < 5:    
            state[5] += 0.1  # Steer right
    else:
        state[5]=0

    if keys[pygame.K_LEFT]:
        FD = max(FD-100, -10000)  # Decrease driving force (Newtons)
    elif keys[pygame.K_RIGHT]:
        FD = min(FD+100, 10000)  # Increase driving force (Newtons)
    else:
        FD=0
    

    # Update state using Runge-Kutta integration
    state = runge_kutta(car_dynamics, state, DT)


    # Check for end of road (when car reaches the right edge of the current road)
    road_width = road.get_width()  # Get the width of the current road image
    if state[0] >= road_width:  # Reached the right edge of the screen
        current_road += 1
        if current_road < len(road_images):
            road = pygame.image.load(road_images[current_road])
            state[0] = 0  # Reset car position to the left edge
        else:
            print("End of map, congrats!")
            running = False

    # Draw the background (road)
    screen.blit(road, (0, 0))

    # Calculate the front of the car
    car_front_x = state[0] 
    car_front_y = state[1] 

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

    for angle_offset in range(-SENSOR_ANGLE // 2, SENSOR_ANGLE // 2 + 1, 2):  # Steps within the cone
        sensor_angle = - math.radians(theta - angle_offset)  # Adjust for rotation
        for distance in range(1, SENSOR_RANGE, 5):  # Incremental steps along the ray
            sensor_x = int(car_front_x + distance * math.cos(sensor_angle))
            sensor_y = int(car_front_y - distance * math.sin(sensor_angle))

            # Ensure the sensor point is within screen bounds
            if 0 <= sensor_x < WIDTH and 0 <= sensor_y < HEIGHT:
                # Check the color of the pixel at the sensor point
                pixel_color = road.get_at((sensor_x, sensor_y))[:3]  # Ignore alpha channel
                if pixel_color in LINE_COLORS:
                    detected_lines.append((sensor_x, sensor_y))  # Record detected line position
                    pygame.draw.circle(screen, (255, 0, 0), (sensor_x, sensor_y), 3)  # Highlight detected point
                    # Save the position to the file
                    #file.write(f"{sensor_x} \t {sensor_y}\n")

                    break  # Stop the ray once a line is detected

            # Draw the sensor ray
            pygame.draw.line(screen, (0, 255, 0), (car_front_x, car_front_y), (sensor_x, sensor_y), 1)

    # Draw detected lines (if any)
    for line_pos in detected_lines:
        pygame.draw.circle(screen, (255, 0, 0), line_pos, 5)  # Red dots for detected lines


    # Draw car
    rotated_car = pygame.transform.rotate(car, -math.degrees(state[2]))
    rect = rotated_car.get_rect(center=(state[0], state[1]))
    screen.blit(rotated_car, rect.topleft)

    # Refresh screen
    pygame.display.flip()
    pygame.time.Clock().tick(60)

pygame.quit()
