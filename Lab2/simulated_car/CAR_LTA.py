import pygame
import math

# Initialize Pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 1000, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Car Simulation")

# Load road images
road_images = ["road1.png", "road2.png", "road3.png"]
current_road = 0
road = pygame.image.load(road_images[current_road])

# Open a file to save detected positions
file = open("detected_positions.txt", "w")

# Load car image
car = pygame.image.load("car.png")
car_width, car_height = car.get_width(), car.get_height()
car_x, car_y = 50, HEIGHT // 2 + 23  # Starting position of the car
car_angle = 0
car_speed = 0

# Constants
L = 2.5  # Wheelbase in meters (adjust this based on your car)
ACCELERATION = 1
MAX_SPEED = 10
INCREMENT = 0.1
SENSOR_RANGE = 450  # 30 meters in pixels
SENSOR_ANGLE = 20  # Sensor cone angle (degrees)
DT = 0.1  # Time step for simulation
a = L * 2 / 3
b = 1 / 3 * L
MAX_V = 5  # Maximum velocity
MAX_STEER_ANGLE = 30  # Maximum steering angle in degrees
M = 1200  # Mass of the car in kg
J = 2000  # Moment of inertia about
FA, FD = -10, 0  # Friction, driving force
tau_s, c_s = 1, 1  # constants for steering
FRICTION = 0.98  # Friction coefficient (reduce speed by 2% per update)
STEERING_ANGLE = 0  # Initial steering angle
SAFE_DISTANCE_THRESHOLD = 25

# Colors for sensor line detection
LINE_COLORS = [
    (250, 253, 253),  # #fafdfd
    (206, 213, 205),  # #ced5cd
    (140, 136, 129),  # #8c8881
    (229, 230, 229),  # #e5e6e5
]

# Game state variables
running = True
in_home_screen = True
mode = None  # None, "joystick", "dynamic", "kinematics", or "help"
selected_option = 0  # Default to the first option
sensor_active = False  # Sensor state

# Help text
help_text = """
Welcome to the Simulated Car Game!
Use the arrow keys to navigate the menu.
Press Enter to select an option.
- Joystick Mode: Control the car using a joystick.
- Dynamic Mode: Simulate car dynamics.
- Kinematics Mode: Simulate car kinematics.
- Help: Show this help information.
Press 'q' to return to the home screen.
Press 's' to toggle the sensor.
"""

# State variables: [x, y, car_angle, v_u, phi, u_2 ]
state = [car_x, car_y, math.radians(car_angle), 0, 0, 0]

# Initialize variables to manage unsafe message display
unsafe_start_time = None
display_unsafe_message = False
current_u2_LTA = 0


########## FUNCTIONS ##########

## FUNCTIONS FOR CAR MOVEMENT

# Helper function: Runge-Kutta integration
def runge_kutta(f, state, dt):
    # print("inside runge kutta fucntion", state)
    k1 = f(state)
    # print("k1:",k1)
    k2 = f([s + dt * 0.5 * k for s, k in zip(state, k1)])
    # print("k2:",k2)
    k3 = f([s + dt * 0.5 * k for s, k in zip(state, k2)])
    # print("k3:",k3)
    k4 = f([s + dt * k for s, k in zip(state, k3)])
    # print("k4:",k4)
    return [s + dt / 6 * (k1i + 2 * k2i + 2 * k3i + k4i) for
            s, k1i, k2i, k3i, k4i in zip(state, k1, k2, k3, k4)]


# Dynamics model for the car
def car_dynamics(state):
    x, y, theta, v_u, phi, u_2 = state
    global FA, FD # Access global variables
    global unsafe_start_time, display_unsafe_message

    # Clamp steering angle
    phi = max(-math.radians(MAX_STEER_ANGLE), min(math.radians(MAX_STEER_ANGLE), phi))

    # Precomputed values
    cos_phi = math.cos(phi)
    tan_phi = math.tan(phi)
    gamma = (cos_phi ** 2) * (L**2 * M + (M * (b)**2 + J) * (tan_phi ** 2))

    # Equations of motion (from Equation 2.23 in the PDF)
    #Unsafe approximation to the up line
    if not calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[0] and calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[1] == 1:
        unsafe_start_time = pygame.time.get_ticks()
        display_unsafe_message = True
        u_2_LTA = 0.8
        dx = v_u * ( math.cos(theta) -b/L *math.tan(theta)*math.sin(theta))
        dy = v_u *( math.sin(theta) +b/L *math.tan(theta)*math.cos(theta))
        dtheta = v_u * tan_phi / L
        dphi = (1 / tau_s) * (phi + c_s * (u_2 + u_2_LTA))  # Steering dynamics
        dv_u = (v_u*(b**2*M+J)*tan_phi * dphi + L**2 * cos_phi**2) / gamma * (FD+FA)
        state[4] = 0
        state[5] = 0

    #Unsafe approximation to the down line
    elif not calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[0] and calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[1] == 0:
        unsafe_start_time = pygame.time.get_ticks()
        display_unsafe_message = True
        u_2_LTA = -0.8
        dx = v_u * ( math.cos(theta) -b/L *math.tan(theta)*math.sin(theta))
        dy = v_u *( math.sin(theta) +b/L *math.tan(theta)*math.cos(theta))
        dtheta = v_u * tan_phi / L
        dphi = (1 / tau_s) * (phi + c_s * (u_2 + u_2_LTA))  # Steering dynamics
        dv_u = (v_u*(b**2*M+J)*tan_phi * dphi + L**2 * cos_phi**2) / gamma * (FD+FA)
        state[4] = 0
        state[5] = 0

    #Safe
    else:
        u_2_LTA = 0
        dx = v_u * ( math.cos(theta) -b/L *math.tan(theta)*math.sin(theta))
        dy = v_u *( math.sin(theta) +b/L *math.tan(theta)*math.cos(theta))
        dtheta = v_u * tan_phi / L
        dphi = (1 / tau_s) * (phi + c_s * u_2)  # Steering dynamics
        dv_u = (v_u*(b**2*M+J)*tan_phi * dphi + L**2 * cos_phi**2) / gamma * (FD+FA)
        state[4] = 0
        state[5] = 0
    return [dx, dy, dtheta, dv_u, dphi, 0, u_2_LTA]

# Derivatives function for the car's motion
def car_derivatives(state):
    """
    Compute the car's derivatives based on its current state.
    state: [x, y, angle]
    """
    global STEERING_ANGLE
    global unsafe_start_time, display_unsafe_message
    x, y, angle = state
    dx = car_speed * math.cos(angle)
    dy = car_speed * math.sin(angle)

    #Unsafe approximation to the up line
    if not calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[0] and calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[1] == 1:
        unsafe_start_time = pygame.time.get_ticks()
        display_unsafe_message = True
        u_2_LTA = 0.8
        dtheta = (car_speed / L) * math.tan(math.radians(STEERING_ANGLE + u_2_LTA))
    
    #Unsafe approximation to the down line
    elif not calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[0] and calculate_safe_distance(x, y, detected_lines, SAFE_DISTANCE_THRESHOLD)[1] == 0:
        unsafe_start_time = pygame.time.get_ticks()
        display_unsafe_message = True
        u_2_LTA = -0.8
        dtheta = (car_speed / L) * math.tan(math.radians(STEERING_ANGLE + u_2_LTA))
    else:
        dtheta = (car_speed / L) * math.tan(math.radians(STEERING_ANGLE))
    
    return [dx, dy, dtheta]

# Calculate the minimum distance to the nearest detected line and determine if it's safe, or needs LTS intervention
def calculate_safe_distance(car_x, car_y, detected_lines, safe_threshold):
    min_distance = float('inf')
    min_y = None
    for line_x, line_y in detected_lines:
        distance = math.sqrt((line_x - car_x)**2 + (line_y - car_y)**2)
        if distance < min_distance:
            min_distance = distance
            min_y = line_y

    if not min_y == None:
        # Up line
        if min_y < car_y:
            line_id = 1
            is_safe = min_distance > safe_threshold
            return is_safe, line_id
        
        if min_y > car_y:
            line_id = 0
            is_safe = min_distance > safe_threshold
            return is_safe, line_id
    else:
        return True, 0

def joystick_mode(keys, car_x, car_y):
    if keys[pygame.K_UP]:
        car_y = max(0, car_y - 5)
    if keys[pygame.K_DOWN]:
        car_y = min(HEIGHT, car_y + 5)
    if keys[pygame.K_LEFT]:
        car_x = max(0, car_x - 5)
    if keys[pygame.K_RIGHT]:
        car_x = min(WIDTH, car_x + 5)
    return car_x, car_y


def dynamic_mode(keys, state):
    global FD  # Access global variable

    if keys[pygame.K_UP]:
        if state[5] > -5:
            state[5] -= 0.1  # Steer left
    elif keys[pygame.K_DOWN]:
        if state[5] < 5:
            state[5] += 0.1  # Steer right
    else:
        state[5] = 0

    if keys[pygame.K_LEFT]:
        FD = max(FD - 100, -10000)  # Decrease driving force (Newtons)
    elif keys[pygame.K_RIGHT]:
        FD = min(FD + 100, 10000)  # Increase driving force (Newtons)
    else:
        FD = 0

    # Update state using Runge-Kutta integration
    state = runge_kutta(car_dynamics, state, DT)
    return state


def kinematics_mode(keys, car_x, car_y, car_angle, car_speed):
    global STEERING_ANGLE  # Access global variable
    # Steering angle input
    if keys[pygame.K_DOWN]:  # Move forward
        STEERING_ANGLE += INCREMENT
    elif keys[pygame.K_UP]:  # Move backward
        STEERING_ANGLE -= INCREMENT
    else:
        STEERING_ANGLE = 0  # No steering change

    # Speed control (accelerate or decelerate)
    if keys[pygame.K_LEFT]:  # Turn left (counter-clockwise)
        car_speed = min(car_speed - ACCELERATION, MAX_SPEED)
    elif keys[pygame.K_RIGHT]:  # Turn right (clockwise)
        car_speed = max(car_speed + ACCELERATION, -MAX_SPEED)

    # Apply road friction to car speed
    car_speed *= FRICTION

    # Update car state using the new Runge-Kutta function
    car_x, car_y, car_angle = runge_kutta(car_derivatives,
                                          [car_x, car_y, car_angle], DT)

    return car_x, car_y, car_angle, car_speed


# Fonts
font = pygame.font.SysFont("Bahnschrift", 30)


def draw_text_with_glow(text, position, is_selected, glow_color=(255, 200, 0),
                        text_color=(255, 255, 255),
                        selected_text_color=(0, 0, 0)):
    """Draw text with glow and rectangle only when selected."""
    x, y = position
    option_width, option_height = font.size(text)

    temp_surface = pygame.Surface((option_width + 20, option_height + 15),
                                  pygame.SRCALPHA)
    temp_surface.fill((0, 0, 0, 0))

    if is_selected:
        glow_surface = font.render(text, True, glow_color)
        for offset_x, offset_y in [(-1, -1), (1, -1), (1, 1), (-1, 1), (0, 0)]:
            temp_surface.blit(glow_surface, (10 + offset_x, 10 + offset_y))

        pygame.draw.rect(temp_surface, (255, 255, 255, 255),
                         (0, 0, option_width + 20, option_height + 15), 3)
        text_surface = font.render(text, True, selected_text_color)
        temp_surface.blit(text_surface, (10, 10))
    else:
        text_surface = font.render(text, True, text_color)
        temp_surface.blit(text_surface, (10, 10))

    screen.blit(temp_surface, (x - 10, y - 10))


def draw_home_screen(selected):
    """Draw the home screen."""
    if hasattr(draw_home_screen, "snapshot"):
        screen.blit(draw_home_screen.snapshot, (0, 0))

    welcome_text = font.render("Welcome to the Car Simulator", True,
                               (255, 255, 255))
    welcome_text = pygame.transform.scale(welcome_text, (
    welcome_text.get_width() * 1.5, welcome_text.get_height() * 1.5))
    screen.blit(welcome_text, (WIDTH // 2 - welcome_text.get_width() // 2,
                               HEIGHT // 2 - welcome_text.get_height() // 2 - 150))

    title_text = font.render("Choose a Mode", True, (255, 255, 255))
    title_text = pygame.transform.scale(title_text, (
    title_text.get_width() * 1.22, title_text.get_height() * 1.2))
    screen.blit(title_text,
                (WIDTH // 2 - title_text.get_width() // 2, HEIGHT // 3 + 170))

    options = ["Joystick Mode", "Dynamic Mode", "Kinematics Mode"]
    help_option = "Help"

    for i, option in enumerate(options):
        pos = (
        WIDTH // 2 - font.size(option)[0] // 2, HEIGHT // 3 + 240 + i * 60)
        draw_text_with_glow(option, pos, selected == i)
    # Draw the Help button in the bottom right corner
    help_pos = (WIDTH - font.size(help_option)[0] - 20,
                HEIGHT - font.size(help_option)[1] - 20)
    draw_text_with_glow(help_option, help_pos, selected == len(options))

    pygame.display.flip()


# Game loop
while running:
    if in_home_screen:
        draw_home_screen(selected_option)

        if not hasattr(draw_home_screen, "snapshot"):
            draw_home_screen.snapshot = screen.copy()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    selected_option = (
                                                  selected_option - 1) % 4  # Update to 4 options
                elif event.key == pygame.K_DOWN:
                    selected_option = (
                                                  selected_option + 1) % 4  # Update to 4 options
                elif event.key == pygame.K_RETURN:
                    if selected_option == 3:  # Help option
                        mode = "help"
                    else:
                        mode = ["joystick", "dynamic", "kinematics"][
                            selected_option]
                    in_home_screen = False
                elif event.key == pygame.K_RIGHT and in_home_screen:
                    selected_option = 3  # Select the Help option
                elif event.key == pygame.K_LEFT and selected_option == 3:
                    selected_option = 0  # Select the first option
    else:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    in_home_screen = True
                    car_x, car_y = 50, HEIGHT // 2 + 23
                    car_angle = 0
                    car_speed = 0
                    state = [car_x, car_y, math.radians(car_angle), 0, 0, 0]
                    del draw_home_screen.snapshot
                elif event.key == pygame.K_s:
                    # Toggle the sensor state
                    sensor_active = not sensor_active

        keys = pygame.key.get_pressed()

        # Check for end of road (when car reaches the right edge of the current road)
        road_width = road.get_width()  # Get the width of the current road image
        if car_x >= road_width:  # Reached the right edge of the screen
            current_road += 1
            if current_road < len(road_images):
                road = pygame.image.load(road_images[current_road])
                if mode == "dynamic":
                    state[0] = 0  # Reset car position to the left edge
                car_x = 0  # Reset car position to the left edge
            else:
                print("End of map, congrats!")
                running = False

        # Draw the background (road)
        screen.blit(road, (0, 0))

        # Calculate the front of the car
        car_front_x = car_x
        car_front_y = car_y

        # Define the color of the road lines (e.g., white lines)
        LINE_COLORS = [
            (250, 253, 253),  # #fafdfd
            (206, 213, 205),  # #ced5cd
            (140, 136, 129),  # #8c8881
            (229, 230, 229),  # #e5e6e5
        ]

        # Sensor simulation (anchored at the front of the car)
        detected_lines = []  # To store the positions of detected lines

        for angle_offset in range(-SENSOR_ANGLE // 2, SENSOR_ANGLE // 2 + 1,
                                    4):  # Steps within the cone
                sensor_angle = - car_angle - math.radians(
                - angle_offset)  # Adjust for rotation
                for distance in range(1, SENSOR_RANGE,
                                        5):  # Incremental steps along the ray
                    sensor_x = int(
                        car_front_x + distance * math.cos(sensor_angle))
                    sensor_y = int(
                        car_front_y - distance * math.sin(sensor_angle))

                    # Ensure the sensor point is within screen bounds
                    if 0 <= sensor_x < WIDTH and 0 <= sensor_y < HEIGHT:
                        # Check the color of the pixel at the sensor point
                        pixel_color = road.get_at((sensor_x, sensor_y))[
                                        :3]  # Ignore alpha channel
                        if pixel_color in LINE_COLORS:
                            detected_lines.append((sensor_x,
                                                    sensor_y))  # Record detected line position
                            # Save the position and time to the file
                            current_time = pygame.time.get_ticks()   # Get current time in miliseconds
                            file.write(f"{sensor_x} \t {HEIGHT - sensor_y} \t {current_time:.2f}\n")
                            break  # Stop the ray once a line is detected
                    if sensor_active:
                        # Draw the sensor ray
                        pygame.draw.line(screen, (0, 255, 0),
                                            (car_front_x, car_front_y),
                                            (sensor_x, sensor_y), 1)
        # Draw detected lines (if any)
        if sensor_active:
            for line_pos in detected_lines:
                pygame.draw.circle(screen, (255, 0, 0), line_pos,
                                5)  # Red dots for detected lines

        
        if mode == "joystick":
            car_x, car_y = joystick_mode(keys, car_x, car_y)

        elif mode == "dynamic":
            # Call the function in the game loop
            state = dynamic_mode(keys, state)
            car_x, car_y, car_angle, v_u, phi, u_2 = state
            if display_unsafe_message:
                if unsafe_start_time and pygame.time.get_ticks() - unsafe_start_time > 2000:
                    display_unsafe_message = False
                    unsafe_start_time = None
                else:
                    text_surface = font.render("Unsafe! LTA intervention", True,
                                                (255, 0, 0))
                    screen.blit(text_surface, (50, 50))

        elif mode == "kinematics":
            # Call the function in the game loop
            car_x, car_y, car_angle, car_speed = kinematics_mode(keys, car_x,
                                                                 car_y,
                                                                 car_angle,
                                                                 car_speed)
            if display_unsafe_message:
                if unsafe_start_time and pygame.time.get_ticks() - unsafe_start_time > 2000:
                    display_unsafe_message = False
                    unsafe_start_time = None
                else:
                    text_surface = font.render("Unsafe! LTA intervention", True,
                                                (255, 0, 0))
                    screen.blit(text_surface, (50, 50))

                # Add the help mode handling
        elif mode == "help":
            screen.fill((0, 0, 0))  # Clear the screen
            y_offset = 50
            for line in help_text.split('\n'):
                text_surface = font.render(line, True, (255, 255, 255))
                screen.blit(text_surface, (50, y_offset))
                y_offset += 40
            pygame.display.flip()
            pygame.time.wait(3000)  # Display help for 5 seconds
            in_home_screen = True
            mode = None

            

        # Rotate and draw the car
        rotated_car = pygame.transform.rotate(car, -math.degrees(
            car_angle))  # Negative angle to match screen coordinates
        rect = rotated_car.get_rect(center=(car_x, car_y))
        screen.blit(rotated_car, rect.topleft)

        # Refresh screen
        pygame.display.flip()
        pygame.time.Clock().tick(60)

pygame.quit()
