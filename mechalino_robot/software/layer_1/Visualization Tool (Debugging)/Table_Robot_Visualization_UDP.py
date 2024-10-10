import socket
import tkinter as tk
import math
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import time
import queue
import matplotlib

# Use TkAgg to ensure that matplotlib works well with Tkinter
matplotlib.use("TkAgg")

# UDP configuration
UDP_IP = "0.0.0.0"
UDP_PORT = 4210

# Table dimensions in meters (including the border)
TABLE_WIDTH = 1.8
TABLE_HEIGHT = 0.8
BORDER_THICKNESS = 0.012  # 1.2cm border on each side

# Usable space of the table (excluding the border)
USABLE_WIDTH = TABLE_WIDTH - 2 * BORDER_THICKNESS
USABLE_HEIGHT = TABLE_HEIGHT - 2 * BORDER_THICKNESS

# Robot circle properties
ROBOT_RADIUS = 0.06  # 12 cm diameter
TARGET_RADIUS = 0.02  # 4 cm diameter
SCALE_FACTOR = 500  # Scale factor for converting meters to pixels

STATES_MAP = {"0": "IDLE", "1": "ROTATING", "2": "MOVING", "3": "SPINNING"}

# Initialize GUI
root = tk.Tk()
root.title("Robot Path Tracker")

canvas_width = int(TABLE_WIDTH * SCALE_FACTOR)
canvas_height = int(TABLE_HEIGHT * SCALE_FACTOR)

canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
canvas.pack()

# Draw the outer rectangle (including the border) and fill it as the border
canvas.create_rectangle(
    0, 0,
    canvas_width, canvas_height,
    outline="white", fill="white", width=2  # "gray" is used to represent the filled border
)

# Draw the actual usable table (inner rectangle) without the border
table_top_left_x = BORDER_THICKNESS * SCALE_FACTOR
table_top_left_y = BORDER_THICKNESS * SCALE_FACTOR
table_bottom_right_x = table_top_left_x + (USABLE_WIDTH * SCALE_FACTOR)
table_bottom_right_y = table_top_left_y + (USABLE_HEIGHT * SCALE_FACTOR)

canvas.create_rectangle(
    table_top_left_x, table_top_left_y,
    table_bottom_right_x, table_bottom_right_y,
    outline="white", fill="black", width=2  # "white" fill for the usable table space
)

# Variables to store robot path and drawing elements
robot_path = {}
robot_circles = {}
robot_arrows = {}
robot_lines = {}
target_circles = {}  # Dictionary to store the target circles

# Thread-safe queue for handling data between threads
data_queue = queue.Queue()

# Data for plotting angle error and motor speeds
times = []
angle_errors = []
motor_speeds_l = []
motor_speeds_r = []
start_time = time.time()

current_target_index = 0


# Function to draw a star shape
def draw_star(x_center, y_center, radius, points=5, fill_color="yellow", outline_color="yellow"):
    angle_between_points = math.pi / points  # Angle between the star points
    star_points = []

    for i in range(points * 2):  # Create twice as many points, alternating inner/outer
        angle = i * angle_between_points
        if i % 2 == 0:  # Outer point
            r = radius
        else:  # Inner point
            r = radius / 2.5  # Smaller inner radius

        x = x_center + r * math.sin(angle)
        y = y_center - r * math.cos(angle)  # Subtract for proper orientation
        star_points.append((x, y))

    # Flatten the list of points
    star_points_flat = [coord for point in star_points for coord in point]

    # Draw the star
    canvas.create_polygon(star_points_flat, outline=outline_color, fill=fill_color, width=2)


# Function to draw the targets and their status
def draw_targets(x_coords, y_coords, current_index):
    for i in range(len(x_coords)):
        target_pixel_x = x_coords[i] * SCALE_FACTOR
        target_pixel_y = y_coords[i] * SCALE_FACTOR * -1  # Flip Y-coordinate

        if i < current_index:
            # Covered target: Green circle with no fill
            canvas.create_oval(
                target_pixel_x - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_x + TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y + TARGET_RADIUS * SCALE_FACTOR,
                outline="black", fill="black", width=2
            )
            canvas.create_oval(
                target_pixel_x - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_x + TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y + TARGET_RADIUS * SCALE_FACTOR,
                outline="green", fill="", width=2
            )
        elif i == current_index:
            # Current target: Yellow star
            canvas.create_oval(
                target_pixel_x - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_x + TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y + TARGET_RADIUS * SCALE_FACTOR,
                outline="black", fill="black", width=2
            )
            draw_star(
                target_pixel_x, target_pixel_y, TARGET_RADIUS * SCALE_FACTOR, points=5,
                fill_color="yellow", outline_color="yellow"
            )
        else:
            # Remaining target: Red circle with fill
            canvas.create_oval(
                target_pixel_x - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_x + TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y + TARGET_RADIUS * SCALE_FACTOR,
                outline="black", fill="black", width=2
            )
            canvas.create_oval(
                target_pixel_x - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y - TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_x + TARGET_RADIUS * SCALE_FACTOR,
                target_pixel_y + TARGET_RADIUS * SCALE_FACTOR,
                outline="red", fill="red", width=2
            )


# Function to draw the robot and path
def draw_robot_and_path(robot_id, x, y, yaw):
    # Convert meters to pixels (flip y-coordinate because GUI origin is top-left)
    pixel_x = x * SCALE_FACTOR
    pixel_y = (y * SCALE_FACTOR * -1)  # Adjusting for negative y-coordinates

    # Draw or update the robot circle
    if robot_id not in robot_circles:
        # Create new robot representation if it doesn't exist
        robot_circles[robot_id] = canvas.create_oval(
            pixel_x - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_x + ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y + ROBOT_RADIUS * SCALE_FACTOR,
            outline="blue",
            width=2
        )
    else:
        # Update existing robot circle position
        canvas.coords(
            robot_circles[robot_id],
            pixel_x - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_x + ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y + ROBOT_RADIUS * SCALE_FACTOR
        )

    # Draw or update the yaw arrow
    arrow_length = ROBOT_RADIUS * SCALE_FACTOR * 1
    arrow_x = pixel_x + arrow_length * math.sin(math.radians(yaw))
    arrow_y = pixel_y - arrow_length * math.cos(math.radians(yaw))

    if robot_id not in robot_arrows:
        # Create new arrow if it doesn't exist
        robot_arrows[robot_id] = canvas.create_line(
            pixel_x, pixel_y, arrow_x, arrow_y, fill="red", width=2, arrow=tk.LAST
        )
    else:
        # Update existing arrow position
        canvas.coords(robot_arrows[robot_id], pixel_x, pixel_y, arrow_x, arrow_y)

    # Store the current position in the robot's path
    if robot_id not in robot_path:
        robot_path[robot_id] = []
    robot_path[robot_id].append((pixel_x, pixel_y))

    # Draw or update the path lines
    if len(robot_path[robot_id]) > 1:
        if robot_id not in robot_lines:
            # Create the initial line
            robot_lines[robot_id] = canvas.create_line(
                *robot_path[robot_id], fill="green", width=2
            )
        else:
            # Update the existing line with new points
            canvas.coords(robot_lines[robot_id], *sum(robot_path[robot_id], ()))


# Function to parse the incoming message and extract robot data
def handle_message(message):
    global current_target_index
    try:
        print(message)
        parts = message.split(';')
        if len(parts) < 3:  # Check for complete data
            print(f"Incomplete message received: {message}")
            return

        if len(parts) < 4:
            x_coords = list(map(float, parts[0].split(':')))
            y_coords = list(map(float, parts[1].split(':')))
            amount_coords = int(parts[2])
            # Draw targets on the table based on their status
            draw_targets(x_coords, y_coords, current_target_index)
            return

        robot_id = parts[0]
        current_x = float(parts[1])
        current_y = float(parts[2])
        current_yaw = float(parts[3])
        state = STATES_MAP.get(parts[4].split(':')[1], "UNKNOWN")
        current_target_index = int(parts[5].split(':')[1])
        motor_l = int(parts[6].split(':')[1])
        motor_r = int(parts[7].split(':')[1])
        angle_error = float(parts[8].split(':')[1])
        calculated_pid = float(parts[9].split(':')[1])
        target_distance = float(parts[10].split(':')[1])

        # Draw/update robot path and yaw
        draw_robot_and_path(robot_id, current_x, current_y, current_yaw)

        # Store data for plotting
        elapsed_time = time.time() - start_time
        times.append(elapsed_time)
        angle_errors.append(angle_error)
        motor_speeds_l.append(motor_l)
        motor_speeds_r.append(motor_r)

    except Exception as e:
        print(f"Error parsing message: {e}")


# Function to listen to UDP broadcast in a separate thread
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, _ = sock.recvfrom(1024)
        message = data.decode('utf-8')
        data_queue.put(message)  # Put data in the queue instead of processing it directly


# Function to process data from the queue and update the GUI (Tkinter-safe)
def process_data():
    while not data_queue.empty():
        message = data_queue.get()
        handle_message(message)

    # Call this function again after a short delay to keep checking the queue
    root.after(100, process_data)


# Plotting Angle Error vs Motor Speeds over Time
fig, ax = plt.subplots()
ax.set_title('Angle Error and Motor Speeds Over Time')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle Error / Motor Speed')

line_angle, = ax.plot([], [], label='Angle Error', color='blue')
line_motor_l, = ax.plot([], [], label='Motor L Speed', color='green')
line_motor_r, = ax.plot([], [], label='Motor R Speed', color='red')
ax.legend()


def update_plot(frame):
    if len(times) == 0:  # Skip if no data yet
        return line_angle, line_motor_l, line_motor_r

    line_angle.set_data(times, angle_errors)
    line_motor_l.set_data(times, motor_speeds_l)
    line_motor_r.set_data(times, motor_speeds_r)

    # Set axis limits dynamically
    ax.set_xlim(0, max(times) if times else 1)
    ax.set_ylim(min(min(angle_errors + motor_speeds_l + motor_speeds_r), -50),
                max(max(angle_errors + motor_speeds_l + motor_speeds_r), 50))
    return line_angle, line_motor_l, line_motor_r


# Use FuncAnimation to update the plot in real-time
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

# Start the UDP listener in a background thread
listener_thread = threading.Thread(target=udp_listener, daemon=True)
listener_thread.start()

# Start the data processing loop
root.after(100, process_data)

# Show the Tkinter GUI and the plot
plt.show()
root.mainloop()
