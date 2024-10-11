import math
import queue
import random
import socket
import threading
import time
import tkinter as tk

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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
TARGET_RADIUS = 0.03  # 6 cm diameter
SCALE_FACTOR = 500  # Scale factor for converting meters to pixels

STATES_MAP = {"0": "IDLE", "1": "ROTATING", "2": "MOVING", "3": "SPINNING"}
STATES_ESP_MAP = {"0": "CONNECTING", "1": "REGISTERING", "2": "SPINNING", "3": "LOCATION_REQUEST", "4": "PATH_REQUEST",
                  "5": "END"}

# Time threshold to remove robots (20 seconds)
TIMEOUT_THRESHOLD = 20

# Initialize GUI
root = tk.Tk()
root.title("Robot Path Tracker")

canvas_width = int(TABLE_WIDTH * SCALE_FACTOR)
canvas_height = int(TABLE_HEIGHT * SCALE_FACTOR)

canvas = tk.Canvas(root, width=canvas_width, height=canvas_height)
canvas.pack()

# Draw the actual usable table (inner rectangle) without the border
table_top_left_x = BORDER_THICKNESS * SCALE_FACTOR
table_top_left_y = BORDER_THICKNESS * SCALE_FACTOR
table_bottom_right_x = table_top_left_x + (USABLE_WIDTH * SCALE_FACTOR)
table_bottom_right_y = table_top_left_y + (USABLE_HEIGHT * SCALE_FACTOR)

# Dictionary to store robots' information keyed by IP address
robots = {}

# Thread-safe queue for handling data between threads
data_queue = queue.Queue()

# Colors for robots
ROBOT_COLORS = ["red", "blue", "green", "purple", "orange", "cyan", "magenta", "yellow"]

# Data for plotting
start_time = time.time()


def canvas_redraw():
    canvas.create_rectangle(
        0, 0,
        canvas_width, canvas_height,
        outline="white", fill="white", width=2  # "gray" is used to represent the filled border
    )
    canvas.create_rectangle(
        table_top_left_x, table_top_left_y,
        table_bottom_right_x, table_bottom_right_y,
        outline="white", fill="black", width=2  # "white" fill for the usable table space
    )


canvas_redraw()


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
def draw_targets(ip_address):
    robot = robots.get(ip_address, {})
    current_index, x_coords, y_coords, _ = robot.get("targets", {}).values()
    robot_color = robot.get("drawings", {}).get("robot_color", "blue")

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
                outline=robot_color, fill="", width=2
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
                fill_color=robot_color, outline_color="yellow"
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
                outline=robot_color, fill=robot_color, width=2
            )


# Function to draw the robot and path
def draw_robot_and_path(ip_address):
    robot = robots.get(ip_address, {})
    x, y, yaw = robot.get("location", {}).values()

    # Convert meters to pixels (flip y-coordinate because GUI origin is top-left)
    pixel_x = x * SCALE_FACTOR
    pixel_y = (y * SCALE_FACTOR * -1)  # Adjusting for negative y-coordinates

    robot_color = robot.get("drawings", {}).get("robot_color", "blue")

    # Draw or update the robot circle
    if 'robot_circle' not in robot.get('drawings', {}):
        # Create new robot representation if it doesn't exist
        robot.setdefault('drawings', {})['robot_circle'] = canvas.create_oval(
            pixel_x - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_x + ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y + ROBOT_RADIUS * SCALE_FACTOR,
            outline=robot_color, width=2
        )
    else:
        # Update existing robot circle position
        canvas.coords(
            robot.get('drawings', {}).get('robot_circle'),
            pixel_x - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y - ROBOT_RADIUS * SCALE_FACTOR,
            pixel_x + ROBOT_RADIUS * SCALE_FACTOR,
            pixel_y + ROBOT_RADIUS * SCALE_FACTOR
        )

    # Draw or update the yaw arrow
    arrow_length = ROBOT_RADIUS * SCALE_FACTOR * 1
    arrow_x = pixel_x + arrow_length * math.sin(math.radians(yaw))
    arrow_y = pixel_y - arrow_length * math.cos(math.radians(yaw))

    if 'robot_arrow' not in robot.get('drawings', {}):
        # Create new arrow if it doesn't exist
        robot.setdefault('drawings', {})['robot_arrow'] = canvas.create_line(
            pixel_x, pixel_y, arrow_x, arrow_y, fill=robot_color, width=2, arrow=tk.LAST
        )
    else:
        # Update existing arrow position
        canvas.coords(
            robot.get('drawings', {}).get('robot_arrow'),
            pixel_x, pixel_y, arrow_x, arrow_y
        )

    # Store the current position in the robot's path
    if 'path_lines' not in robot.get('drawings', {}):
        robot.setdefault('drawings', {})['path_lines'] = []

    # append the current position to the path
    # Store the path points in 'path_lines' and append the new point
    robot.setdefault('drawings', {}).setdefault('path_lines', []).append((pixel_x, pixel_y))

    # Draw or update the path lines
    if len(robot['drawings']['path_lines']) > 1:
        if 'path_line_obj' not in robot['drawings']:
            # Create the initial line object on the canvas
            robot['drawings']['path_line_obj'] = canvas.create_line(
                *sum(robot['drawings']['path_lines'], ()), fill=robot_color, width=2
            )
        else:
            # Update the existing line object with the new set of points
            canvas.coords(robot['drawings']['path_line_obj'], *sum(robot['drawings']['path_lines'], ()))


# Function to parse the incoming message and extract robot data
def handle_message(message, ip_address):
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
            robots.setdefault(ip_address, {}).setdefault("targets", {}).update(
                {"x_coords": x_coords, "y_coords": y_coords, "amount_coords": amount_coords})
            draw_targets(ip_address)
            return

        robot_id = parts[0]
        #esp_state = STATES_ESP_MAP.get(parts[1], "UNKNOWN")
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

        robots.setdefault(ip_address, {}).update(
            {"robot_id": robot_id, "states": {"stm_state": state},
             "location": {"current_x": current_x, "current_y": current_y, "current_yaw": current_yaw}})

        # Update only the necessary fields in the "targets" key while keeping other target-related data
        robots.setdefault(ip_address, {}).setdefault("targets", {}).update({
            "current_target_index": current_target_index
        })

        # Use setdefault to preserve existing "drawings" entries and only update "circle_color" and "last_update"
        robots.setdefault(ip_address, {}).setdefault("drawings", {}).update({
            "circle_color": robots[ip_address]["drawings"].get("circle_color", random.choice(ROBOT_COLORS)),
            "last_update": time.time()
        })

        # for motor_l, motor_r, angle_errors, elapsed_times and calculated_pids we want to add the new ones to the
        # list to plot them
        # Ensure motor_speeds, calculations, and elapsed_times are present
        robots.setdefault(ip_address, {}).setdefault("motor_speeds", {"motorL": [], "motorR": []})
        robots.setdefault(ip_address, {}).setdefault("calculations",
                                                     {"distance_to_target": target_distance, "angle_errors": [],
                                                      "calculated_pids": []})
        robots.setdefault(ip_address, {}).setdefault("drawings", {}).setdefault("elapsed_times", [])

        elapsed_time = time.time() - start_time
        # add the new values to the lists
        robots.setdefault(ip_address, {}).get("motor_speeds", {}).get("motorL").append(motor_l)
        robots.setdefault(ip_address, {}).get("motor_speeds", {}).get("motorR").append(motor_r)
        robots.setdefault(ip_address, {}).get("calculations", {}).get("angle_errors").append(angle_error)
        robots.setdefault(ip_address, {}).get("calculations", {}).get("calculated_pids").append(calculated_pid)
        robots.setdefault(ip_address, {}).get("drawings", {}).get("elapsed_times").append(elapsed_time)

        #print(robots)

        # Draw/update robot path and yaw
        draw_robot_and_path(ip_address)

    except Exception as e:
        print(f"Error parsing message: {e}")


# Function to remove outdated robots
def remove_old_robots():
    current_time = time.time()
    to_remove = [ip for ip, robot in robots.items() if
                 current_time - robot.get("drawings", {}).get("last_update", 0) > TIMEOUT_THRESHOLD]

    for ip in to_remove:
        """print(f"Removing robot with IP: {ip}")
        # Remove robot form the dictionary and its drawings from the canvas
        robots.pop(ip)
        canvas_redraw()
        # refresh the plotlib
        plt.draw()"""
    # make the canvas empty
    # Call this function again after a delay to continuously check for outdated robots
    root.after(1000, remove_old_robots)


# Function to listen to UDP broadcast in a separate thread
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(1024)
        message = data.decode('utf-8')
        ip_address = addr[0]
        data_queue.put((message, ip_address))  # Put data in the queue instead of processing it directly


# Function to process data from the queue and update the GUI (Tkinter-safe)
def process_data():
    while not data_queue.empty():
        message, ip_address = data_queue.get()
        handle_message(message, ip_address)

    # Call this function again after a short delay to keep checking the queue
    root.after(100, process_data)


# Plotting Angle Error vs Motor Speeds over Time
fig, ax = plt.subplots()
ax.set_title('Angle Error and Motor Speeds Over Time')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Angle Error / Motor Speed')

line_angle, = ax.plot([], [], label='Angle Error', color='blue')
line_pid, = ax.plot([], [], label='Calculated PID', color='purple')
line_motor_l, = ax.plot([], [], label='Motor L Speed', color='green')
line_motor_r, = ax.plot([], [], label='Motor R Speed', color='red')
ax.legend()


def update_plot(frame):
    for ip_address, robot in robots.items():
        if "elapsed_times" not in robot.get("drawings", {}) or len(robot.get("drawings", {}).get("elapsed_times")) == 0:
            continue  # skip this robot if no data to plot

        # Update plot lines for this robot
        elapsed_times = robot.get("drawings", {}).get("elapsed_times")
        angle_errors = robot.get("calculations", {}).get("angle_errors", [])
        calculated_pids = robot.get("calculations", {}).get("calculated_pids", [])
        motor_speed_l = robot.get("motor_speeds", {}).get("motorL", [])
        motor_speed_r = robot.get("motor_speeds", {}).get("motorR", [])

        # Plot angle error, pid value and motor speeds over time
        line_angle.set_data(elapsed_times, angle_errors)
        line_pid.set_data(elapsed_times, calculated_pids)
        line_motor_l.set_data(elapsed_times, motor_speed_l)
        line_motor_r.set_data(elapsed_times, motor_speed_r)

        # Adjust axis limits dynamically
        ax.set_xlim(0, 150)
        y_min = min(min(angle_errors), min(motor_speed_l), min(motor_speed_r), -50)
        y_max = max(max(angle_errors), max(motor_speed_l), max(motor_speed_r), 50)
        ax.set_ylim(y_min, y_max)

        # Scale y-axis logarithmically as angle_error is much smaller than motor speeds
        ax.set_yscale('symlog')

    return line_angle, line_motor_l, line_motor_r


# Use FuncAnimation to update the plot in real-time
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

# Start the UDP listener in a background thread
listener_thread = threading.Thread(target=udp_listener, daemon=True)
listener_thread.start()

# Start the data processing loop and the robot removal loop
root.after(100, process_data)
root.after(1000, remove_old_robots)

# Show the Tkinter GUI and the plot
plt.show()
root.mainloop()
