#!/usr/bin/env python3

import copy
import rospy
from std_msgs.msg import Float32MultiArray
import socket
import threading
import path_generation


robots_dict = {}  # Dictionary: ID -> {'Address': ('ip-address', port), 'Position': {'x': x, 'y': y, 'yaw': yaw}}


# Path 1
x1, y1 = 0.125, -0.56
x2, y2 = 0.687, -0.124
m1, n1 = 2, 2
path1 = path_generation.generate_rectangle_path(x1, y1, x2, y2, m1, n1)

# Path 2
x3, y3 = 1.58, -0.127
x4, y4 = 1.029, -0.54
m2, n2 = 2, 2
path2 = path_generation.generate_rectangle_path(x3, y3, x4, y4, m2, n2)

# Function to check if the robot has reached a target location
def check_target_reached(position, target):
    x, y = position['x'], position['y']
    target_x, target_y = target
    # Check if the robot is within 0.05 units of the target location
    if abs(x - target_x) <= 0.05 and abs(y - target_y) <= 0.05:
        return True
    return False

def callback(data):
    global robots_dict
    # Extract the robot's ID from the 4th element of the incoming data array
    robot_id = int(data.data[3])
    # Extract the x, y, and yaw values and round them appropriately
    x = round(data.data[0], 4)
    y = round(data.data[1], 4)
    yaw = round(data.data[2], 0)

    robots_dict.setdefault(robot_id, {}).setdefault('Position', {})
    robots_dict[robot_id]['Position'].update({'x': x, 'y': y, 'yaw': yaw})

# Function to handle a TCP server that sends position data to connected clients (robots)
def tcp_server():
    host = '0.0.0.0'
    port = 5000

    # Create a TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)  # Allow up to 5 connections to queue

    print(f"TCP Server is listening on {host}:{port}")

    # Continuously accept new client connections in a loop
    while True:
        client_socket, addr = server_socket.accept()
        print(f"Connection from: {addr}")
        threading.Thread(target=handle_client, args=(client_socket, addr)).start()

# Function to handle communication with each client
def handle_client(client_socket: socket.socket, address: tuple):
    global robots_dict
    client_ip, port = address[0], address[1]
    current_target_index = 0
    robot_id = None

    try:
        # Loop to continuously communicate with the robot
        while True:
            data = client_socket.recv(1024)
            if data:
                message = data.decode("utf-8")

                print(f"Received from robot {client_ip}:{port}: {message}")

                # Handle robot registration requests
                if message.startswith("REGISTER"):
                    split_msg = message.split()

                    # If the message contains "REGISTER" followed by an ID:
                    if len(split_msg) == 2:
                        robot_id = int(split_msg[1])
                        if robot_id in robots_dict:
                            # If the robot is already registered, acknowledge with "REGISTERED"
                            response = f"REGISTERED {robot_id}\n"
                            print(f"Sending to robot {client_ip}:{port}: {response}")
                            client_socket.send(response.encode())
                        else:
                            # Otherwise, add the robot to the dictionary and respond with "REGISTERED"
                            robots_dict[robot_id] = {'Address': address, 'Position': {'x': 0, 'y': 0, 'yaw': 0}}
                            response = f"REGISTERED {robot_id}\n"
                            print(f"Sending to robot {client_ip}:{port}: {response}")
                            client_socket.send(response.encode())

                    # If no ID is provided, attempt to assign one
                    else:
                        while True:
                            print(f"New robot with IP {client_ip}:{port} is not registered. Sending SPIN command.")
                            client_socket.send("SPIN\n".encode())  # Send "SPIN" command to rotate the robot
                            moving_robot_id = determine_moving_robot_id()  # Determine which robot is moving
                            if moving_robot_id is not None:
                                # Assign the moving robot to the current connection
                                robots_dict.setdefault(moving_robot_id, {}).setdefault('Address', address)
                                response = f"REGISTER_COMPLETE {moving_robot_id}\n"
                                print(f"Sending to robot {client_ip}:{port}: {response}")
                                client_socket.send(response.encode())
                                robot_id = moving_robot_id  # Set robot_id to the identified moving robot
                                break
                            else:
                                print("Could not determine robot ID. Registration failed.")
                                rospy.sleep(1)

                # Handle location update requests from the robot
                elif message.startswith("REQUEST_LOCATION_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])
                        if robot_id in robots_dict:
                            # Fetch and send the robot's current position
                            position = robots_dict[robot_id].get('Position', {})
                            response = f"LOCATION_UPDATE x:{position.get('x', 0)};y:{position.get('y', 0)};yaw:{position.get('yaw', 0)} {robot_id}"
                            print(f"Sending to robot {client_ip}:{port}: {response}")
                            client_socket.send((response + "\n").encode())
                        else:
                            # If robot ID is not found, prompt it to register
                            response = f"ERROR Robot ID {robot_id} not found. Sending REGISTER"
                            print(f"Sending to robot {client_ip}:{port}: {response}")
                            client_socket.send("REGISTER\n".encode())
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_LOCATION_UPDATE"
                    print(f"Sent location update for robot {robot_id}: {response}")

                # Handle target update requests from the robot
                elif message.startswith("REQUEST_TARGET_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])

                        # Assign paths based on the robot's ID
                        if robot_id == 20:
                            chosen_path = path1
                        elif robot_id == 15:
                            chosen_path = path2
                        else:
                            chosen_path = path1

                        if robot_id in robots_dict:
                            position = robots_dict[robot_id].get('Position', {})

                            # Check if the robot has reached its current target
                            if check_target_reached(position, chosen_path[current_target_index]):
                                print(f"Robot {robot_id} has reached target {current_target_index + 1}.")
                                current_target_index += 1  # Move to the next target
                                if current_target_index >= len(chosen_path):
                                    # If all targets are reached, send a "STOP" command
                                    print(f"Robot {robot_id} has completed the route. Sending STOP.")
                                    client_socket.send("STOP\n".encode())
                                    break

                            # Send the next target coordinates if the route is not yet complete
                            if current_target_index < len(chosen_path):
                                target_x, target_y = chosen_path[current_target_index]
                                response = f"TARGET_UPDATE x:{target_x};y:{target_y} {robot_id}"
                                print(f"Sending to robot {client_ip}:{port}: {response}")
                                client_socket.send((response + "\n").encode())
                        else:
                            response = f"ERROR Robot ID {robot_id} not found"
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_TARGET_UPDATE"
                    print(f"Sent target update for robot {robot_id}: {response}")

                # Handle path update requests from the robot
                elif message.startswith("REQUEST_PATH_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])

                        # Assign paths based on the robot's ID
                        if robot_id == 20:
                            chosen_path = path1
                        elif robot_id == 15:
                            chosen_path = path2
                        else:
                            chosen_path = path1

                        # Format the path into a string (x and y values) for sending to the robot
                        x_values = ":".join([str(round(x, 2)) for x, y in chosen_path])
                        y_values = ":".join([str(round(y, 2)) for x, y in chosen_path])
                        value_count = len(chosen_path) * 2  # Total number of path points

                        path_str = f"x:{x_values};y:{y_values};{value_count} {robot_id}"
                        response = f"PATH_UPDATE {path_str}"
                        print(f"Sending to robot {client_ip}:{port}: {response}")
                        client_socket.send((response + "\n").encode())
                    except Exception as e:
                        print(f"Error in processing REQUEST_PATH_UPDATE: {e}")
                        response = "ERROR Unable to process REQUEST_PATH_UPDATE"
                        client_socket.send((response + "\n").encode())

    except Exception as e:
        print("Client disconnected:", e)
    finally:
        # If the robot was registered, remove it from the dictionary upon disconnection
        if robot_id is not None and robot_id in robots_dict:
            del robots_dict[robot_id]
            print(f"Robot with ID {robot_id} removed from robots_dict due to disconnection.")
        client_socket.close()

# Function to determine which robot is moving based on a change in yaw
def determine_moving_robot_id(epsilon: int = 5):
    initial_robots = copy.deepcopy(robots_dict)  # Make a copy of the current robots' state
    print(f"Initial positions: {initial_robots}")
    rospy.sleep(4)  # Sleep for 4 seconds to allow for movement

    if not initial_robots:
        print("No initial positions recorded.")
        return None

    for robot_id, current_robot_data in robots_dict.items():
        if "Address" in current_robot_data:
            initial_robots.pop(robot_id, None)
            continue

        # Compare the initial and current yaw of each robot to detect movement
        initial_yaw = initial_robots.get(robot_id, {}).get('Position', {}).get('yaw', None)
        current_yaw = current_robot_data.get('Position', {}).get('yaw', None)

        # If the yaw difference exceeds the epsilon threshold, the robot is considered to have moved
        if initial_yaw is not None and current_yaw is not None:
            yaw_difference = abs(current_yaw - initial_yaw)
            if yaw_difference > epsilon:
                print(f"Robot {robot_id} has moved.")
                return robot_id

    print("No significant movement detected.")
    return None

if __name__ == '__main__':
    try:
        tcp_thread = threading.Thread(target=tcp_server)
        tcp_thread.start()

        rospy.init_node('robot_position_yaw_tcp', anonymous=True)

        mechalino_ids = rospy.get_param('/mechalino_ids', [15, 60, 20])

        for robot_id in mechalino_ids:
            topic_name = f"/pos/mechalino_{robot_id}"
            rospy.Subscriber(topic_name, Float32MultiArray, callback)


        rospy.spin()
    except rospy.ROSInterruptException:
        pass
