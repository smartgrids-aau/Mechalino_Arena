#!/usr/bin/env python3

import copy
import rospy
from std_msgs.msg import Float32MultiArray
import socket
import threading
import path_generation  # Importiere den path_generator

# Globale Variablen für die Positionsdaten und das Roboterdictionary
robots_dict = {}  # Dictionary: ID -> {'Address': ('ip-address', port), 'Position': {'x': x, 'y': y, 'yaw': yaw}}

# Generiere die Route dynamisch mit den Werten für das Rechteck und die Unterteilung
x1, y1 = 0.1, -0.035  # Linke obere Ecke
x2, y2 = 1.98, -0.81  # Rechte untere Ecke
m, n = 3, 4  # Anzahl der Abschnitte
route = path_generation.generate_rectangle_path(x1, y1, x2, y2, m, n)

# Funktion zum Überprüfen, ob der Roboter das Ziel erreicht hat
def check_target_reached(position, target):
    x, y = position['x'], position['y']
    target_x, target_y = target
    if abs(x - target_x) <= 0.05 and abs(y - target_y) <= 0.05:
        return True
    return False

# Callback-Funktion für das Abonnieren der Positionsdaten
def callback(data):
    global robots_dict
    robot_id = int(data.data[3])  # Die ID des Roboters ist das 4. Element im Array
    x = round(data.data[0], 4)  # Runden auf 4 Nachkommastellen
    y = round(data.data[1], 4)  # Runden auf 4 Nachkommastellen
    yaw = round(data.data[2], 0)  # Runden auf ganze Zahl
    robots_dict.setdefault(robot_id, {}).setdefault('Position', {})
    robots_dict[robot_id]['Position'].update({'x': x, 'y': y, 'yaw': yaw})

# Funktion für den TCP-Server, der die Positionsdaten sendet
def tcp_server():
    host = '0.0.0.0'
    port = 5000

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print("TCP Server is listening on {}:{}".format(host, port))

    while True:
        client_socket, addr = server_socket.accept()
        print("Connection from:", addr)
        threading.Thread(target=handle_client, args=(client_socket, addr)).start()

def handle_client(client_socket: socket.socket, address: tuple):
    global robots_dict
    client_ip, port = address[0], address[1]
    current_target_index = 0
    try:
        while True:
            data = client_socket.recv(1024)
            if data:
                message = data.decode("utf-8")
                print(f"Received: {message} from {client_ip}:{port}")

                if message == "REGISTER":
                    while True:
                        print(f"New robot with IP {client_ip}:{port} is not registered. Sending SPIN command.")
                        client_socket.send("SPIN\n".encode())
                        moving_robot_id = determine_moving_robot_id()
                        if moving_robot_id is not None:
                            robots_dict.setdefault(moving_robot_id, {}).setdefault('Address', address)
                            response = f"REGISTER_COMPLETE {moving_robot_id}"
                            client_socket.send(response.encode())
                            print(f"Robot registered with ID {moving_robot_id} and IP {client_ip}:{port}.")
                            break
                        else:
                            print("Could not determine robot ID. Registration failed.")
                            rospy.sleep(1)

                elif message.startswith("REQUEST_LOCATION_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])
                        if robot_id in robots_dict:
                            position = robots_dict[robot_id].get('Position', {})
                            response = f"LOCATION_UPDATE x:{position.get('x', 0)};y:{position.get('y', 0)};yaw:{position.get('yaw', 0)} {robot_id}"
                            client_socket.send((response + "\n").encode())
                        else:
                            response = f"ERROR Robot ID {robot_id} not found. Sending REGISTER"
                            client_socket.send((f"REGISTER" + "\n").encode())
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_LOCATION_UPDATE"
                    print(f"Sent location update for robot {robot_id}: {response}")

                elif message.startswith("REQUEST_TARGET_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])
                        if robot_id in robots_dict:
                            position = robots_dict[robot_id].get('Position', {})
                            if check_target_reached(position, route[current_target_index]):
                                print(f"Robot {robot_id} has reached target {current_target_index + 1}.")
                                current_target_index += 1
                                if current_target_index >= len(route):
                                    print(f"Robot {robot_id} has completed the route. Sending STOP.")
                                    client_socket.send("STOP\n".encode())
                                    break
                            if current_target_index < len(route):
                                target_x, target_y = route[current_target_index]  # Keine yaw-Werte mehr
                                response = f"TARGET_UPDATE x:{target_x};y:{target_y} {robot_id}"
                                client_socket.send((response + "\n").encode())
                        else:
                            response = f"ERROR Robot ID {robot_id} not found"
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_TARGET_UPDATE"
                    print(f"Sent target update for robot {robot_id}: {response}")

                # Behandle REQUEST_PATH_UPDATE Nachricht
                elif message == "REQUEST_PATH_UPDATE":
                    try:
                        # Pfad als String formatieren: x0;y0;x1;y1;...;xn;yn
                        path_str = ";".join([f"{x}:{y}" for x, y in route])
                        response = f"PATH_UPDATE {path_str}"
                        client_socket.send((response + "\n").encode())
                        print(f"Sent path update: {response}")
                    except Exception as e:
                        print(f"Error in processing REQUEST_PATH_UPDATE: {e}")
                        response = "ERROR Unable to process REQUEST_PATH_UPDATE"
                        client_socket.send((response + "\n").encode())

    except Exception as e:
        print("Client disconnected:", e)
    finally:
        if robot_id is not None and robot_id in robots_dict:
            del robots_dict[robot_id]
            print(f"Robot with ID {robot_id} removed from robots_dict due to disconnection.")
        client_socket.close()

def determine_moving_robot_id(epsilon: int = 5):
    initial_robots = copy.deepcopy(robots_dict)
    print(f"Initial positions: {initial_robots}")
    rospy.sleep(4)
    
    if not initial_robots:
        print("No initial positions recorded.")
        return None
    
    for robot_id, current_robot_data in robots_dict.items():
        if "Address" in current_robot_data:
            initial_robots.pop(robot_id, None)
            continue
        initial_yaw = initial_robots.get(robot_id, {}).get('Position', {}).get('yaw', None)
        current_yaw = current_robot_data.get('Position', {}).get('yaw', None)
        
        if initial_yaw is not None and current_yaw is not None:
            yaw_difference = abs(current_yaw - initial_yaw)
            if yaw_difference > epsilon:
                print(f"Robot {robot_id} has moved.")
                return robot_id
    
    print("No significant movement detected.")
    return None

# Hauptfunktion, die den ROS-Node und den TCP-Server startet
if __name__ == '__main__':
    try:
        tcp_thread = threading.Thread(target=tcp_server)
        tcp_thread.start()
        
        rospy.init_node('robot_position_yaw_tcp', anonymous=True)

        for i in range(61):
            topic_name = f"/pos/mechalino_{i}"
            rospy.Subscriber(topic_name, Float32MultiArray, callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
