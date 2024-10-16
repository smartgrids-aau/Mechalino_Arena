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
# Generiere zwei verschiedene Routen für unterschiedliche Roboter
x1, y1 = 0.125, -0.56  # Linke obere Ecke für Path 1
x2, y2 = 0.687, -0.124    # Rechte untere Ecke für Path 1
m1, n1 = 2, 2          # Anzahl der Abschnitte für Path 1
path1 = path_generation.generate_rectangle_path(x1, y1, x2, y2, m1, n1)

x3, y3 = 1.58, -0.127     # Linke obere Ecke für Path 2
x4, y4 = 1.029, -0.54     # Rechte untere Ecke für Path 2
m2, n2 = 2, 2          # Anzahl der Abschnitte für Path 2
path2 = path_generation.generate_rectangle_path(x3, y3, x4, y4, m2, n2)

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

    # Ausgabe der empfangenen Positionsdaten
    #print(f"Received position update for robot {robot_id}: x={x}, y={y}, yaw={yaw}")

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
    robot_id = None  # Sicherstellen, dass robot_id initial None ist
    try:
        while True:
            data = client_socket.recv(1024)
            if data:
                message = data.decode("utf-8")

                # Gib die empfangene Nachricht in der Konsole aus
                print(f"Received from robot {client_ip}:{port}: {message}")

                # Überprüfe, ob der Roboter mit einer ID registrieren möchte
                if message.startswith("REGISTER"):
                    split_msg = message.split()
                    
                    # Behandlung von "REGISTER ID"
                    if len(split_msg) == 2:
                        robot_id = int(split_msg[1])
                        if robot_id in robots_dict:
                            response = f"REGISTERED {robot_id}\n"
                            print(f"Sending to robot {client_ip}:{port}: {response}")  # Ausgabe der gesendeten Nachricht
                            client_socket.send(response.encode())
                        else:
                            robots_dict[robot_id] = {'Address': address, 'Position': {'x': 0, 'y': 0, 'yaw': 0}}
                            response = f"REGISTERED {robot_id}\n"
                            print(f"Sending to robot {client_ip}:{port}: {response}")  # Ausgabe der gesendeten Nachricht
                            client_socket.send(response.encode())

                    # Behandlung von "REGISTER" (ohne ID)
                    else:
                        while True:
                            print(f"New robot with IP {client_ip}:{port} is not registered. Sending SPIN command.")
                            client_socket.send("SPIN\n".encode())
                            moving_robot_id = determine_moving_robot_id()
                            if moving_robot_id is not None:
                                robots_dict.setdefault(moving_robot_id, {}).setdefault('Address', address)
                                response = f"REGISTER_COMPLETE {moving_robot_id}\n"
                                print(f"Sending to robot {client_ip}:{port}: {response}")  # Ausgabe der gesendeten Nachricht
                                client_socket.send(response.encode())
                                robot_id = moving_robot_id  # Setze robot_id auf das sich bewegende Robot
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
                            print(f"Sending to robot {client_ip}:{port}: {response}")  # Ausgabe der gesendeten Nachricht
                            client_socket.send((response + "\n").encode())
                        else:
                            response = f"ERROR Robot ID {robot_id} not found. Sending REGISTER"
                            print(f"Sending to robot {client_ip}:{port}: {response}")  # Ausgabe der gesendeten Nachricht
                            client_socket.send((f"REGISTER" + "\n").encode())
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_LOCATION_UPDATE"
                    print(f"Sent location update for robot {robot_id}: {response}")

                elif message.startswith("REQUEST_TARGET_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])

                        # Unterschiedliche Pfade basierend auf der Roboter-ID
                        if robot_id == 20:
                            chosen_path = path1
                        elif robot_id == 15:
                            chosen_path = path2
                        else:
                            chosen_path = path1  # Standardpfad, falls keine Übereinstimmung

                        if robot_id in robots_dict:
                            position = robots_dict[robot_id].get('Position', {})

                            # Überprüfe, ob das Ziel erreicht wurde
                            if check_target_reached(position, chosen_path[current_target_index]):
                                print(f"Robot {robot_id} has reached target {current_target_index + 1}.")
                                current_target_index += 1
                                if current_target_index >= len(chosen_path):
                                    print(f"Robot {robot_id} has completed the route. Sending STOP.")
                                    client_socket.send("STOP\n".encode())
                                    break

                            # Sende das nächste Ziel
                            if current_target_index < len(chosen_path):
                                target_x, target_y = chosen_path[current_target_index]  # Keine yaw-Werte mehr
                                response = f"TARGET_UPDATE x:{target_x};y:{target_y} {robot_id}"
                                print(f"Sending to robot {client_ip}:{port}: {response}")
                                client_socket.send((response + "\n").encode())
                        else:
                            response = f"ERROR Robot ID {robot_id} not found"
                    except ValueError:
                        response = "ERROR Invalid format for REQUEST_TARGET_UPDATE"
                    print(f"Sent target update for robot {robot_id}: {response}")

                # Die Pfade für die Roboter zuweisen, basierend auf der ID
                elif message.startswith("REQUEST_PATH_UPDATE"):
                    try:
                        split_msg = message.split()
                        robot_id = int(split_msg[1])

                        # Unterschiedliche Pfade basierend auf der Roboter-ID
                        if robot_id == 20:
                            chosen_path = path1
                        elif robot_id == 15:
                            chosen_path = path2
                        else:
                            chosen_path = path1  # Standardpfad, falls keine Übereinstimmung

                        x_values = ":".join([str(round(x, 2)) for x, y in chosen_path])
                        y_values = ":".join([str(round(y, 2)) for x, y in chosen_path])

                        value_count = len(chosen_path) * 2

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
