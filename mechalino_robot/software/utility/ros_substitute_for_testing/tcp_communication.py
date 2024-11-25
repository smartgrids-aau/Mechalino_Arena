import socket
import time

# Define the IP and port for the TCP server (replace with the IP address of your ESP8266)
HOST = '192.168.4.1'  # Replace with the ESP8266 IP address
PORT = 5000  # Port for TCP communication

# Simulate a basic ROS handshake and send location and target updates
def simulate_ros_server():
    requested:bool = False
    # Create a TCP/IP socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind((HOST, PORT))
        s.listen()
        print(f"Server listening on {HOST}:{PORT}...")
        
        # Wait for the robot to connect
        conn, addr = s.accept()
        with conn:
            print(f"Connected by {addr}")
            
            # Handshake: Register the robot
            data = conn.recv(1024).decode('utf-8')
            if data.strip() == "REGISTER":
                print("Received REGISTER command")
                conn.sendall(b"SPIN\n")  # Simulate sending the "SPIN" command
                
                time.sleep(2)  # Wait for the robot to spin

                # Simulate completing the registration and assigning the robot an ID
                robot_id = "23"
                conn.sendall(f"REGISTER_COMPLETE {robot_id}\n".encode('utf-8'))
                print(f"Sent REGISTER_COMPLETE {robot_id}")
            
            # Handle requests for location and target updates
            while True:
                # Receive request from the ESP8266
                data = conn.recv(1024).decode('utf-8').strip()
                if not data:
                    break  # If no data, end connection

                print(f"Received request: {data}")

                # Handle location update request
                if data.startswith("REQUEST_LOCATION_UPDATE"):
                    requested_id = data.split()[-1]
                    if requested_id == robot_id:
                        if not requested:
                            requested = True
                            location_update = "LOCATION_UPDATE x:0.25;y:0.35;yaw:90 23\n"
                            conn.sendall(location_update.encode('utf-8'))
                            print(f"Sent LOCATION_UPDATE: {location_update.strip()}")

                # Handle target update request
                elif data.startswith("REQUEST_TARGET_UPDATE"):
                    requested_id = data.split()[-1]
                    if requested_id == robot_id:
                        target_update = "TARGET_UPDATE x:0.75;y:0.85;yaw:180 23\n"
                        conn.sendall(target_update.encode('utf-8'))
                        print(f"Sent TARGET_UPDATE: {target_update.strip()}")
                
                # Other unrecognized commands
                else:
                    print(f"Unrecognized command: {data}")
                time.sleep(1)  # Optional: Slow down the loop to simulate processing time


if __name__ == "__main__":
    simulate_ros_server()