"""
This program captures images from a camera and serves them to clients over a TCP socket. The script runs two main functions: 
one to capture images from the camera and another to serve these images to clients. It uses threading to handle concurrent execution.
"""

import cv2  # OpenCV library for image capturing and processing
import socket  # Library for network communication
import threading  # Library for handling concurrent execution

# Global variable to hold the latest captured image
global_image = None
image_lock = threading.Lock() # Lock to synchronize access to the global_image variable

# Function to read images from the camera and update global_image
def capture_images():
    """
    Captures images from the camera and updates the global_image variable.
    """
    global global_image
    # Open the camera
    camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not camera.isOpened():
        print("Failed to connect to the camera")
        exit()

    # Set camera parameters
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    # Continuously capture frames from the camera
    while True:
        ret, frame = camera.read()
        # Update the global_image variable with the latest frame
        with image_lock:
            global_image = frame

# Function to serve images to clients over TCP socket
def serve_images(host, port):
    """
    Serves the captured images to clients over a TCP socket.

    Args:
        host (str): The IP address of the server.
        port (int): The port number of the server.
    """
    global global_image
    # Create a TCP socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")
    
    images_sent = 0
    while True:
        # Accept a new client connection
        client_socket, addr = server_socket.accept()
        try:
            # Acquire the lock and get the latest image as bytes
            with image_lock:
                frame_bytes = global_image.tobytes()
            frame_size = len(frame_bytes)
            # Send the number of bytes first
            client_socket.sendall(frame_size.to_bytes(8, byteorder='big'))
            bytes_sent = 0
            # Send the image data in chunks to the client
            while bytes_sent < frame_size:
                chunk = frame_bytes[bytes_sent : bytes_sent + 4096]
                client_socket.sendall(chunk)
                bytes_sent += len(chunk)
            images_sent += 1
            print(f"images sent: {images_sent}", end="\r")
        except Exception as e:
            print(f"Error serving image: {e}")
        finally:
            # Ensure the client socket is closed properly
            client_socket.close()

if __name__ == "__main__":
    # Start the image capturing function in a separate thread
    camera_thread = threading.Thread(target=capture_images)
    camera_thread.daemon = True
    camera_thread.start()
    
    # Change these values according to your server configuration
    host = "192.168.111.1"
    port = 9999
    
    # Run the function to serve images to clients
    serve_images(host, port)
