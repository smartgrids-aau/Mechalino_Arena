import cv2
import socket
import threading

# Global variable to hold the latest captured image
global_image = None
image_lock = threading.Lock()

# Function to read images from the camera and update global_image
def capture_images():
    global global_image
    # Open the camera
    camera = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not camera.isOpened():
        print("Failed to connect to the camera")
        exit()

    # Set camera parameters
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    while True:
        ret, frame = camera.read()
        with image_lock:
            global_image = frame

# Function to serve images to clients over TCP socket
def serve_images(host, port):
    global global_image
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(5)
    print(f"Server listening on {host}:{port}")
    
    images_sent = 0
    while True:
        client_socket, addr = server_socket.accept()
        try:
            with image_lock:
                frame_bytes = global_image.tobytes()
            frame_size = len(frame_bytes)
            # Send the number of bytes first
            client_socket.sendall(frame_size.to_bytes(8, byteorder='big'))
            bytes_sent = 0
            while bytes_sent < frame_size:
                chunk = frame_bytes[bytes_sent : bytes_sent + 4096]
                client_socket.sendall(chunk)
                bytes_sent += len(chunk)
            images_sent += 1
            print(f"images sent: {images_sent}", end="\r")
        except Exception as e:
            print(f"Error serving image: {e}")
        finally:
            client_socket.close()

if __name__ == "__main__":
    camera_thread = threading.Thread(target=capture_images)
    camera_thread.daemon = True
    camera_thread.start()
    
    # Change these values according to your server configuration
    host = "192.168.111.2"
    port = 9999
    
    serve_images(host, port)
