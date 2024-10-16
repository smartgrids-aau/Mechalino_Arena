#!/usr/bin/env python

import socket  # Library for handling network communication (TCP/IP)
import rospy  # ROS Python client library
from cv_bridge import CvBridge  # CV Bridge to convert between ROS Image messages and OpenCV images
from sensor_msgs.msg import Image  # ROS message type for images
import numpy as np  # Numpy for efficient array operations
import cv2  # OpenCV library for image processing

# Function to handle receiving images from a server via TCP socket
def receive_images(host, port):
    # Global variables for image dimensions and camera calibration parameters
    global height, width, camera_matrix, dist_coeff

    # Publishers to send the received images over ROS topics
    # One for the original image and one for the undistorted image
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    uimage_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=10)

    # Set the loop rate (10 Hz), meaning the loop runs 10 times per second
    rate = rospy.Rate(10)

    # Bridge to convert between ROS Image messages and OpenCV images
    bridge = CvBridge()

    # Main loop to continuously receive and publish images
    while not rospy.is_shutdown():
        try:
            # Create a TCP/IP socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the image server using the provided host and port
            client_socket.connect((host, port))

            # Receive the size of the incoming image frame in bytes (8-byte header)
            frame_size_data = client_socket.recv(8)
            frame_size = int.from_bytes(frame_size_data, byteorder='big')  # Convert the received byte data to an integer

            # Initialize variables to store the incoming image data
            received_data = b''  # Empty byte buffer
            bytes_received = 0  # Counter to track the number of bytes received

            # Continue receiving data until all the expected bytes (frame size) are received
            while bytes_received < frame_size:
                # Receive data in chunks (up to 4096 bytes at a time)
                chunk = client_socket.recv(min(4096, frame_size - bytes_received))
                received_data += chunk  # Append the chunk to the buffer
                bytes_received += len(chunk)  # Update the count of received bytes

            # Convert the received byte data into a Numpy array
            np_arr = np.frombuffer(received_data, dtype=np.uint8)

            # Reshape the numpy array into the expected image dimensions (height x width x 3 for RGB)
            img_np = np_arr.reshape((height, width, 3))

            # Convert the Numpy array (OpenCV format) into a ROS Image message
            image_message = bridge.cv2_to_imgmsg(img_np, encoding="bgr8")

            # Publish the ROS Image message on the '/camera/image' topic
            image_pub.publish(image_message)

            # Undistort the image using the camera matrix and distortion coefficients
            uimage = cv2.undistort(img_np, camera_matrix, dist_coeff, None, camera_matrix)

            # Convert the undistorted image to a ROS Image message
            uimage_message = bridge.cv2_to_imgmsg(uimage, encoding="bgr8")

            # Publish the undistorted image on the '/camera/image/undistorted' topic
            uimage_pub.publish(uimage_message)
            
            # Sleep to maintain the loop rate of 10 Hz
            rate.sleep()

        except Exception as e:
            # Handle any exceptions that occur (e.g., connection issues or errors in data processing)
            print(f"Error receiving image: {e}")
            exit(-1)  # Exit the program with an error code

        finally:
            # Ensure the client socket is closed to avoid resource leaks
            client_socket.close()

# Main block to set up the ROS node and initialize parameters
if __name__ == "__main__":
    # Initialize the ROS node with a name ('image_publisher') and set it as anonymous
    rospy.init_node('image_publisher', anonymous=True)

    # Fetch the server host and port from ROS parameters (set in a launch file or parameter server)
    server_host = rospy.get_param('~camera_server_ip')  # Get camera server IP
    server_port = int(rospy.get_param('~camera_server_port'))  # Get camera server port

    # Fetch the image dimensions from ROS parameters
    width = int(rospy.get_param('~image_width'))  # Image width
    height = int(rospy.get_param('~image_height'))  # Image height

    # Fetch the camera matrix and distortion coefficients (for undistortion) from ROS parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))  # Camera matrix (for calibration)
    dist_coeff = np.array(rospy.get_param('~dist_coeff'))  # Distortion coefficients

    # Call the function to start receiving images from the server
    receive_images(server_host, server_port)
