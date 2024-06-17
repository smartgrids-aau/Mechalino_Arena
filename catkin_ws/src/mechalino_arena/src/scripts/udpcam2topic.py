#!/usr/bin/env python

"""
This program receives images from a server via a TCP socket, converts them into ROS Image messages, 
publishes the raw and undistorted images to respective ROS topics. The main function, receive_images, 
handles the reception and processing of images.
"""

import socket  # Library for network communication
import rospy  # ROS Python client library
from cv_bridge import CvBridge  # For converting between ROS Image messages and OpenCV images
from sensor_msgs.msg import Image  # ROS Image message type
import numpy as np  # Library for numerical operations
import cv2  # OpenCV library for image processing

def receive_images(host, port):
    """
    Receives images from a server via a TCP socket, publishes the raw and undistorted images to ROS topics.

    Args:
        host (str): The IP address of the server.
        port (int): The port number of the server.
    """
    global height, width, camera_matrix, dist_coeff

    # Initialize ROS publishers for raw and undistorted images
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    uimage_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=10)
    
    rate = rospy.Rate(10) # Set the loop rate for publishing
    bridge = CvBridge() # Initialize CvBridge for converting between ROS and OpenCV images

    while not rospy.is_shutdown():
        try:
            # Create a TCP socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((host, port))

            # Receive the number of bytes first
            frame_size_data = client_socket.recv(8)
            frame_size = int.from_bytes(frame_size_data, byteorder='big')

            # Initialize variables for receiving image data
            received_data = b''
            bytes_received = 0

            # Receive image data in chunks until the entire frame is received
            while bytes_received < frame_size:
                chunk = client_socket.recv(min(4096, frame_size - bytes_received))
                received_data += chunk
                bytes_received += len(chunk)

            # Convert received bytes to numpy array and reshape to image dimensions
            np_arr = np.frombuffer(received_data, dtype=np.uint8)
            img_np = np_arr.reshape((height, width, 3))  # Adjust the shape according to your image dimensions
            # Convert numpy array to ROS Image message
            image_message = bridge.cv2_to_imgmsg(img_np, encoding="bgr8")
            image_pub.publish(image_message)

            # Undistort the image using the camera matrix and distortion coefficients
            uimage = cv2.undistort(img_np, camera_matrix, dist_coeff, None, camera_matrix)
            # Convert undistorted image to ROS Image message and publish
            uimage_message = bridge.cv2_to_imgmsg(uimage, encoding="bgr8")
            uimage_pub.publish(uimage_message)
            
            rate.sleep()

        except Exception as e:
            # Log any errors that occur during image reception and processing
            print(f"Error receiving image: {e}")
            exit(-1)
        finally:
            # Ensure the socket is closed after each connection
            client_socket.close()
if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('image_publisher', anonymous=True)
    # Change these values to match your server configuration
    server_host = rospy.get_param('~camera_server_ip')
    server_port = int(rospy.get_param('~camera_server_port'))

    width = int(rospy.get_param('~image_width'))
    height = int(rospy.get_param('~image_height'))
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    dist_coeff = np.array(rospy.get_param('~dist_coeff'))
    # Call the receive_images function with the server host and port
    receive_images(server_host, server_port)
