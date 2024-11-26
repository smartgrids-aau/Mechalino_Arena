#!/usr/bin/env python

import socket
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

def receive_images(host, port):
    global height, width, camera_matrix, dist_coeff

    # Publishers to send the received images over ROS topics
    # One for the original image and one for the undistorted image
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    uimage_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=10)

    rate = rospy.Rate(10)

    bridge = CvBridge()

    # Main loop to continuously receive and publish images
    while not rospy.is_shutdown():
        try:
            # Create a TCP/IP socket
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # Connect to the image server using the provided host and port
            client_socket.connect((host, port))

            # Receive the size of the incoming image frame in bytes
            frame_size_data = client_socket.recv(8)
            frame_size = int.from_bytes(frame_size_data, byteorder='big')


            received_data = b''
            bytes_received = 0

            # Continue receiving data until all the expected bytes are received
            while bytes_received < frame_size:
                chunk = client_socket.recv(min(4096, frame_size - bytes_received))
                received_data += chunk
                bytes_received += len(chunk)

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
            
            rate.sleep()

        except Exception as e:
            print(f"Error receiving image: {e}")
            exit(-1)

        finally:
            # Ensure the client socket is closed to avoid resource leaks
            client_socket.close()

if __name__ == "__main__":
    rospy.init_node('image_publisher', anonymous=True)

    # Fetch the server host and port from ROS parameters
    server_host = rospy.get_param('~camera_server_ip')
    server_port = int(rospy.get_param('~camera_server_port'))

    # Fetch the image dimensions from ROS parameters
    width = int(rospy.get_param('~image_width'))
    height = int(rospy.get_param('~image_height'))

    # Fetch the camera matrix and distortion coefficients (for undistortion) from ROS parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    dist_coeff = np.array(rospy.get_param('~dist_coeff'))

    # Call the function to start receiving images from the server
    receive_images(server_host, server_port)
