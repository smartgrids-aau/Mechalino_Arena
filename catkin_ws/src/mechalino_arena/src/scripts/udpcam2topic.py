#!/usr/bin/env python

import socket
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import numpy as np
import cv2

def receive_images(host, port):
    global height, width, camera_matrix, dist_coeff
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=10)
    uimage_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=10)
    rate = rospy.Rate(10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((host, port))

            # Receive the number of bytes first
            frame_size_data = client_socket.recv(8)
            frame_size = int.from_bytes(frame_size_data, byteorder='big')

            received_data = b''
            bytes_received = 0
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

            uimage = cv2.undistort(img_np, camera_matrix, dist_coeff, None, camera_matrix)
            uimage_message = bridge.cv2_to_imgmsg(uimage, encoding="bgr8")
            uimage_pub.publish(uimage_message)
            
            rate.sleep()

        except Exception as e:
            print(f"Error receiving image: {e}")
            exit(-1)
        finally:
            client_socket.close()
if __name__ == "__main__":
    rospy.init_node('image_publisher', anonymous=True)
    # Change these values to match your server configuration
    server_host = rospy.get_param('~camera_server_ip')
    server_port = int(rospy.get_param('~camera_server_port'))

    width = int(rospy.get_param('~image_width'))
    height = int(rospy.get_param('~image_height'))
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    dist_coeff = np.array(rospy.get_param('~dist_coeff'))
    receive_images(server_host, server_port)
