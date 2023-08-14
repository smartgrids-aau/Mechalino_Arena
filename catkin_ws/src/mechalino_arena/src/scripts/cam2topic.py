#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from v4l2ctl import Frame

# initialize the cameras
try:
    camera = Frame('/dev/video0')
except Exception as e:
    rospy.logerr("Error: failed to connect to the camera!")
    raise e

def capture():
    global camera
    try:
        byte_array = camera.get_frame()

        # Convert the bytearray to a numpy array
        np_array = np.frombuffer(byte_array, dtype=np.uint8)
        # Decode the numpy array to an OpenCV image
        image = cv2.imdecode(np_array, cv2.IMREAD_COLOR)

        return image
    
    except:
        rospy.logerr("Failed to capture the image!")


def undistort(image):
    global camera_matrix, distortion_coeffs
    return cv2.undistort(image, camera_matrix, distortion_coeffs, None, camera_matrix)

def camera_publisher():
    global camera
    global camera_matrix, distortion_coeffs

    # Initialize the ROS node
    rospy.init_node('cam2topic', anonymous=True)

    # load parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=1)
    undistorted_image_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=1)
    
    bridge = CvBridge()
    while not rospy.is_shutdown():
        try:
            cv2_image = capture()

            image = bridge.cv2_to_imgmsg(cv2_image)
            image_pub.publish(image)

            undistorted_cv2_image = undistort(cv2_image)
            undistorted_image = bridge.cv2_to_imgmsg(undistorted_cv2_image)
            undistorted_image_pub.publish(undistorted_image)
        except Exception as e:
            rospy.logerr("Failed to publish the image!" + str(e))

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass