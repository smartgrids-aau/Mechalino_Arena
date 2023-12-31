#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from v4l2ctl import Frame
from PIL import Image as PILImage
import io
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

        # Open the image using PIL
        np_array = np.frombuffer(byte_array, dtype=np.uint8)
        pil_image = PILImage.open(io.BytesIO(np_array))

        # Convert PIL image to OpenCV format
        image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR) 

        return image
    
    except:
        rospy.logwarn("Failed to capture the image!")


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
            rospy.logwarn("Failed to publish the image!" + str(e))

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass