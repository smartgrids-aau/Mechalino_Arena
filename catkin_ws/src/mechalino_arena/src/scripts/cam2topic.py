#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from v4l2ctl import Frame
from PIL import Image as PILImage
import io
import threading

# defined in the global scope, so that it can be accessed by the image_updater function {running inside the thread}
capturer = None 
image = None
image_undistorted = None
block_r = False
block_w = False
flag_new = False
no_camera_detected = False
camera_matrix, dist_coeff = None, None

def undistort(image):
    global camera_matrix, dist_coeff
    return cv2.undistort(image, camera_matrix, dist_coeff, None, camera_matrix)

def image_updater():
    global capturer, image, image_undistorted, block_r, block_w, flag_new, no_camera_detected
    bridge = CvBridge()
    while True:
        try:
            ret, image_c = capturer.read()
            if not ret:
                raise Exception('Camera not connected! Exiting capturer frame ...')
            if not block_r:
                block_w = True
                image = bridge.cv2_to_imgmsg(image_c)
                undistorted_cv2_image = undistort(image_c)
                image_undistorted = bridge.cv2_to_imgmsg(undistorted_cv2_image)
                flag_new = True
                block_w = False
        except Exception as e:
            rospy.logerr(e)
            no_camera_detected = True
            break

stream_thread = threading.Thread(target=image_updater)

def camera_publisher():
    global capturer, image, image_undistorted, block_r, block_w, flag_new, no_camera_detected, camera_matrix, dist_coeff
    # Initialize the ROS node
    rospy.init_node('cam2topic', anonymous=True)
    
    camera_id = rospy.get_param('~camera_id')
    capturer = cv2.VideoCapture(camera_id, cv2.CAP_ANY)
    width = int(rospy.get_param('~image_width'))
    height = int(rospy.get_param('~image_height'))
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    dist_coeff = np.array(rospy.get_param('~dist_coeff'))
    capturer.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    capturer.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    stream_thread.start()

    # Create a publisher for the image topic
    image_pub = rospy.Publisher('/camera/image', Image, queue_size=2)
    image_undistorted_pub = rospy.Publisher('/camera/image/undistorted', Image, queue_size=2)

    rate = rospy.Rate(30)  # Publish at 30 Hz
    
    while not rospy.is_shutdown():
        if no_camera_detected:
            rospy.logerr('Camera not connected!')
            break
        else:
            if not flag_new:
                continue
            flag_new = False
            block_r = True
            while(block_w):
                continue
            image_pub.publish(image)
            image_undistorted_pub.publish(image_undistorted)
            block_r = False

        rate.sleep()

    capturer.release()

if __name__ == '__main__':
    try:
        camera_publisher()
    except rospy.ROSInterruptException:
        pass