#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Int32MultiArray

def image_callback(data):
    global corners
    global bridge
    global image_pub
    try:
        # do not process if corners are not detected
        if corners is None:
            return
        
        image = bridge.imgmsg_to_cv2(data, desired_encoding="8UC3")

        # mark corners on the image with a blue circle
        for corner in corners:
            cv2.circle(image, tuple(corner), 5, (255, 0, 0), -1)
        
        # gray sacle
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # get hue
        hue = hsv_image[:,:,0]

        # keep only the red color
        # range 0 to 10 and 160 to 180
        mask1 = cv2.inRange(hue, 0, 10)
        mask2 = cv2.inRange(hue, 160, 180)
        mask = mask1 + mask2

        # morphological operations to remove noise
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)


        msg = bridge.cv2_to_imgmsg(image)
        image_pub.publish(msg)
    except CvBridgeError as e:
        rospy.logerr(e)
        
def corners_callback(msg):
    global corners
    corners = np.array(msg.data).reshape(-1, 2)

if __name__ == '__main__':
    try:
        rospy.init_node('obstacle_visualization', anonymous=True)

        image_pub = rospy.Publisher('/obstacle_mask', Image, queue_size=1)

        corners = None # for global access
        rospy.Subscriber('/corners_in_img_cord', Int32MultiArray, corners_callback)

        bridge = CvBridge()
        image_sub = rospy.Subscriber("/camera/image", Image, image_callback)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
