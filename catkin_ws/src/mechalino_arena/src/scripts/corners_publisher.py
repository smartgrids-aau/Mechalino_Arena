#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import traceback
import tf
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

cv_bridge = CvBridge()
def image_callback(msg):
    global cv_bridge, cv_image
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

def corners_tf_publisher():
    global cv_image, camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, broadcaster, tl_id, tr_id, br_id, bl_id
    try:
        # Convert ROS Image message to OpenCV image using CvBridge
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)
        
        for i in range(len(markerIds)):
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)
            
            # Convert rvec to a rotation matrix
            R, _ = cv2.Rodrigues(rvec)

            # Invert the rotation matrix using its transpose
            inverted_R = np.transpose(R)

            # Invert the translation vector
            inverted_tvec = -tvec

            # Convert the inverted rotation matrix to an inverted rotation vector
            inv_rvec, _ = cv2.Rodrigues(inverted_R)

            # Express the inverted translation vector as it is
            inv_tvec = inverted_tvec

            # inv_rvec = rvec
            # inv_tvec = tvec
            
            quaternion = tf.transformations.quaternion_from_euler(inv_rvec[0], inv_rvec[1], inv_rvec[2])

            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "map"
            pose = PoseStamped(header=header)
            pose.pose.position.x = -1 * inv_tvec[0]
            pose.pose.position.y = inv_tvec[1]
            pose.pose.position.z = inv_tvec[2]
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            corner_frame_id = "corner_"
            if markerIds[i] == tl_id:
                corner_frame_id += "tl"
            elif markerIds[i] == tr_id:
                corner_frame_id += "tr"
            elif markerIds[i] == br_id:
                corner_frame_id += "br"
            elif markerIds[i] == bl_id:
                corner_frame_id += "bl"

            corner_frame_id += "_"+ str(markerIds[i])

            broadcaster.sendTransform((-1 * inv_tvec[0], inv_tvec[1], inv_tvec[2]),
                                    quaternion,
                                    header.stamp,
                                    corner_frame_id,
                                    "camera")

            
    except Exception as e:
        rospy.logwarn("Error detecting corners: %s", str(e))

def corners_publisher():
    global camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, broadcaster, tl_id, tr_id, br_id, bl_id


    # Initialize the ROS node
    rospy.init_node('corners_publisher', anonymous=True)

    # load parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    corners_dictionary = np.array(rospy.get_param('~corners_dictionary'))
    detectorParams = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(int(corners_dictionary))
    aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)
    
    corners_marker_size = np.array(rospy.get_param('~corners_marker_size'))
    objPoints = np.zeros((4, 1, 3))
    objPoints[0] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[1] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[2] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
    objPoints[3] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

    tl_id = np.array(rospy.get_param('~tl_id'))
    tr_id = np.array(rospy.get_param('~tr_id'))
    br_id = np.array(rospy.get_param('~br_id'))
    bl_id = np.array(rospy.get_param('~bl_id'))

    rospy.Subscriber("/camera/image", Image, image_callback)

    broadcaster = tf.TransformBroadcaster()

    # first 10 seconds publish corners every second
    slow_down_turns = 10
    rate = rospy.Rate(1)
    while slow_down_turns > 0:
        corners_tf_publisher()
        rate.sleep()
        slow_down_turns -= 1
    # after that publish them every 1min
    rate = rospy.Rate(1/15)
    while not rospy.is_shutdown():
        corners_tf_publisher()
        rate.sleep()

if __name__ == '__main__':
    try:
        corners_publisher()
    except rospy.ROSInterruptException:
        pass