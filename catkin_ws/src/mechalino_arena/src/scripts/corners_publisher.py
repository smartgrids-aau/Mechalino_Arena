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

len_of_histoty = 30
history_index = 0
publishable = False
tl_hist = np.zeros((len_of_histoty,3))
tr_hist = np.zeros((len_of_histoty,3))
br_hist = np.zeros((len_of_histoty,3))
bl_hist = np.zeros((len_of_histoty,3))

def publish_tvec(tvec, id):
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera"
    pose = PoseStamped(header=header)
    pose.pose.position.x = -1 * tvec[0]
    pose.pose.position.y = tvec[1]
    pose.pose.position.z = tvec[2]
    pose.pose.orientation.x = quaternion[0]
    pose.pose.orientation.y = quaternion[1]
    pose.pose.orientation.z = quaternion[2]
    pose.pose.orientation.w = quaternion[3]

    corner_frame_id = "corner_"
    if id == tl_id:
        corner_frame_id += "tl"
    elif id == tr_id:
        corner_frame_id += "tr"
    elif id == br_id:
        corner_frame_id += "br"
    elif id == bl_id:
        corner_frame_id += "bl"

    corner_frame_id += "_"+ str(id)

    broadcaster.sendTransform((-1 * tvec[0], tvec[1], tvec[2]),
                            quaternion,
                            header.stamp,
                            corner_frame_id,
                            "camera")
def corners_tf_publisher():
    global cv_image, camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, broadcaster, tl_id, tr_id, br_id, bl_id
    global publishable, history_index, len_of_histoty, tl_hist, tr_hist, br_hist, bl_hist
    try:
        # Convert ROS Image message to OpenCV image using CvBridge
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)

        if (len(markerIds)!=4):
            rospy.logwarn("Not all 4 corners are detectable!")
            return
        
        for id in markerIds:
            if not (id in [tl_id,tr_id,br_id,bl_id]):
                rospy.logwarn("Invalid id detected!")
                return
        for i in range(len(markerIds)):
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)

            # Invert the translation vector
            inverted_tvec = -tvec

            
            inverted_tvec =np.reshape(inverted_tvec,(1,3))

            if markerIds[i] == tl_id:
                tl_hist[history_index,:] = inverted_tvec
            elif markerIds[i] == tr_id:
                tr_hist[history_index,:] = inverted_tvec
            elif markerIds[i] == br_id:
                br_hist[history_index,:] = inverted_tvec
            elif markerIds[i] == bl_id:
                bl_hist[history_index,:] = inverted_tvec

        history_index = (history_index + 1) % len_of_histoty

        if history_index == 0: # all history slots are filled with samples
            publishable = True # after enough sampleing tf will be published

        if (publishable):
            publish_tvec(np.average(tl_hist,axis=0),tl_id)
            publish_tvec(np.average(tr_hist,axis=0),tr_id)
            publish_tvec(np.average(br_hist,axis=0),br_id)
            publish_tvec(np.average(bl_hist,axis=0),bl_id)
        else:
            rospy.loginfo(f"sampling {history_index+1} of {len_of_histoty}")
        

            
    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

def corners_publisher():
    global camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, broadcaster, tl_id, tr_id, br_id, bl_id


    # Initialize the ROS node
    rospy.init_node('corners_publisher', anonymous=True)

    # load parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    corners_dictionary = int(rospy.get_param('~corners_dictionary'))
    detectorParams = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(corners_dictionary)
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

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        corners_tf_publisher()
        rate.sleep()

if __name__ == '__main__':
    try:
        corners_publisher()
    except rospy.ROSInterruptException:
        pass