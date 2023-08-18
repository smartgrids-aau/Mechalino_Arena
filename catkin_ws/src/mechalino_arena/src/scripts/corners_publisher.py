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

def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global tl_id, tr_id, br_id, bl_id
    global camera_matrix, distortion_coeffs, objPoints
    global tl_hist, tr_hist, br_hist, bl_hist, tableCornerHistoricalLength

    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

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
            
            tvec =np.reshape(tvec,(1,3))

            if markerIds[i] == tl_id:
                tl_hist.append(tvec + objPoints[0])
            elif markerIds[i] == tr_id:
                tr_hist.append(tvec + objPoints[1])
            elif markerIds[i] == br_id:
                br_hist.append(tvec + objPoints[2])
            elif markerIds[i] == bl_id:
                bl_hist.append(tvec + objPoints[3])

        if (len(tl_hist)==tableCornerHistoricalLength):
            tl_avg = np.average(np.array(tl_hist).reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(tl_avg,tl_id)
            tl_hist.clear()

        if (len(tr_hist)==tableCornerHistoricalLength):
            tr_avg = np.average(np.array(tr_hist).reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(tr_avg,tr_id)
            tr_hist.clear()

        if (len(br_hist)==tableCornerHistoricalLength):
            br_avg = np.average(np.array(br_hist).reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(br_avg,br_id)
            br_hist.clear()

        if (len(bl_hist)==tableCornerHistoricalLength):
            bl_avg = np.average(np.array(bl_hist).reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(bl_avg,bl_id)
            bl_hist.clear()
            
    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

        


def publish_tvec(tvec, id):
    global broadcaster
    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)

    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "camera"
    pose = PoseStamped(header=header)
    pose.pose.position.x = tvec[0]
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

    broadcaster.sendTransform((tvec[0], tvec[1], tvec[2]),
                            quaternion,
                            header.stamp,
                            corner_frame_id,
                            "camera")

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('corners_publisher', anonymous=True)

        cv_bridge = CvBridge()

        corners_dictionary = int(rospy.get_param('~corners_dictionary'))
        detectorParams = aruco.DetectorParameters()
        dictionary = aruco.getPredefinedDictionary(corners_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)

        tl_id = np.array(rospy.get_param('~tl_id'))
        tr_id = np.array(rospy.get_param('~tr_id'))
        br_id = np.array(rospy.get_param('~br_id'))
        bl_id = np.array(rospy.get_param('~bl_id'))

        tableCornerHistoricalLength = int(rospy.get_param('~tableCornerHistoricalLength'))

        tl_hist = []
        tr_hist = []
        br_hist = []
        bl_hist = []

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

        corners_marker_size = np.array(rospy.get_param('~corners_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[0] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[1] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[2] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
        objPoints[3] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

        broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass