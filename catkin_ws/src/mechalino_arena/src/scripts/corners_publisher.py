#!/usr/bin/env python3
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2.aruco as aruco
import traceback
import tf
import tf.transformations as tf_transformations
from geometry_msgs.msg import TransformStamped
import tf2_ros
import mechalino_arena_utility as ma_utility

error_tl = 1000 # arbitrary large number
error_tr = 1000 # arbitrary large number
error_br = 1000 # arbitrary large number
error_bl = 1000 # arbitrary large number

def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global tl_id, tr_id, br_id, bl_id
    global camera_matrix, distortion_coeffs, objPoints
    global tl_hist, tr_hist, br_hist, bl_hist, tableCornerHistoricalLength
    global error_tl, error_tr, error_br, error_bl
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

    try:
        # Convert ROS Image message to OpenCV image using CvBridge
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)
        if markerIds is None:
            rospy.logwarn("No corners were detected!")
            return
        
        if (len(markerIds)!=4):
            rospy.logwarn("Not all 4 corners are detectable!")
            return
        
        for id in markerIds:
            if not (id in [tl_id,tr_id,br_id,bl_id]):
                rospy.logwarn("Invalid id detected!")
                return
            
        for i in range(len(markerIds)):
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)
            error = ma_utility.calculate_reprojection_error(markerCorners[i], objPoints, rvec, tvec, camera_matrix, dist_coeffs=distortion_coeffs)
            print(f"error: {error}")
            if error < error_tl and markerIds[i] == tl_id:
                error_tl = error
            elif error < error_tr and markerIds[i] == tr_id:
                error_tr = error
            elif error < error_br and markerIds[i] == br_id:
                error_br = error
            elif error < error_bl and markerIds[i] == bl_id:
                error_bl = error
            else:
                # increase all errors
                error_tl += 0.01 # 1 cm
                error_tr += 0.01 # 1 cm
                error_br += 0.01 # 1 cm
                error_bl += 0.01 # 1 cm
                break

            tvec =np.reshape(tvec,(1,3))
            rvec =np.reshape(rvec,(1,3))

            if markerIds[i] == tl_id:
                tl_hist.append([tvec,rvec])
            elif markerIds[i] == tr_id:
                tr_hist.append([tvec,rvec])
            elif markerIds[i] == br_id:
                br_hist.append([tvec,rvec])
            elif markerIds[i] == bl_id:
                bl_hist.append([tvec,rvec])

        if (len(tl_hist)==tableCornerHistoricalLength):
            tl_avg_tvec = np.average(np.array(tl_hist)[:,0].reshape((tableCornerHistoricalLength,3)),axis=0)
            tl_avg_rvec = np.average(np.array(tl_hist)[:,1].reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(tl_avg_tvec,tl_avg_rvec,tl_id)
            tl_hist.clear()

        if (len(tr_hist)==tableCornerHistoricalLength):
            tr_avg_tvec = np.average(np.array(tr_hist)[:,0].reshape((tableCornerHistoricalLength,3)),axis=0)
            tr_avg_rvec = np.average(np.array(tr_hist)[:,1].reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(tr_avg_tvec,tr_avg_rvec,tr_id)
            tr_hist.clear()

        if (len(br_hist)==tableCornerHistoricalLength):
            br_avg_tvec = np.average(np.array(br_hist)[:,0].reshape((tableCornerHistoricalLength,3)),axis=0)
            br_avg_rvec = np.average(np.array(br_hist)[:,1].reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(br_avg_tvec,br_avg_rvec,br_id)
            br_hist.clear()

        if (len(bl_hist)==tableCornerHistoricalLength):
            bl_avg_tvec = np.average(np.array(bl_hist)[:,0].reshape((tableCornerHistoricalLength,3)),axis=0)
            bl_avg_rvec = np.average(np.array(bl_hist)[:,1].reshape((tableCornerHistoricalLength,3)),axis=0)
            publish_tvec(bl_avg_tvec,bl_avg_rvec,bl_id)
            bl_hist.clear()
            
    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

        


def publish_tvec(tvec, rvec, id):
    global broadcaster
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Extract Euler angles from rotation matrix
    euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

    corner_frame_id = "corner_"
    if id == tl_id:
        corner_frame_id += "tl"
    elif id == tr_id:
        corner_frame_id += "tr"
    elif id == br_id:
        corner_frame_id += "br"
    elif id == bl_id:
        corner_frame_id += "bl"

    transform_stamped = TransformStamped()

    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "camera"
    transform_stamped.child_frame_id = corner_frame_id

    # Set translation
    transform_stamped.transform.translation.x = tvec[0]
    transform_stamped.transform.translation.y = tvec[1]
    transform_stamped.transform.translation.z = tvec[2]

    # Convert rotation vector to quaternion
    quaternion = tf_transformations.quaternion_from_euler(np.deg2rad(euler_angles[0]), np.deg2rad(euler_angles[1]), np.deg2rad(euler_angles[2]))

    # Set rotation
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    broadcaster.sendTransform(transform_stamped)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('corners_publisher', anonymous=True)

        cv_bridge = CvBridge()

        corners_dictionary = int(rospy.get_param('~corners_dictionary'))
        detectorParams = aruco.DetectorParameters()

        # not that important in our problem (these are important when detecting the marker)
        detectorParams = cv2.aruco.DetectorParameters()
        detectorParams.minMarkerPerimeterRate = 0.1
        detectorParams.maxMarkerPerimeterRate = 0.2
        detectorParams.polygonalApproxAccuracyRate = 0.025 #0.05
        detectorParams.minCornerDistanceRate = 0.005 #0.05
        detectorParams.minOtsuStdDev = 6 # 5
        detectorParams.perspectiveRemovePixelPerCell = 4 # 4
        detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.13 #0.13
        detectorParams.maxErroneousBitsInBorderRate = 0.35 #0.35
        detectorParams.errorCorrectionRate = 0.5 # 0.6

        # Very important! These are important for pose estimation
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # CORNER_REFINE_NONE | CORNER_REFINE_SUBPIX | CORNER_REFINE_CONTOUR
        # for subpix
        detectorParams.cornerRefinementWinSize = 4 # 5
        detectorParams.cornerRefinementMaxIterations = 200 # 30
        detectorParams.cornerRefinementMinAccuracy = 0.05 # 0.1

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
        objPoints[3] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[2] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[1] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
        objPoints[0] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

        broadcaster = tf2_ros.TransformBroadcaster()

        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")