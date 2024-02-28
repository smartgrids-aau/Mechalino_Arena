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
from std_msgs.msg import Int32MultiArray

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
        

        corners_in_img_cord = np.zeros((4,2))
        for i in range(len(markerIds)):
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)
            error = ma_utility.calculate_reprojection_error(markerCorners[i], objPoints, rvec, tvec, camera_matrix, dist_coeffs=distortion_coeffs)
            # if error < error_tl and markerIds[i] == tl_id:
            #     error_tl = error
            # elif error < error_tr and markerIds[i] == tr_id:
            #     error_tr = error
            # elif error < error_br and markerIds[i] == br_id:
            #     error_br = error
            # elif error < error_bl and markerIds[i] == bl_id:
            #     error_bl = error
            # else:
            #     # increase all errors
            #     error_tl *= 1.005 # 0.5% increase
            #     error_tr *= 1.005 # 0.5% increase
            #     error_br *= 1.005 # 0.5% increase
            #     error_bl *= 1.005 # 0.5% increase
            #     continue

            publish_tvec(tvec, rvec, markerIds[i])

            # convert rvec and tvec to image cordiante system (pixels)
            # if tl, move tvec to half marker size bottom and right
            # if tr, move tvec to half marker size bottom and left
            # if br, move tvec to half marker size top and left
            # if bl, move tvec to half marker size top and right
            if markerIds[i] == tl_id:
                tvec[0] += corners_marker_size/2.0
                tvec[1] += corners_marker_size/2.0
                image_cord = ma_utility.convert_to_image_cord(rvec, tvec, camera_matrix, distortion_coeffs)
                corners_in_img_cord[0] = image_cord
                print("tl is located at: ", image_cord)
            elif markerIds[i] == tr_id:
                tvec[0] -= corners_marker_size/2.0
                tvec[1] += corners_marker_size/2.0
                image_cord = ma_utility.convert_to_image_cord(rvec, tvec, camera_matrix, distortion_coeffs)
                corners_in_img_cord[1] = image_cord
                print("tr is located at: ", image_cord)
            elif markerIds[i] == br_id:
                tvec[0] -= corners_marker_size/2.0
                tvec[1] -= corners_marker_size/2.0
                image_cord = ma_utility.convert_to_image_cord(rvec, tvec, camera_matrix, distortion_coeffs)
                corners_in_img_cord[2] = image_cord
                print("br is located at: ", image_cord)
            elif markerIds[i] == bl_id:
                tvec[0] += corners_marker_size/2.0
                tvec[1] -= corners_marker_size/2.0
                image_cord = ma_utility.convert_to_image_cord(rvec, tvec, camera_matrix, distortion_coeffs)
                corners_in_img_cord[3] = image_cord
                print("bl is located at: ", image_cord)
        # Create Int32MultiArray message
        corners_in_img_cord = corners_in_img_cord.astype(int)
        msg = Int32MultiArray()
        msg.data = corners_in_img_cord.flatten().tolist()  # Flatten the array and convert to list
        corners_in_img_cord_pub.publish(msg)

    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

        


def publish_tvec(tvec, rvec, id):
    global broadcaster
    global corners_in_img_cord_pub

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

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

        corners_marker_size = np.array(rospy.get_param('~corners_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[2] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
        objPoints[1] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
        objPoints[0] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

        broadcaster = tf2_ros.TransformBroadcaster()
        
        # topic to publish x,y position of the corners in image cordiante system
        # it is a numpy array of shape (4,2)
        corners_in_img_cord_pub = rospy.Publisher('/corners_in_img_cord', Int32MultiArray, queue_size=10)
        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        print("ROSInterruptException")