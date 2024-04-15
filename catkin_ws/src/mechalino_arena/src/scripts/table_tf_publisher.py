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

corner_name = None # global access
corner_id = None # global access

avg_tvec = [] # global access
avg_rotation_matrix = [] # global access

cv_bridge = None # global access
aruco_marker_detector = None # global access
objPoints = None # global access
markerCorners = None # global access
camera_matrix = None # global access
distortion_coeffs = None # global access

def image_callback(image):
    global corner_name, corner_id, cv_bridge, aruco_marker_detector, camera_matrix, distortion_coeffs, objPoints
    global avg_tvec, avg_rotation_matrix

    cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding="8UC3")
    try:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)
        if markerIds is None:
            rospy.logwarn("No corners were detected!")
            return
        
        main_id_index = -1
        for index in range(len(markerIds)):
            if markerIds[index] == corner_id:
                main_id_index = index
                break
        if main_id_index == -1:
            rospy.logwarn("Main id not found!")
            return
        
        retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[main_id_index],camera_matrix,distortion_coeffs)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        # error = ma_utility.calculate_reprojection_error(markerCorners[i], objPoints, rvec, tvec, camera_matrix, dist_coeffs=distortion_coeffs)
        avg_tvec.append(tvec)
        avg_rotation_matrix.append(rotation_matrix)
    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('table_tf_publisher', anonymous=True)

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

    avgCount = rospy.get_param('~corner_avg_count')
    corner_name = rospy.get_param('~corner_name')
    corner_id = rospy.get_param('~corner_id')

    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    corners_marker_size = np.array(rospy.get_param('~corners_marker_size'))
    objPoints = np.zeros((4, 1, 3))
    objPoints[3] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[2] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[1] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
    objPoints[0] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

    broadcaster = tf2_ros.TransformBroadcaster()
    
    image_subscriber = rospy.Subscriber("/camera/image", Image, image_callback)
    # wait for enough average count
    rospy.loginfo("Waiting for enough average count...")
    while len(avg_tvec) < avgCount and not rospy.is_shutdown():
        rospy.sleep(1)
        rospy.loginfo("Current average count: %d", len(avg_tvec))
    # disconnect subscriber
    image_subscriber.unregister()
    # wait for unregistering
    rospy.sleep(1)
    
    rospy.loginfo("Average count reached! Publishing table tf...")

    # compute average tvec and rotation matrix
    avg_tvec = np.array(avg_tvec)
    avg_rotation_matrix = np.array(avg_rotation_matrix)
    final_tvec = np.mean(avg_tvec, axis=0)
    final_rotation_matrix = np.mean(avg_rotation_matrix, axis=0)

    euler_angles = cv2.RQDecomp3x3(final_rotation_matrix)[0]
    # Convert rotation vector to quaternion
    quaternion = tf_transformations.quaternion_from_euler(np.deg2rad(euler_angles[0]), np.deg2rad(euler_angles[1]), np.deg2rad(euler_angles[2]))
    
    child_frame_id = 'table'

    transform_stamped = TransformStamped()

    transform_stamped.header.frame_id = "camera"
    transform_stamped.child_frame_id = child_frame_id

    # Set translation
    transform_stamped.transform.translation.x = final_tvec[0]
    transform_stamped.transform.translation.y = final_tvec[1]
    transform_stamped.transform.translation.z = final_tvec[2]

    # Set rotation
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    # Publish at 3 Hz
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        try:
            transform_stamped.header.stamp = rospy.Time.now()
            broadcaster.sendTransform(transform_stamped)
            rate.sleep()
        except Exception as e:
            rospy.logerr("Error publishing table tf: %s", str(e))
            traceback.print_exc()
            break