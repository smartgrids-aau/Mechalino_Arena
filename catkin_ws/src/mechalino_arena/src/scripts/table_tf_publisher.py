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


corner_name = None
corner_id = None

avg_tvec = []
avg_rotation_matrix = []

cv_bridge = None
aruco_marker_detector = None
objPoints = None
markerCorners = None
camera_matrix = None
distortion_coeffs = None

def image_callback(image):
    global corner_name, corner_id, cv_bridge, aruco_marker_detector, camera_matrix, distortion_coeffs, objPoints
    global avg_tvec, avg_rotation_matrix

    # Convert the ROS Image message to an OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(image, desired_encoding="8UC3")
    try:
        # Convert the image to grayscale for marker detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)
        if markerIds is None:
            rospy.logwarn("No corners were detected!")
            return

        # Search for the main marker by ID (specified by corner_id)
        main_id_index = -1
        for index in range(len(markerIds)):
            if markerIds[index] == corner_id:
                main_id_index = index
                break
        if main_id_index == -1:
            rospy.logwarn("Main id not found!")
            return

        # Estimate the pose (rotation and translation vectors) of the detected marker
        retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[main_id_index], camera_matrix, distortion_coeffs)

        # Convert the rotation vector to a rotation matrix
        rotation_matrix, _ = cv2.Rodrigues(rvec)

        # Append the translation vector and rotation matrix to the respective lists for averaging later
        avg_tvec.append(tvec)
        avg_rotation_matrix.append(rotation_matrix)

    except Exception as e:
        # Log any errors that occur during marker detection or pose estimation
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

if __name__ == '__main__':
    rospy.init_node('table_tf_publisher', anonymous=True)

    cv_bridge = CvBridge()

    corners_dictionary = int(rospy.get_param('~corners_dictionary'))
    detectorParams = aruco.DetectorParameters()

    # Set custom ArUco detector parameters for more precise marker detection
    detectorParams.minMarkerPerimeterRate = 0.1
    detectorParams.maxMarkerPerimeterRate = 0.2
    detectorParams.polygonalApproxAccuracyRate = 0.025
    detectorParams.minCornerDistanceRate = 0.005
    detectorParams.minOtsuStdDev = 6
    detectorParams.perspectiveRemovePixelPerCell = 4
    detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.13
    detectorParams.maxErroneousBitsInBorderRate = 0.35
    detectorParams.errorCorrectionRate = 0.5

    # Enable sub-pixel corner refinement to increase pose estimation accuracy
    detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detectorParams.cornerRefinementWinSize = 4
    detectorParams.cornerRefinementMaxIterations = 200
    detectorParams.cornerRefinementMinAccuracy = 0.05

    # Load the predefined ArUco dictionary
    dictionary = aruco.getPredefinedDictionary(corners_dictionary)
    aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)

    # Retrieve parameters for averaging, marker information, and camera calibration
    avgCount = rospy.get_param('~corner_avg_count')
    corner_name = rospy.get_param('~corner_name')
    corner_id = rospy.get_param('~corner_id')

    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    # Define the 3D coordinates of the marker corners (relative to the marker's center)
    corners_marker_size = np.array(rospy.get_param('~corners_marker_size'))
    objPoints = np.zeros((4, 1, 3))
    objPoints[3] = np.array([-corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[2] = np.array([corners_marker_size/2.0, -corners_marker_size/2.0, 0])
    objPoints[1] = np.array([corners_marker_size/2.0, corners_marker_size/2.0, 0])
    objPoints[0] = np.array([-corners_marker_size/2.0, corners_marker_size/2.0, 0])

    broadcaster = tf2_ros.TransformBroadcaster()

    # Subscribe to the camera image topic
    image_subscriber = rospy.Subscriber("/camera/image", Image, image_callback)

    # Wait until we have enough frames to average
    rospy.loginfo("Waiting for enough average count...")
    while len(avg_tvec) < avgCount and not rospy.is_shutdown():
        rospy.sleep(1)
        rospy.loginfo("Current average count: %d", len(avg_tvec))

    # Once we have enough data, stop subscribing to the image topic
    image_subscriber.unregister()

    rospy.sleep(1)
    
    rospy.loginfo("Average count reached! Publishing table tf...")

    # Compute the average translation vector (tvec) and rotation matrix
    avg_tvec = np.array(avg_tvec)
    avg_rotation_matrix = np.array(avg_rotation_matrix)
    final_tvec = np.mean(avg_tvec, axis=0)
    final_rotation_matrix = np.mean(avg_rotation_matrix, axis=0)

    # Convert the average rotation matrix to Euler angles
    euler_angles = cv2.RQDecomp3x3(final_rotation_matrix)[0]

    # Convert the Euler angles to a quaternion (for ROS transform messages)
    quaternion = tf_transformations.quaternion_from_euler(
        np.deg2rad(euler_angles[0]), 
        np.deg2rad(euler_angles[1]), 
        np.deg2rad(euler_angles[2])
    )
    
    # Create the transform message to publish the transformation
    child_frame_id = 'table'
    transform_stamped = TransformStamped()

    transform_stamped.header.frame_id = "camera"
    transform_stamped.child_frame_id = child_frame_id

    # Set the translation component of the transform (from the averaged tvec)
    transform_stamped.transform.translation.x = final_tvec[0]
    transform_stamped.transform.translation.y = final_tvec[1]
    transform_stamped.transform.translation.z = final_tvec[2]

    # Set the rotation component of the transform (from the averaged rotation matrix as a quaternion)
    transform_stamped.transform.rotation.x = quaternion[0]
    transform_stamped.transform.rotation.y = quaternion[1]
    transform_stamped.transform.rotation.z = quaternion[2]
    transform_stamped.transform.rotation.w = quaternion[3]

    # Publish the transformation at a rate of 3 Hz
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():
        try:
            # Set the current timestamp for the transform
            transform_stamped.header.stamp = rospy.Time.now()

            # Broadcast the transform
            broadcaster.sendTransform(transform_stamped)

            rate.sleep()
        except Exception as e:
            rospy.logerr("Error publishing table tf: %s", str(e))
            traceback.print_exc()
            break