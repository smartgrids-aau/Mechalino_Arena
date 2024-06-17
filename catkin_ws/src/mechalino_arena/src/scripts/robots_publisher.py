#!/usr/bin/env python3

"""
This program captures images from a ROS topic, detects ArUco markers on robots, calculates their poses,
and publishes the transformations and poses to respective ROS topics. The main function handles the detection
and processing of the ArUco markers and computes the transformations between the robots and the table.
"""

import numpy as np  # Library for numerical operations
import rospy  # ROS Python client library
import cv2  # OpenCV library for image processing
from sensor_msgs.msg import Image  # ROS Image message type
from cv_bridge import CvBridge  # For converting between ROS Image messages and OpenCV images
import cv2.aruco as aruco  # ArUco marker detection
import traceback  # Provides error traceback information
import tf  # ROS transform library
import tf.transformations as tf_transformations  # Provides transformation utilities
import tf.transformations as tf_trans  # Alias for transformation utilities
from geometry_msgs.msg import TransformStamped  # ROS TransformStamped message type
import tf2_ros  # ROS transform broadcaster
from geometry_msgs.msg import PoseStamped  # ROS PoseStamped message type
from std_msgs.msg import Float32MultiArray  # ROS Float32MultiArray message type

# Global variable to store the transformation between the table and the camera
T_table_camera = None # global access

def image_callback(msg):
    """
    Processes the incoming image to detect ArUco markers and calculate the poses of the robots.

    Args:
        msg (sensor_msgs.msg.Image): The incoming image message.
    """
    global cv_bridge
    global aruco_marker_detector
    global mechalino_ids, number_of_specified_robots
    global camera_matrix, distortion_coeffs, objPoints
    global broadcaster, pose_publishers
    global T_table_camera

    # Convert the ROS Image message to an OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

    try:
        # Convert the image to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the image
        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)

        if markerIds is None:
            rospy.logwarn("No robot is being detected!")
            return

        # Check if all specified Mechalino IDs are detected
        for id in mechalino_ids:
            if not (id in markerIds):
                rospy.logwarn(f"Mechalino with id {id} is not detected!")
        
        # Ensure that only specified Mechalino IDs are detected
        for i in range(len(markerIds)):
            if not (markerIds[i] in mechalino_ids):
                rospy.logwarn(f"Invalid robot id detected: {markerIds[i]}")
                return
            
        for i in range(len(markerIds)):
            # Calculate the pose of the marker
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            # Extract Euler angles from rotation matrix
            euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

            # Convert rotation vector to quaternion
            quaternion = tf_transformations.quaternion_from_euler(np.deg2rad(euler_angles[0]), np.deg2rad(euler_angles[1]), np.deg2rad(euler_angles[2]))
            
            # Get the transformations between robot and camera
            T_robot_camera = tf_trans.concatenate_matrices(tf_trans.translation_matrix(tvec.reshape(3)),
                                                            tf_trans.quaternion_matrix(quaternion))

            # Compute the transformation between robot and table
            T_robot_table = tf_trans.concatenate_matrices(T_table_camera, T_robot_camera)

            # Create a TransformStamped message
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "table"
            transform_stamped.child_frame_id = f"mechalino_{id}"
            transform_stamped.transform.translation.x = T_robot_table[0, 3]
            transform_stamped.transform.translation.y = T_robot_table[1, 3]
            transform_stamped.transform.translation.z = T_robot_table[2, 3]
            transform_stamped.transform.rotation.x = tf_trans.quaternion_from_matrix(T_robot_table)[0]
            transform_stamped.transform.rotation.y = tf_trans.quaternion_from_matrix(T_robot_table)[1]
            transform_stamped.transform.rotation.z = tf_trans.quaternion_from_matrix(T_robot_table)[2]
            transform_stamped.transform.rotation.w = tf_trans.quaternion_from_matrix(T_robot_table)[3]

            # Broadcast the transform
            broadcaster.sendTransform(transform_stamped)

            # Publish the pose of the robot
            pose_data = Float32MultiArray(data=[T_robot_table[0, 3], T_robot_table[1, 3]])
            robot_index = np.where(mechalino_ids == markerIds[i])[0][0]
            pose_publishers[robot_index].publish(pose_data)

    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('robots_publisher', anonymous=True)

        # wait for tf to be ready
        # check if table tf is available
        rospy.loginfo("Mechalino publisher is Waiting for the table tf...")
        tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                tf_listener.waitForTransform("camera", "table", rospy.Time(), rospy.Duration(1.0))
                break
            except Exception as e:
                pass
        # Get the transformation between the table and the camera
        (trans_table_camera, rot_table_camera) = tf_listener.lookupTransform('table', 'camera', rospy.Time(0))
        T_table_camera = tf_trans.concatenate_matrices(tf_trans.translation_matrix(trans_table_camera),
                                                            tf_trans.quaternion_matrix(rot_table_camera))
        rospy.loginfo("Table tf found! Publishing mechalino tfs ...")

        # Initialize the CvBridge object
        cv_bridge = CvBridge()

        # Retrieve parameters for ArUco marker detection and pose estimation
        robots_dictionary = int(rospy.get_param('~robots_dictionary'))
        detectorParams = aruco.DetectorParameters()

        # not that important in our problem (these are important when detecting the marker)
        detectorParams = cv2.aruco.DetectorParameters()
        detectorParams.minMarkerPerimeterRate = 0.08
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

        # Initialize the ArUco marker detector
        dictionary = aruco.getPredefinedDictionary(robots_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)
        
        mechalino_ids = np.array(rospy.get_param('~mechalino_ids'))
        number_of_specified_robots = len(mechalino_ids)

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

        robots_marker_size = np.array(rospy.get_param('~robots_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[2] = np.array([robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[1] = np.array([robots_marker_size/2.0, robots_marker_size/2.0, 0])
        objPoints[0] = np.array([-robots_marker_size/2.0, robots_marker_size/2.0, 0])

        # Initialize the transform broadcaster
        broadcaster = tf2_ros.TransformBroadcaster()

        # Initialize pose publishers for each robot
        pose_publishers = []
        for i in range(len(mechalino_ids)):
            pose_publishers.append(rospy.Publisher(f"pos/mechalino_{mechalino_ids[i]}", Float32MultiArray, queue_size=1))

        # Subscribe to the camera image topic
        rospy.Subscriber("/camera/image", Image, image_callback)
        # Spin to keep the script running
        rospy.spin()
    except rospy.ROSInterruptException:
        pass