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
import tf.transformations as tf_trans
from geometry_msgs.msg import TransformStamped
import tf2_ros
from std_msgs.msg import Float32MultiArray


T_table_camera = None

# Callback function that processes incoming camera images
def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global mechalino_ids, number_of_specified_robots
    global camera_matrix, distortion_coeffs, objPoints
    global broadcaster, pose_publishers
    global T_table_camera

    # Convert the ROS Image message into an OpenCV image
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

    try:
        # Convert the image to grayscale for ArUco marker detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers in the grayscale image
        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)

        # Log detected marker IDs for debugging
        rospy.loginfo(f"Detected Marker IDs: {markerIds}")

        # If no markers are detected, log a warning and return
        if markerIds is None:
            rospy.logwarn("No robot is being detected!")
            return

        # Check which Mechalino robot IDs are not detected
        for id in mechalino_ids:
            if id not in markerIds.flatten():
                rospy.logwarn(f"Mechalino with id {id} is not detected!")

        # Process each detected marker
        for i in range(len(markerIds)):
            # Ignore any marker that is not in the list of valid Mechalino IDs
            if markerIds[i][0] not in mechalino_ids:
                rospy.logwarn(f"Invalid robot id detected: {markerIds[i][0]}")
                return

            # Estimate the marker's pose (rotation vector and translation vector)
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i], camera_matrix, distortion_coeffs)

            # Convert the rotation vector (rvec) to a rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Extract Euler angles from the rotation matrix
            euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

            # Convert the Euler angles to a quaternion for use in transformations
            quaternion = tf_transformations.quaternion_from_euler(
                np.deg2rad(euler_angles[0]), np.deg2rad(euler_angles[1]), np.deg2rad(euler_angles[2]))

            # Calculate the transformation matrix from the robot to the camera
            T_robot_camera = tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(tvec.reshape(3)),
                tf_trans.quaternion_matrix(quaternion)
            )

            # Calculate the transformation matrix from the robot to the table
            T_robot_table = tf_trans.concatenate_matrices(T_table_camera, T_robot_camera)

            # Create a TransformStamped message to broadcast the transformation
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "table"
            transform_stamped.child_frame_id = f"mechalino_{markerIds[i][0]}"

            # Set the translation component of the transform (x, y, z coordinates)
            transform_stamped.transform.translation.x = T_robot_table[0, 3]
            transform_stamped.transform.translation.y = T_robot_table[1, 3]
            transform_stamped.transform.translation.z = T_robot_table[2, 3]

            # Set the rotation component of the transform (as a quaternion)
            transform_stamped.transform.rotation.x = tf_trans.quaternion_from_matrix(T_robot_table)[0]
            transform_stamped.transform.rotation.y = tf_trans.quaternion_from_matrix(T_robot_table)[1]
            transform_stamped.transform.rotation.z = tf_trans.quaternion_from_matrix(T_robot_table)[2]
            transform_stamped.transform.rotation.w = tf_trans.quaternion_from_matrix(T_robot_table)[3]

            # Broadcast the transform using the tf2 broadcaster
            broadcaster.sendTransform(transform_stamped)

            # Publish the robot's pose (x, y, yaw) and ID
            pose_data = Float32MultiArray(data=[
                T_robot_table[0, 3],   # x-coordinate of the robot
                T_robot_table[1, 3],   # y-coordinate of the robot
                euler_angles[2],       # z-angle (yaw) of the robot
                markerIds[i][0]        # ID of the detected marker
            ])
            robot_index = np.where(mechalino_ids == markerIds[i])[0][0]  # Find the index of the robot ID
            pose_publishers[robot_index].publish(pose_data)  # Publish the robot's pose

    except Exception as e:
        # Log any exceptions that occur during marker detection or pose estimation
        rospy.logerr(f"Error detecting markers: {str(e)}")
        traceback.print_exc()

if __name__ == '__main__':
    try:
        rospy.init_node('robots_publisher', anonymous=True)

        # Wait for the transform between the camera and the table
        rospy.loginfo("Mechalino publisher is waiting for the table tf...")
        tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                tf_listener.waitForTransform("camera", "table", rospy.Time(), rospy.Duration(1.0))
                break
            except Exception as e:
                pass
        # Get the translation and rotation between the table and the camera
        (trans_table_camera, rot_table_camera) = tf_listener.lookupTransform('table', 'camera', rospy.Time(0))
        T_table_camera = tf_trans.concatenate_matrices(
            tf_trans.translation_matrix(trans_table_camera),
            tf_trans.quaternion_matrix(rot_table_camera)
        )
        rospy.loginfo("Table tf found! Publishing Mechalino tfs...")

        cv_bridge = CvBridge()

        robots_dictionary = int(rospy.get_param('~robots_dictionary'))
        detectorParams = aruco.DetectorParameters()

        # not that important in our problem (these are important when detecting the marker)
        
        #Commented out because id60 was not detected
        #detectorParams = cv2.aruco.DetectorParameters()
        #detectorParams.minMarkerPerimeterRate = 0.08
        #detectorParams.maxMarkerPerimeterRate = 0.2
        #detectorParams.polygonalApproxAccuracyRate = 0.025 #0.05
        #detectorParams.minCornerDistanceRate = 0.005 #0.05
        #detectorParams.minOtsuStdDev = 6 # 5
        #detectorParams.perspectiveRemovePixelPerCell = 4 # 4
        #detectorParams.perspectiveRemoveIgnoredMarginPerCell = 0.13 #0.13
        #detectorParams.maxErroneousBitsInBorderRate = 0.35 #0.35
        #detectorParams.errorCorrectionRate = 0.5 # 0.6

        

        # Important parameters for pose estimation (refining the detected marker corners)
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # CORNER_REFINE_NONE | CORNER_REFINE_SUBPIX | CORNER_REFINE_CONTOUR
        detectorParams.cornerRefinementWinSize = 4
        detectorParams.cornerRefinementMaxIterations = 200
        detectorParams.cornerRefinementMinAccuracy = 0.05

        # Get the predefined ArUco dictionary for detecting markers
        dictionary = aruco.getPredefinedDictionary(robots_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)

        # Load robot IDs and other parameters from the ROS parameter server
        mechalino_ids = np.array(rospy.get_param('~mechalino_ids'))  # Array of Mechalino robot IDs
        number_of_specified_robots = len(mechalino_ids)

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))  # Camera intrinsic matrix
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))  # Camera distortion coefficients

        # Define the 3D coordinates of the robot marker corners
        robots_marker_size = np.array(rospy.get_param('~robots_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-robots_marker_size / 2.0, -robots_marker_size / 2.0, 0])
        objPoints[2] = np.array([robots_marker_size / 2.0, -robots_marker_size / 2.0, 0])
        objPoints[1] = np.array([robots_marker_size / 2.0, robots_marker_size / 2.0, 0])
        objPoints[0] = np.array([-robots_marker_size / 2.0, robots_marker_size / 2.0, 0])

        # Initialize the tf2 transform broadcaster
        broadcaster = tf2_ros.TransformBroadcaster()

        # Create publishers for each robot's pose
        pose_publishers = []
        for i in range(len(mechalino_ids)):
            pose_publishers.append(rospy.Publisher(f"pos/mechalino_{mechalino_ids[i]}", Float32MultiArray, queue_size=1))

        # Subscribe to the camera image topic to start the image processing callback
        rospy.Subscriber("/camera/image", Image, image_callback)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
