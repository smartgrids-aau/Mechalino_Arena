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
from geometry_msgs.msg import PoseStamped

def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global mechalino_ids, number_of_specified_robots
    global camera_matrix, distortion_coeffs, objPoints
    global broadcaster, pose_publishers
    
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

    try:
        # Convert ROS Image message to OpenCV image using CvBridge
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)

        if markerIds is None:
            rospy.logwarn("No robot is being detected!")
            return

        for id in mechalino_ids:
            if not (id in markerIds):
                rospy.logwarn(f"Mechalino with id {id} is not detected!")
        
        for i in range(len(markerIds)):
            if not (markerIds[i] in mechalino_ids):
                rospy.logwarn(f"Invalid robot id detected: {markerIds[i]}")
                return
            
        for i in range(len(markerIds)):
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)
            
            # tvec =np.reshape(tvec,(1,3))

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Extract Euler angles from rotation matrix
            euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

            robot_frame_id = f"mechalino_{id}"

            transform_stamped = TransformStamped()

            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = "camera"
            transform_stamped.child_frame_id = robot_frame_id

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

            # Publish the pose
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = "camera"
            pose_stamped.pose.position.x = tvec[0]
            pose_stamped.pose.position.y = tvec[1]
            pose_stamped.pose.position.z = tvec[2]
            pose_stamped.pose.orientation.x = quaternion[0]
            pose_stamped.pose.orientation.y = quaternion[1]
            pose_stamped.pose.orientation.z = quaternion[2]
            pose_stamped.pose.orientation.w = quaternion[3]
            robot_index = np.where(mechalino_ids == markerIds[i])[0][0]
            pose_publishers[robot_index].publish(pose_stamped)
            
    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('robots_publisher', anonymous=True)

        cv_bridge = CvBridge()

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

        dictionary = aruco.getPredefinedDictionary(robots_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)
        
        mechalino_ids = np.array(rospy.get_param('~mechalino_ids'))
        number_of_specified_robots = len(mechalino_ids)

        robots_tvec_hist = []
        for i in range(number_of_specified_robots):
            robots_tvec_hist.append([])

        robots_rvec_hist = []
        for i in range(number_of_specified_robots):
            robots_rvec_hist.append([])

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

        robots_marker_size = np.array(rospy.get_param('~robots_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[2] = np.array([robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[1] = np.array([robots_marker_size/2.0, robots_marker_size/2.0, 0])
        objPoints[0] = np.array([-robots_marker_size/2.0, robots_marker_size/2.0, 0])

        broadcaster = tf2_ros.TransformBroadcaster()

        pose_publishers = []
        for i in range(len(mechalino_ids)):
            pose_publishers.append(rospy.Publisher(f"mechalino_{mechalino_ids[i]}", PoseStamped, queue_size=10))

        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass