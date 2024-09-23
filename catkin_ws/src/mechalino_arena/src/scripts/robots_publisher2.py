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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray

T_table_camera = None # global access

def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global mechalino_ids, number_of_specified_robots
    global camera_matrix, distortion_coeffs, objPoints
    global broadcaster, pose_publishers
    global T_table_camera

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
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i], camera_matrix, distortion_coeffs)

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
            transform_stamped.child_frame_id = f"mechalino_{markerIds[i][0]}"
            transform_stamped.transform.translation.x = T_robot_table[0, 3]
            transform_stamped.transform.translation.y = T_robot_table[1, 3]
            transform_stamped.transform.translation.z = T_robot_table[2, 3]
            transform_stamped.transform.rotation.x = tf_trans.quaternion_from_matrix(T_robot_table)[0]
            transform_stamped.transform.rotation.y = tf_trans.quaternion_from_matrix(T_robot_table)[1]
            transform_stamped.transform.rotation.z = tf_trans.quaternion_from_matrix(T_robot_table)[2]
            transform_stamped.transform.rotation.w = tf_trans.quaternion_from_matrix(T_robot_table)[3]

            # Broadcast the transform
            broadcaster.sendTransform(transform_stamped)

            # Publish the pose of the robot along with its ID
            pose_data = Float32MultiArray(data=[T_robot_table[0, 3], T_robot_table[1, 3], euler_angles[2], markerIds[i][0]])
            robot_index = np.where(mechalino_ids == markerIds[i])[0][0]
            pose_publishers[robot_index].publish(pose_data)

    except Exception as e:
        rospy.logerr("Error detecting corners: %s", str(e))
        traceback.print_exc()

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('robots_publisher', anonymous=True)

        # Setzen des Standardwertes für robots_dictionary
        robots_dictionary = int(rospy.get_param('~robots_dictionary', 1))  # 10 ist hier ein Beispielwert

        # Feste Kamera-Matrix und Verzerrungskoeffizienten
        camera_matrix = np.array([[1254.9086631872176, 0.0, 1001.8711926600247],
                                  [0.0, 1258.9000885674702, 551.2156190689843],
                                  [0.0, 0.0, 1.0]])

        distortion_coeffs = np.array([-0.4004486439793733, 0.2106824447096251, -0.000240543913508847, -0.0013062327622668823, -0.06153362032103862])

        # Die restlichen Initialisierungen bleiben gleich
        rospy.loginfo("Mechalino publisher is Waiting for the table tf...")
        tf_listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                tf_listener.waitForTransform("camera", "table", rospy.Time(), rospy.Duration(1.0))
                break
            except Exception as e:
                pass
        (trans_table_camera, rot_table_camera) = tf_listener.lookupTransform('table', 'camera', rospy.Time(0))
        T_table_camera = tf_trans.concatenate_matrices(tf_trans.translation_matrix(trans_table_camera),
                                                            tf_trans.quaternion_matrix(rot_table_camera))
        rospy.loginfo("Table tf found! Publishing mechalino tfs ...")

        cv_bridge = CvBridge()

        detectorParams = aruco.DetectorParameters()

        # Die restliche Initialisierung und das Starten des Subscribers
        dictionary = aruco.getPredefinedDictionary(robots_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)

        mechalino_ids = np.array(rospy.get_param('~mechalino_ids', [15, 60, 3]))  # Beispielwerte
        number_of_specified_robots = len(mechalino_ids)

        robots_marker_size = np.array(rospy.get_param('~robots_marker_size', 0.0663))  # Beispielwert 0.05 Meter
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[2] = np.array([robots_marker_size/2.0, -robots_marker_size/2.0, 0])
        objPoints[1] = np.array([robots_marker_size/2.0, robots_marker_size/2.0, 0])
        objPoints[0] = np.array([-robots_marker_size/2.0, robots_marker_size/2.0, 0])

        broadcaster = tf2_ros.TransformBroadcaster()

        pose_publishers = []
        for i in range(len(mechalino_ids)):
            pose_publishers.append(rospy.Publisher(f"pos/mechalino_new{mechalino_ids[i]}", Float32MultiArray, queue_size=1))

        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
