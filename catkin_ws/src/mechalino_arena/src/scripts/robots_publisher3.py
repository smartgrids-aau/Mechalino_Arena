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

T_table_camera = None  # global access

def image_callback(msg):
    global cv_bridge
    global aruco_marker_detector
    global mechalino_ids, number_of_specified_robots
    global camera_matrix, distortion_coeffs, objPoints
    global broadcaster, pose_publishers
    global T_table_camera

    # Konvertiere das ROS-Bild in ein OpenCV-Bild
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

    try:
        # Konvertiere das Bild zu Graustufen für die Marker-Erkennung
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ARuco Marker erkennen
        markerCorners, markerIds, _ = aruco_marker_detector.detectMarkers(gray)

        # Debug: Zeige die erkannten Marker-IDs an
        rospy.loginfo(f"Erkannte Marker-IDs: {markerIds}")

        if markerIds is None:
            rospy.logwarn("No robot is being detected!")
            return

        # Prüfe, welche Marker-IDs nicht erkannt wurden
        for id in mechalino_ids:
            if id not in markerIds.flatten():
                rospy.logwarn(f"Mechalino with id {id} is not detected!")

        # Bearbeite die erkannten Marker
        for i in range(len(markerIds)):
            if markerIds[i][0] not in mechalino_ids:
                rospy.logwarn(f"Invalid robot id detected: {markerIds[i][0]}")
                return

            # Berechne die Pose des Markers (Rotation und Translation)
            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i], camera_matrix, distortion_coeffs)

            # Konvertiere den Rotationsvektor in eine Rotationsmatrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)

            # Extrahiere die Euler-Winkel aus der Rotationsmatrix
            euler_angles = cv2.RQDecomp3x3(rotation_matrix)[0]

            # Konvertiere den Rotationsvektor in Quaternionen
            quaternion = tf_transformations.quaternion_from_euler(
                np.deg2rad(euler_angles[0]), np.deg2rad(euler_angles[1]), np.deg2rad(euler_angles[2]))

            # Berechne die Transformation zwischen Roboter und Kamera
            T_robot_camera = tf_trans.concatenate_matrices(
                tf_trans.translation_matrix(tvec.reshape(3)),
                tf_trans.quaternion_matrix(quaternion)
            )

            # Berechne die Transformation zwischen Roboter und Tisch
            T_robot_table = tf_trans.concatenate_matrices(T_table_camera, T_robot_camera)

            # Erstelle eine TransformStamped-Nachricht
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

            # Sende die Transform-Nachricht
            broadcaster.sendTransform(transform_stamped)

            # Veröffentliche die Pose des Roboters zusammen mit seiner ID und dem Z-Winkel (Yaw)
            pose_data = Float32MultiArray(data=[
                T_robot_table[0, 3],   # x-Koordinate
                T_robot_table[1, 3],   # y-Koordinate
                euler_angles[2],       # z-Winkel (Yaw)
                markerIds[i][0]        # ARuco-Marker-ID
            ])
            robot_index = np.where(mechalino_ids == markerIds[i])[0][0]
            pose_publishers[robot_index].publish(pose_data)

    except Exception as e:
        rospy.logerr(f"Error detecting markers: {str(e)}")
        traceback.print_exc()

if __name__ == '__main__':
    try:
        # Initialisiere den ROS-Node
        rospy.init_node('robots_publisher', anonymous=True)

        # Warte auf das Transform zwischen Kamera und Tisch
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

        # Initialisiere den CvBridge
        cv_bridge = CvBridge()

        # ARuco-Detektor initialisieren
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

        # Very important! These are important for pose estimation
        detectorParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX # CORNER_REFINE_NONE | CORNER_REFINE_SUBPIX | CORNER_REFINE_CONTOUR
        # for subpix
        detectorParams.cornerRefinementWinSize = 4 # 5
        detectorParams.cornerRefinementMaxIterations = 200 # 30
        detectorParams.cornerRefinementMinAccuracy = 0.05 # 0.1

        dictionary = aruco.getPredefinedDictionary(robots_dictionary)
        aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)

        # Lade die mechalino-IDs und andere Parameter
        mechalino_ids = np.array(rospy.get_param('~mechalino_ids'))
        number_of_specified_robots = len(mechalino_ids)

        camera_matrix = np.array(rospy.get_param('~camera_matrix'))
        distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

        robots_marker_size = np.array(rospy.get_param('~robots_marker_size'))
        objPoints = np.zeros((4, 1, 3))
        objPoints[3] = np.array([-robots_marker_size / 2.0, -robots_marker_size / 2.0, 0])
        objPoints[2] = np.array([robots_marker_size / 2.0, -robots_marker_size / 2.0, 0])
        objPoints[1] = np.array([robots_marker_size / 2.0, robots_marker_size / 2.0, 0])
        objPoints[0] = np.array([-robots_marker_size / 2.0, robots_marker_size / 2.0, 0])

        # Initialisiere den Transform-Broadcaster
        broadcaster = tf2_ros.TransformBroadcaster()

        # Erstelle Pose-Publisher für jeden Roboter
        pose_publishers = []
        for i in range(len(mechalino_ids)):
            pose_publishers.append(rospy.Publisher(f"pos/mechalino_{mechalino_ids[i]}", Float32MultiArray, queue_size=1))

        # Abonniere das Kamerabild und starte den Callback
        rospy.Subscriber("/camera/image", Image, image_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
