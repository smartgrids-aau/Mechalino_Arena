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

cv_bridge = CvBridge()
def image_callback(msg):
    global cv_bridge, cv_image
    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="8UC3")

class robot_pose_publisher():
    def __init__(self, id, broadcaster, len_of_hist) -> None:
        self.id = id
        self.broadcaster = broadcaster
        self.len_of_hist = len_of_hist
        self.tvechist = np.empty((self.len_of_hist,3))
        self.rvechist = np.empty((self.len_of_hist,3))
        self.hist_index = 0
    def update(self,tvec,rvec):
        self.tvechist[self.hist_index] = tvec[:, 0]
        self.rvechist[self.hist_index] = rvec[:, 0]
        self.hist_index = (self.hist_index + 1) % self.len_of_hist
        if (self.hist_index == 0): #return back to 0
            self.publish()
    def publish(self):
        inv_tvec = np.average(self.tvechist,axis=0)
        inv_rvec = np.average(self.rvechist,axis=0)

        quaternion = tf.transformations.quaternion_from_euler(inv_rvec[0], inv_rvec[1], inv_rvec[2])

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "camera"
        pose = PoseStamped(header=header)
        pose.pose.position.x = inv_tvec[0]
        pose.pose.position.y = inv_tvec[1]
        pose.pose.position.z = inv_tvec[2]
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]

        self.broadcaster.sendTransform((inv_tvec[0], inv_tvec[1], inv_tvec[2]),
                                quaternion,
                                header.stamp,
                                "mechalino_"+str(self.id),
                                "camera")


def robots_tf_publisher():
    global cv_image, camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, robot_pose_publishers, mechalino_ids
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
                rospy.logwarn(f"Invalid robot id detected: {id}")
                continue

            retval, rvec, tvec = cv2.solvePnP(objPoints, markerCorners[i],camera_matrix,distortion_coeffs)

            robot_index = np.argmax(mechalino_ids == markerIds[i])
            robot_pose_publishers[robot_index].update(tvec,rvec)
            
    except Exception as e:
        rospy.logerr("Error detecting robots: %s", str(e))
        traceback.print_exc()

def robots_publisher():
    global camera_matrix, distortion_coeffs, aruco_marker_detector, objPoints, robot_pose_publishers, mechalino_ids

    # Initialize the ROS node
    rospy.init_node('robots_publisher', anonymous=True)

    # load parameters
    camera_matrix = np.array(rospy.get_param('~camera_matrix'))
    distortion_coeffs = np.array(rospy.get_param('~dist_coeff'))

    robots_dictionary = int(rospy.get_param('~robots_dictionary'))
    detectorParams = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(robots_dictionary)
    aruco_marker_detector = aruco.ArucoDetector(dictionary, detectorParams)
    
    robots_marker_size = np.array(rospy.get_param('~robots_marker_size'))
    objPoints = np.zeros((4, 1, 3))
    objPoints[0] = np.array([-robots_marker_size/2.0, -robots_marker_size/2.0, 0])
    objPoints[1] = np.array([robots_marker_size/2.0, -robots_marker_size/2.0, 0])
    objPoints[2] = np.array([robots_marker_size/2.0, robots_marker_size/2.0, 0])
    objPoints[3] = np.array([-robots_marker_size/2.0, robots_marker_size/2.0, 0])

    mechalino_ids = np.array(rospy.get_param('~mechalino_ids'))[0]

    rospy.loginfo(str(mechalino_ids) + ' are mechalino ids.')
    rospy.Subscriber("/camera/image", Image, image_callback)

    broadcaster = tf.TransformBroadcaster()
    len_of_history = 10
    robot_pose_publishers = []
    for id in mechalino_ids:
        robot_pose_publishers.append(robot_pose_publisher(id,broadcaster,len_of_history))
        
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        robots_tf_publisher()
        rate.sleep()

if __name__ == '__main__':
    try:
        robots_publisher()
    except rospy.ROSInterruptException:
        pass