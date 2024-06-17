#!/usr/bin/env python

"""
This program generates a point cloud for a table using transforms from ROS. It extracts corner points of the table,
generates intermediate points between these corners, and publishes the point cloud to a ROS topic.
"""

import rospy  # ROS Python client library
import tf2_ros  # ROS library for transforming coordinates
from sensor_msgs.msg import PointCloud2, PointField  # ROS PointCloud2 and PointField message types
from geometry_msgs.msg import TransformStamped  # ROS TransformStamped message type
import numpy as np  # Library for numerical operations

class PointCloudGenerator:
    def __init__(self):
        """
        Initializes the PointCloudGenerator class, sets up the ROS node, the transform listener, and the point cloud publisher.
        """
        # Initialize the ROS node
        rospy.init_node('point_cloud_generator')
        # Create a buffer and listener for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Get the resolution parameter from ROS parameter server, defaulting to 0.02 if not set
        self.resolution = rospy.get_param('~resolution', 0.02)  # Default resolution
        # Set up the point cloud publisher
        self.point_cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=1)

    def generate_point_cloud(self):
        """
        Generates the point cloud by extracting corner points of the table, creating lines between them,
        and generating intermediate points.
        """
        corners = ['corner_tl', 'corner_tr', 'corner_bl', 'corner_br']
        point_cloud = []

        # Extract corner points
        corner_points = []
        for corner in corners:
            try:
                # Lookup the transform for each corner of the table
                transform = self.tf_buffer.lookup_transform('table', corner, rospy.Time(0), rospy.Duration(1.0))
                corner_points.append(np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ]))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn("Failed to lookup transform for {}".format(corner))
                return

        # Define lines between TL and BL, and TR and BR
        line_tl_bl = np.linspace(corner_points[0], corner_points[2], int(np.linalg.norm(corner_points[0] - corner_points[2]) / self.resolution))
        line_tr_br = np.linspace(corner_points[1], corner_points[3], int(np.linalg.norm(corner_points[1] - corner_points[3]) / self.resolution))

        # Generate points between the lines
        for point_tl_bl, point_tr_br in zip(line_tl_bl, line_tr_br):
            line = np.linspace(point_tl_bl, point_tr_br, int(np.linalg.norm(point_tl_bl - point_tr_br) / self.resolution))
            for point in line:
                pc_point = [point[0], point[1], point[2]]
                point_cloud.append(pc_point)

        # Publish the generated point cloud
        self.publish_point_cloud(point_cloud)


    def publish_point_cloud(self, points):
        """
        Publishes the generated points as a ROS PointCloud2 message.

        Args:
            points (list): List of points to be published.
        """
        # Create the header for the point cloud message
        header = rospy.Header()
        header.frame_id = 'table'

        # Define the fields for the point cloud message
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        # Create the PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header = header
        cloud_msg.fields = fields
        cloud_msg.is_bigendian = False
        cloud_msg.point_step = 12
        cloud_msg.row_step = 12 * len(points)
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.is_dense = False
        cloud_msg.data = np.asarray(points, dtype=np.float32).tostring()

        # Publish the point cloud message
        self.point_cloud_pub.publish(cloud_msg)

    def run(self):
        """
        Continuously generates and publishes the point cloud at a specified rate.
        """
        rate = rospy.Rate(10)  # Adjust as per your requirement
        while not rospy.is_shutdown():
            self.generate_point_cloud()
            rate.sleep()

if __name__ == '__main__':
    try:
        # Create an instance of the PointCloudGenerator class
        pcg = PointCloudGenerator()
         # Run the point cloud generation and publishing loop
        pcg.run()
    except rospy.ROSInterruptException:
        pass
