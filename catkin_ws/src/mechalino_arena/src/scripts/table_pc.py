#!/usr/bin/env python

import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
import numpy as np

class PointCloudGenerator:
    def __init__(self):
        rospy.init_node('point_cloud_generator')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.resolution = rospy.get_param('~resolution', 0.02)  # Default resolution
        self.point_cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=1)

    def generate_point_cloud(self):
        corners = ['corner_tl', 'corner_tr', 'corner_bl', 'corner_br']
        point_cloud = []

        # Extract corner points
        corner_points = []
        for corner in corners:
            try:
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

        self.publish_point_cloud(point_cloud)


    def publish_point_cloud(self, points):
        header = rospy.Header()
        header.frame_id = 'table'

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

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

        self.point_cloud_pub.publish(cloud_msg)

    def run(self):
        rate = rospy.Rate(10)  # Adjust as per your requirement
        while not rospy.is_shutdown():
            self.generate_point_cloud()
            rate.sleep()

if __name__ == '__main__':
    try:
        pcg = PointCloudGenerator()
        pcg.run()
    except rospy.ROSInterruptException:
        pass
