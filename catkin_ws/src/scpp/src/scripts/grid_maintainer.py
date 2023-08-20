#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
import tf
import numpy as np

def create_marker(id, x, y):
    marker = Marker()
    marker.header.frame_id = "grid"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.id = id
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.0
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    return marker

def get_matching_tf_position(tf_listener, prefix):
    try:
        now = rospy.Time(0)
        tf_listener.waitForTransform("grid", prefix, now, rospy.Duration(10.0))
        (trans, rot) = tf_listener.lookupTransform("grid", prefix, now)
        position = trans
        return position[:2]
    except Exception as e:
        rospy.logerr('Corner not found!' + str(e))
        return [None, None]

def cross_product(p1, p2):
    return p1[0] * p2[1] - p1[1] * p2[0]

def dot_product(p1, p2):
    return p1[0] * p2[0] + p1[1] * p2[1]

def subtract(p1, p2):
    return (p1[0] - p2[0], p1[1] - p2[1])

def point_in_convex_polygon(polygon, point):
    n = len(polygon)
    if n < 3:
        return False  # Not a polygon

    for i in range(n):
        edge = subtract(polygon[(i + 1) % n], polygon[i])
        to_point = subtract(point, polygon[i])
        cross = cross_product(edge, to_point)
        dot = dot_product(edge, to_point)

        if cross < 0 or dot < 0:
            return False

    return True

def grid_maintainer():
    rospy.init_node("grid_maintainer", anonymous=True)

    listener = tf.TransformListener()
    # rospy.sleep(5) # wait for corners being published

    M =  int(rospy.get_param('~M'))
    N =  int(rospy.get_param('~N'))

    pub = rospy.Publisher("marker_array", MarkerArray, queue_size=10)
    rate = rospy.Rate(1/60)  # 10 Hz

    while not rospy.is_shutdown():
        
        tl = get_matching_tf_position(listener,'corner_tl')
        tr = get_matching_tf_position(listener,'corner_tr')
        br = get_matching_tf_position(listener,'corner_br')
        bl = get_matching_tf_position(listener,'corner_bl')
        # print(f'tl={tl},tr={tr},bl={bl},br={br}')
        limit_x = np.min([bl[0],tl[0]])
        limit_y = np.max([br[1],bl[1]])
        
        sx,sy = tr

        ex = sx + np.max([bl[0]-br[0],tl[0]-tr[0]]) + 0.12
        ey = sy + np.max([br[1]-tr[1],bl[1]-tl[1]]) + 0.12
        
        marker_array = MarkerArray()

        dx = (ex - sx) / (N - 1)
        dy = (ey - sy) / (M - 1)
        
        id = 0
        for i in range(M):
            for j in range(N):
                x = sx + j * dx
                y = sy + i * dy
                marker = create_marker(id, x, y)
                marker_array.markers.append(marker)
                id += 1
                if not point_in_convex_polygon([tr,tl,bl,br],(x,y)):
                    marker.color.a = 1.0
                    marker.color.r = 0.5
                    marker.color.g = 0.5
                    marker.color.b = 0.5

        pub.publish(marker_array)
        rate.sleep()

if __name__ == "__main__":
    try:
        grid_maintainer()
    except rospy.ROSInterruptException:
        pass
