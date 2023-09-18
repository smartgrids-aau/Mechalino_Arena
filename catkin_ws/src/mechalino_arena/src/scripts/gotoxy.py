#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_msgs
import tf.transformations
import geometry_msgs.msg
import threading
import numpy as np
import math
import sys
from MechalinoControl import MechalinoControl

# Define locks for synchronization
translation_lock = threading.Lock()
rotation_lock = threading.Lock()

# Global variables to hold the pose information
last_translation = None
last_rotation = None

def tf_callback(tf_msg):
    global last_translation, last_rotation
    
    try:
        transform = tf_buffer.lookup_transform("table", "mechalino_15", rospy.Time(0))
        
        with translation_lock:
            last_translation = transform.transform.translation
        
        with rotation_lock:
            last_rotation = transform.transform.rotation
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Transform lookup failed.")

def calculate_direction_and_distance(x0, y0, x1, y1):
    # Calculate the angle between the two points in radians
    angle = math.atan2(y1 - y0, x1 - x0)
    
    # Calculate the distance between the two points
    distance = math.sqrt((x1 - x0)**2 + (y1 - y0)**2)
    
    # Convert the angle from radians to degrees
    angle_degrees = math.degrees(angle)
    if angle_degrees < 0:
        angle_degrees += 360
    return angle_degrees, distance

def main_loop():
    arguments = rospy.myargv(argv=sys.argv)
    if len(arguments)!=3:
        print(arguments)
        rospy.loginfo("No target x,y selected. Returing home (0,0)")
        xg,yg = 0,0
    else:
        xg = float(arguments[1])/100
        yg = float(arguments[2])/100
    rate = rospy.Rate(1)  # 1 Hz loop rate
    mechalino = MechalinoControl('192.168.137.188')

    while not rospy.is_shutdown():
        with translation_lock:
            translation = last_translation
        
        with rotation_lock:
            rotation = last_rotation
        
        if rotation is None or translation is None:
            rate.sleep()
            continue
        euler_angles = tf.transformations.euler_from_quaternion(
                [rotation.x, rotation.y, rotation.z, rotation.w])
        euler_degrees = np.degrees(euler_angles)

        x0,y0 = translation.x, translation.y
        yaw0 = euler_degrees[2]
        if yaw0 < 0:
            yaw0 += 360

        face,distance = calculate_direction_and_distance(x0,y0,xg,yg)
        
        if yaw0<=90:
            alpha = 270 + yaw0
        else:
            alpha = 270 - (360 - yaw0)

        beta = 360 - face

        cw_rotation = (alpha+beta) % 360
        print(f'I am at x={x0}, y={y0} and yaw={yaw0}')
        print(f'and want to go to x={xg}, y={yg}')
        print(f'so I should rotate {cw_rotation} degrees cw and then run {distance*100} cm.')
        mechalino.rotate_cw(int(cw_rotation))
        rospy.sleep(int(cw_rotation*0.036)+2)
        mechalino.backward(100)
        rospy.sleep(distance*100/8.5)
        mechalino.stop()
        exit()
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("tf_pose_printer")
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    rospy.Subscriber("/tf", tf2_msgs.msg.TFMessage, tf_callback)
    
    main_loop()