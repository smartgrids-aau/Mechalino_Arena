#!/usr/bin/env python
from MechalinoControl import MechalinoControl
import rospy
import tf
import math
import numpy as np

def get_current_xy(id):
    global listener
    try:
        rospy.sleep(1)
        # Look up the transformation from "table" to "mechalino_60"
        (trans, rot) = listener.lookupTransform('/table', f'/mechalino_{id}', rospy.Time(0))

        x, y, _ = trans

        # Convert quaternion to Euler angles
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
        # Convert yaw (z-axis rotation) to degrees
        yaw_deg = math.degrees(yaw)

        return x, y, yaw_deg
    except Exception as e:
        rospy.logerr('Could not determine robot location! Exiting...')
        rospy.logerr(str(e))
        exit()

if __name__ == '__main__':
    rospy.init_node('face0')
    
    arguments = rospy.myargv()[1:]
    if len(arguments) != 2:
        rospy.logerr("Please check arguments! You should give me robot IP, id. Exiting ...")
        exit()

    ip = arguments[0]
    mechalino = MechalinoControl(ip)
    if mechalino is None:
        rospy.logerr("No bot was found at the specified IP address. Exiting ...")
        exit()

    listener = tf.TransformListener()
    id = arguments[1]

    while not rospy.is_shutdown():
        _,_,yaw_deg0 = get_current_xy(id)
        if yaw_deg0 < 0:
            cyaw_deg0 = int(yaw_deg0+360.0)
        else:
            cyaw_deg0 = int(yaw_deg0)

        if cyaw_deg0 < 7 or cyaw_deg0 > 353:
            rospy.loginfo(f'best we could get was {yaw_deg0} degrees. Exisiting ...')
            exit()
        else:
            rospy.loginfo(f'faced {cyaw_deg0}! rotating ...')
            if cyaw_deg0 < 180:
                mechalino.rotate_cw(int(cyaw_deg0))
            else:
                mechalino.rotate_ccw(int(360-cyaw_deg0))