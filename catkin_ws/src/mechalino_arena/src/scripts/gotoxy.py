#!/usr/bin/env python
from MechalinoControl import MechalinoControl
import rospy
import tf
import math
import numpy as np

def get_current_xy(id):
    global listener
    try:
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

def calculate_movement(x0, y0, yaw_deg, x1, y1):
    # Convert yaw_deg to radians
    yaw_rad = np.radians(yaw_deg)
    
    # Calculate the difference in x and y coordinates
    dx = x1 - x0
    dy = y1 - y0
    
    # Calculate the distance to move
    distance = np.sqrt(dx**2 + dy**2)
    
    # Calculate the angle between the current direction and the target direction
    target_yaw_rad = np.arctan2(dy, dx)
    angle_diff_rad = target_yaw_rad - yaw_rad
    
    # Normalize the angle difference to the range [-pi, pi]
    angle_diff_rad = np.arctan2(np.sin(angle_diff_rad), np.cos(angle_diff_rad))
    
    # Convert the angle difference from radians to degrees
    angle_diff_deg = np.degrees(angle_diff_rad)
    
    return distance, angle_diff_deg

if __name__ == '__main__':
    rospy.init_node('gotoxy')
    
    arguments = rospy.myargv()[1:]
    if len(arguments) != 4:
        rospy.logerr("Please check arguments! You should give me robot IP, id and destination (x, y). Exiting ...")
        exit()

    ip = arguments[0]
    mechalino = MechalinoControl(ip)
    if mechalino is None:
        rospy.logerr("No bot was found at the specified IP address. Exiting ...")
        exit()

    listener = tf.TransformListener()
    rospy.sleep(1)
    id = arguments[1]
    x0,y0,yaw_deg0 = get_current_xy(id)

    x,y = np.array(arguments[2:],dtype=float)/100
    
    fw, cwr = calculate_movement(x0,y0,yaw_deg0,x,y)

    rospy.loginfo(f'current status is x={x0}, y={y0}, yaw={yaw_deg0}')

    if (fw>0.02):
        rospy.loginfo(f'distance to target is {(fw*100/24)} cm and robot should rotate {cwr} degrees clock-wise.')
        if cwr > 0:
            mechalino.rotate_cw(int(cwr))
        else:
            mechalino.rotate_ccw(int(-cwr))
        rospy.sleep(3)
        mechalino.forward(int(fw*100/24))
    else:
        rospy.loginfo(f'Almost at target location. Exiting')
