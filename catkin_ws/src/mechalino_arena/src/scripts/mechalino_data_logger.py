#!/usr/bin/env python
from MechalinoControl import MechalinoControl
import rospy
import tf
import math

def get_current_xy():
    # Look up the transformation from "table" to "mechalino_60"
    (trans, rot) = listener.lookupTransform('/table', '/mechalino_60', rospy.Time(0))

    x, y, _ = trans

    # Convert quaternion to Euler angles
    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
    # Convert yaw (z-axis rotation) to degrees
    yaw_deg = math.degrees(yaw)

    return x, y, yaw_deg

if __name__ == '__main__':
    rospy.init_node('mechalino_data_logger')
    
    mechalino_control = MechalinoControl('192.168.137.230')
    if mechalino_control is None:
        rospy.logerr("No bot was found at the specified IP address. Exiting ...")
        exit()

    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)  # 1 Hz update rate
    record = 0
    while not rospy.is_shutdown():
        try:
            print(f'logging {record}th record ...')
            record += 1

            # get init pose
            x0,y0,yaw0 = get_current_xy()

            # ask robot to move forward for 4 sec
            mechalino_control.forward(4)

            # get new position
            x1,y1,yaw1 = get_current_xy()

            # print out diff
            result = f"forward,4,{x1-x0},{y1-y0},{yaw1-yaw0}\n"
            print(result,end='')
            with open("forward_backward.csv", "a") as file:
                file.write(result)

            # get init pose
            x0,y0,yaw0 = get_current_xy()

            # ask robot to move forward for 4 sec
            mechalino_control.backward(4)

            # get new position
            x1,y1,yaw1 = get_current_xy()

            # print out diff
            result = f"backward,4,{x1-x0},{y1-y0},{yaw1-yaw0}\n"
            print(result,end='')
            with open("forward_backward.csv", "a") as file:
                file.write(result)
        
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass

        rate.sleep()
