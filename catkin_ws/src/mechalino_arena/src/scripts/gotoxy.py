#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import math
import matplotlib.pyplot as plt
import numpy as np
import MechalinoControl

# Initialize lists to store the historical X, Y, and yaw values
x_history = []
y_history = []
yaw_history = []

# Initialize the figure and axes for the plot
plt.figure(figsize=(10, 5))
plt.ion()  # Turn on interactive mode for dynamic updating

def tf_listener():
    rospy.init_node('gotoxy')
    
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)  # Update rate in Hz
    
    goal_publisher = rospy.Publisher('/goal', Marker, queue_size=1)
    xg = 1.25
    yg = 0.529
    
    mechalino = MechalinoControl.MechalinoControl('192.168.137.175')
    while not rospy.is_shutdown():
        try:
            # Lookup the transform of "mechalino_15" in the "table" frame
            (trans, rot) = listener.lookupTransform('corner_tl', 'mechalino_15', rospy.Time(0))

            # Convert quaternion to Euler angles (roll, pitch, yaw)
            euler_angles = tf.transformations.euler_from_quaternion(rot)

            # Extract X, Y, and yaw
            x, y, _ = trans
            yaw = 180 - math.degrees(euler_angles[2])
        

            # Append the values to the history lists
            x_history.append(x * 100)  # Convert to centimeters
            y_history.append(y * 100)  # Convert to centimeters
            yaw_history.append(yaw)

            md = np.sqrt((x-xg)**2 + (y-yg)**2)

            stdeg = np.round((np.degrees(np.arctan2(xg - x,yg - y)) - yaw + 180) % 360 - 180)
            # Print the position and orientation
            rospy.loginfo(f"x={x*100:.1f}cm y={y*100:.1f}cm yaw={yaw}° dis={md*100:.1f} turn={stdeg}")
            
            if(stdeg<5):
                print('asssssssss',int(stdeg))
                mechalino.rotate_cw(int(-stdeg))
                rospy.sleep(0.5+-stdeg*0.05)
            elif(stdeg>5):
                print('a22sssssssss',int(stdeg))
                mechalino.rotate_ccw(int(stdeg))
                rospy.sleep(0.5+stdeg*0.05)
            
            marker = Marker(type=Marker.CUBE, action=Marker.ADD,
                        pose=Marker().pose, scale=Marker().scale,
                        color=Marker().color)
            marker.header.frame_id = "corner_tl"  # Set the frame ID you want to use
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xg, yg, 0.0
            marker.scale.x, marker.scale.y, marker.scale.z = 0.05, 0.05, 0.05
            marker.color.a, marker.color.r = 1.0, 1.0

            goal_publisher.publish(marker)

            if(md*100>3 and np.abs(stdeg)<10):
                mechalino.forward(100)

            

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception")

        rate.sleep()

if __name__ == '__main__':
    try:
        tf_listener()
    except rospy.ROSInterruptException:
        pass
