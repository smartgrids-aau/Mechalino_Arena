#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray  # Ändere dies auf Float32MultiArray

def callback(data):
    rospy.loginfo("Received data: %s", data.data)
    if len(data.data) >= 2:
        rospy.loginfo("Position: x=%f, y=%f", data.data[0], data.data[1])
    else:
        rospy.logwarn("Received data does not contain enough elements.")

def listener():
    rospy.init_node('position_listener', anonymous=True)
    rospy.loginfo("Node position_listener started")
    rospy.Subscriber('/pos/mechalino_15', Float32MultiArray, callback)  # Ändere dies auf Float32MultiArray
    rospy.loginfo("Subscribed to /pos/mechalino_15")
    rospy.spin()

if __name__ == '__main__':
    listener()
