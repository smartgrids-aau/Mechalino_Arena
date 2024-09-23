#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32MultiArray
from tf.transformations import euler_from_quaternion
import tf

def callback(data):
    x = data.data[0]
    y = data.data[1]

    try:
        # tf listener to get the current transformation from the 'table' frame to the robot frame
        listener = tf.TransformListener()
        listener.waitForTransform("table", "mechalino_15", rospy.Time(0), rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform("table", "mechalino_15", rospy.Time(0))

        # Rotationsmatrix in Euler-Winkel umwandeln
        roll, pitch, yaw = euler_from_quaternion(rot)

        # Formatierung der Daten in Float32MultiArray
        position_yaw_msg = Float32MultiArray()
        position_yaw_msg.data = [x, y, yaw]

        pub.publish(position_yaw_msg)  # Veröffentlichen der Nachricht

    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transformationsfehler bei der Berechnung des Yaw-Winkels")

def subscriber_node():
    global pub  # Publisher als globale Variable

    rospy.init_node('robot_position_yaw_publisher', anonymous=True)
    
    # Initialisiere den Publisher
    pub = rospy.Publisher('/robot_position_yaw', Float32MultiArray, queue_size=10)
    
    rospy.Subscriber("/pos/mechalino_15", Float32MultiArray, callback)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        subscriber_node()
    except rospy.ROSInterruptException:
        pass
