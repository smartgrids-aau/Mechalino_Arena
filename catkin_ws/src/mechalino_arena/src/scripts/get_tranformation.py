#!/usr/bin/env python3
import rospy
import tf

def get_transform():
    rospy.init_node('get_table_camera_transform', anonymous=True)
    tf_listener = tf.TransformListener()

    rospy.loginfo("Waiting for the table to camera transform...")
    while not rospy.is_shutdown():
        try:
            tf_listener.waitForTransform('table', 'camera', rospy.Time(), rospy.Duration(4.0))
            (trans, rot) = tf_listener.lookupTransform('table', 'camera', rospy.Time(0))
            rospy.loginfo(f"Translation: {trans}")
            rospy.loginfo(f"Rotation: {rot}")
            break
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr(f"Error: {e}")
            continue

if __name__ == '__main__':
    try:
        get_transform()
    except rospy.ROSInterruptException:
        pass
