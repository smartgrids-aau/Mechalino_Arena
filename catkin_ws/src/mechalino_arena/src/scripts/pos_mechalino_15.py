#!/usr/bin/env python

# This script is a subscriber node that listens to the topic "/pos/mechalino_15" and logs the received data.
# used for calculating the speed of the robot

import rospy
from std_msgs.msg import Float32MultiArray
import requests



x,y,time = None, None, None # global access

def callback(data):
    global x,y,time
    x = data.data[0]
    y = data.data[1]
    time = rospy.get_time()
    


def subscriber_node():
    rospy.init_node('pos_mechalino_15', anonymous=True)

    # robot contact data
    server_ip = "192.168.137.230"
    server_port = 80
    forward = f"http://{server_ip}:{server_port}/?data=M 100 1000\r"
    backward = f"http://{server_ip}:{server_port}/?data=M -100 1000\r"

    # csv path to desktop
    csv_path = '/home/khalil/Desktop/pos_mechalino_15.csv'
    # header
    csv_data = [['init_x', 'init_y', 'new_x', 'new_y', 'displacement', 'time_diff', 'speed']]

    rospy.Subscriber("/pos/mechalino_15", Float32MultiArray, callback)
    # wait for the first message
    while x is None or y is None:
        rospy.sleep(0.1)

    N = 10
    for i in range(N):
        init_x = x
        init_y = y
        init_time = rospy.get_time()
        # if ros is not running exit
        if rospy.is_shutdown():
            break
        requests.get(forward)
        rospy.sleep(5) # wait for the robot to move
        new_x = x
        new_y = y
        new_time = time
        dx = new_x - init_x
        dy = new_y - init_y
        displacement = (dx**2 + dy**2)**0.5
        time_diff = new_time - init_time
        speed = displacement / time_diff
        csv_data.append([init_x, init_y, new_x, new_y, displacement, time_diff, speed])
        # return the robot
        requests.get(backward)
        rospy.sleep(5) # wait for the robot to move
        print(f"Done {i+1}/{N}")
    print('Wrriting to csv...')
    # write to csv
    with open(csv_path, 'a') as f:
        for row in csv_data:
            f.write(','.join([str(x) for x in row]) + '\n')
    print('Done! Exiting...')
if __name__ == '__main__':
    subscriber_node()
