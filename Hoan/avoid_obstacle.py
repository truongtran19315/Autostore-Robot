#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    print('===================================')
    print('s1 [315]')
    print (msg.ranges[315])
    print('s2 [270]')
    print (msg.ranges[270])
    print('s2\' [-90]')
    print (msg.ranges[-90])

    print('s3 [0]')
    print (msg.ranges[0])
    print('s4 [45]')
    print (msg.ranges[45])
    print('s5 [90]')
    print (msg.ranges[90])

    right_obstacle = 0
    left_obstacle = 0
    
    for i in range(-45, 46):
        if msg.ranges[i] <= 0.4 and i <= 0:
            right_obstacle+=1
        elif msg.ranges[i] <= 0.4 and i > 0:
            left_obstacle+=1

    if left_obstacle > 0 or right_obstacle > 0:
        if right_obstacle >= left_obstacle:
            move.linear.x = 0.0
            move.angular.z = 0.5
        elif right_obstacle < left_obstacle:
            move.linear.x = 0.0
            move.angular.z = -0.5
    else:
        move.linear.x = 0.5
        move.angular.z = 0.0

    pub.publish(move)

rospy.init_node('obtacle_avoidance')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)

move = Twist()

rospy.spin()
        
