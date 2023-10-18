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
    print('s3 [0]')
    print (msg.ranges[0])
    print('s4 [45]')
    print (msg.ranges[45])
    print('s5 [90]')
    print (msg.ranges[90])

    if msg.ranges[0] > 0.5:
        move.linear.x = 0.5
        move.angular.z = 0.0
    else:
        move.linear.x = 0.0
        move.angular.z = 0.5
    
    pub.publish(move)

rospy.init_node('obtacle_avoidance')
sub = rospy.Subscriber('/scan', LaserScan, callback)
pub = rospy.Publisher('/cmd_vel', Twist)

move = Twist()

rospy.spin()
        