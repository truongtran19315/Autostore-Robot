#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans

import cv2
import math

def callback(msg):
    img = cv2.imread('~/Autostore-Robot/Hoan/black_bg.jpg')

    # Draw circle
    center = (400, 533)
    r = 4
    circle_color = (255,0,0)
    circle_thickness = 5

    img = cv2.circle(img, center, r, circle_color, circle_thickness)

    # Draw in front of turtlebot
    img = cv2.line(img, center, (400, 527), (0, 255 , 255), 2) 

    # Draw line
    start_point = (400, 533)
    # end_point = (300,286)
    # color = (0, 255, 0)
    # thickness = 2

    for x in range(0,360):
        if msg.ranges[x] < 3.5:
            minus_x = round(math.sin(math.radians( x  ))*msg.ranges[x]*100)
            minus_y = round(math.sin(math.radians( 90 - x ))*msg.ranges[x]*100)
            color = (0, 0, 255)
            thickness = 2

        else:
            minus_x = round(math.sin(math.radians( x  ))*350)
            minus_y = round(math.sin(math.radians( 90 - x ))*350)
            color = (0, 255, 0)
            thickness = 1



        end_point = (400 - minus_x, 533 - minus_y)
        img = cv2.circle(img,end_point, 0, color, thickness)


    
    cv2.imshow('Visualization window', img)
    cv2.waitKey(10)

rospy.init_node('visualize_lidar')

rospy.Subscriber('/scan', LaserScan, callback)



rospy.spin()