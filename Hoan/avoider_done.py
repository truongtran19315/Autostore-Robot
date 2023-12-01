#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans
from Avoider_v1 import AvoiderV1

def main():

    rospy.init_node('obtacle_avoidance')
    avoider = AvoiderV1()

    rospy.Subscriber('/scan', LaserScan, avoider.indentify_scan_msg)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
        # avoider.print_scan()
        # avoider.print_time()
        # if len(min_laser_scan_list) != 0:
        #     pub.publish(avoid_obstacle())
        # avoider.read_qtable()
        # pub.publish(avoider.avoid())

        pub.publish(avoider.read_qtable())
        avoider.update_robot_position()

        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass