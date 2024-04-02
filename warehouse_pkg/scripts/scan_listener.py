#!/usr/bin/env python3

import sys
import rospy
from sensor_msgs.msg import LaserScan

def clear_terminal():
    sys.stdout.write("\033[F\033[K")  # Move the cursor up one line and clear that line

def scan_callback(data):
    ranges = data.ranges
    
    # Xóa nội dung hiện tại trên terminal
    clear_terminal()
    
    # Ghi ra thông điệp mới
    rospy.loginfo("Laser scan ranges:")
    rospy.loginfo("front: {}".format(ranges[0]))            #! front: 0
    rospy.loginfo("left : {}".format(ranges[90]))           #! left : 90
    rospy.loginfo("rear : {}".format(ranges[180]))          #! rear : 180
    rospy.loginfo("right: {}".format(ranges[270]))          #! right: 270
    rospy.loginfo("=========================== \n")
    
def listener():
    rospy.init_node('scan_listener', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    
    # Tạo một bộ hẹn giờ với chu kỳ là 5 giây
    rospy.Timer(rospy.Duration(5), timer_callback)
    
    rospy.spin()

def timer_callback(event):
    rospy.loginfo("Reading /scan...")

if __name__ == '__main__':
    listener()
