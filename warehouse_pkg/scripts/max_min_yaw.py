#!/usr/bin/env python3

import rospy
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Biến toàn cục để lưu trữ giá trị lớn nhất và nhỏ nhất của yaw
max_yaw = -float('inf')
min_yaw = float('inf')

def odom_callback(data):
    global max_yaw, min_yaw
    # Lấy góc yaw từ vị trí hiện tại của robot
    quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                  data.pose.pose.orientation.z, data.pose.pose.orientation.w]
    _, _, yaw = euler_from_quaternion(quaternion)
    # Cập nhật giá trị lớn nhất và nhỏ nhất của yaw
    max_yaw = max(max_yaw, yaw)
    min_yaw = min(min_yaw, yaw)

def clear_terminal():
    sys.stdout.write("\033[H\033[J")  # Clear terminal

def main():
    global max_yaw, min_yaw
    rospy.init_node('yaw_min_max_node', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rate = rospy.Rate(10)  # Thường là 10 Hz cho các ứng dụng thời gian thực
    while not rospy.is_shutdown():
        clear_terminal()
        print("Giá trị lớn nhất của yaw:", max_yaw)
        print("Giá trị nhỏ nhất của yaw:", min_yaw)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


# =============================================
# Giá trị lớn nhất của yaw: 3.137074060411566
# Giá trị nhỏ nhất của yaw: -3.139431611332625
