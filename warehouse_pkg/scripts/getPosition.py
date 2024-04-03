#!/usr/bin/env python3

import rospy
import sys
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def clear_terminal():
    sys.stdout.write("\033[H\033[J")  # Clear terminal

def odom_callback(data):
    # Lấy tọa độ x, y từ dữ liệu odometry
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    
    # Lấy hướng quay (yaw) từ quaternion
    orientation_q = data.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    
    # Xóa nội dung hiện tại trên terminal và in ra thông tin mới
    clear_terminal()
    rospy.loginfo("Current position:")
    rospy.loginfo("x: {}".format(x))
    rospy.loginfo("y: {}".format(y))
    rospy.loginfo("yaw: {}".format(yaw))
    rospy.loginfo("==================== \n")

def listener():
    rospy.init_node('robot_position_listener', anonymous=True)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    
    rate = rospy.Rate(0.2)  # Tần số cập nhật 0.2 Hz (mỗi 5 giây cập nhật 1 lần)
    
    while not rospy.is_shutdown():
        # Chờ cho tin nhắn mới từ chủ đề "/odom"
        rospy.wait_for_message("/odom", Odometry)
        
        # Xử lý tin nhắn khi nhận được
        odom_callback(rospy.wait_for_message("/odom", Odometry))
        
        # Chờ một khoảng thời gian trước khi tiếp tục vòng lặp
        rate.sleep()

if __name__ == '__main__':
    listener()
