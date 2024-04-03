#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0
        self.current_x = 0
        self.current_y = 0
        self.rate = rospy.Rate(10)  # Tần số cập nhật 10 Hz

    def odom_callback(self, msg):
        # Lấy thông tin về vị trí và hướng của robot từ dữ liệu odometry
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def move_forward(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.5  # Tốc độ tuyến tính: 0.5 m/s
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(1)  # Di chuyển trong 1 giây
        twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate_left(self):
        twist_cmd = Twist()
        twist_cmd.angular.z = math.pi/2  # Quay sang trái: Pi/2 rad
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(1)  # Quay trong 1 giây
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate_right(self):
        twist_cmd = Twist()
        twist_cmd.angular.z = -math.pi/2  # Quay sang phải: -Pi/2 rad
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(1)  # Quay trong 1 giây
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate_back(self):
        twist_cmd = Twist()
        twist_cmd.angular.z = math.pi  # Quay ra sau: Pi rad
        self.cmd_vel_pub.publish(twist_cmd)
        rospy.sleep(1)  # Quay trong 1 giây
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def adjust_to_right_angle(self):        #! Bug
        # Tính toán góc yaw gần nhất là bội số của π/2
        nearest_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
        # Tính toán góc cần điều chỉnh
        angle_diff = nearest_yaw - self.current_yaw
        # Quay robot sang trái hoặc phải để đảm bảo là nó ở vị trí vuông góc
        if nearest_yaw > 0: # góc quay nằm ở 
            if angle_diff > 0:
                while abs(angle_diff) > 0.01:  # Đảm bảo sai số nhỏ
                    self.rotate_left()
                    angle_diff = nearest_yaw - self.current_yaw
            elif angle_diff < 0:
                while abs(angle_diff) > 0.01:  # Đảm bảo sai số nhỏ
                    self.rotate_right()
                    angle_diff = nearest_yaw - self.current_yaw
                    
        elif nearest_yaw < 0:
            if angle_diff > 0:
                while abs(angle_diff) > 0.01:  # Đảm bảo sai số nhỏ
                    self.rotate_right()
                    angle_diff = nearest_yaw - self.current_yaw
            elif angle_diff < 0:
                while abs(angle_diff) > 0.01:  # Đảm bảo sai số nhỏ
                    self.rotate_left()
                    angle_diff = nearest_yaw - self.current_yaw


    def run(self):
        rospy.sleep(1)  # Chờ nhận được dữ liệu odometry
        while not rospy.is_shutdown():
            action = input("Nhập hành động (w: đi tới, a: quay trái, d: quay phải, x: quay ra sau): ")
            if action == 'w':
                # Đảm bảo robot ở vị trí vuông góc trước khi di chuyển
                self.adjust_to_right_angle()
                # Tiến hành di chuyển
                target_x = self.current_x + math.cos(self.current_yaw)
                target_y = self.current_y + math.sin(self.current_yaw)
                while math.sqrt((self.current_x - target_x)**2 + (self.current_y - target_y)**2) < 1:
                    self.move_forward()
                    self.rate.sleep()
            elif action == 'a':
                # Đảm bảo robot ở vị trí vuông góc trước khi quay trái
                self.adjust_to_right_angle()
                self.rotate_left()
            elif action == 'd':
                # Đảm bảo robot ở vị trí vuông góc trước khi quay phải
                self.adjust_to_right_angle()
                self.rotate_right()
            elif action == 'x':
                # Đảm bảo robot ở vị trí vuông góc trước khi quay ra sau
                self.adjust_to_right_angle()
                self.rotate_back()
            else:
                print("Hành động không hợp lệ!")

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
