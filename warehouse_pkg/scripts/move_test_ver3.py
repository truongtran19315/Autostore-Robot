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
        self.rate = rospy.Rate(10)  # Tần số cập nhật 10 Hz

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)

    def move_forward(self, distance):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.5
        start_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
        self.cmd_vel_pub.publish(twist_cmd)
        
        while not rospy.is_shutdown():
            current_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
            if math.sqrt((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2) >= distance:
                break
            self.rate.sleep()
        
        twist_cmd.linear.x = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate(self, angle, clockwise):
        twist_cmd = Twist()
        twist_cmd.angular.z = -0.1 if clockwise else 0.1
        last_yaw = self.current_yaw
        angle_moved = 0.0

        self.cmd_vel_pub.publish(twist_cmd)
        while not rospy.is_shutdown() and abs(angle_moved) < abs(angle):
            self.rate.sleep()
            delta_yaw = self.normalize_angle(self.current_yaw - last_yaw)
            angle_moved += delta_yaw
            last_yaw = self.current_yaw
        
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def adjust_to_right_angle(self):
        target_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
        angle_diff = target_yaw - self.current_yaw

        # Điều chỉnh tốc độ quay dựa trên góc cần quay
        twist_cmd = Twist()
        if angle_diff > 0:
            twist_cmd.angular.z = 0.1
        else:
            twist_cmd.angular.z = -0.1

        # Quay robot cho đến khi đạt góc mong muốn
        while abs(angle_diff) > 0.01:
            self.cmd_vel_pub.publish(twist_cmd)
            rospy.sleep(0.1)  # Đợi một chút để robot cập nhật vị trí mới
            self.odom_callback(self.get_odom_data())  # Cập nhật self.current_yaw sau mỗi lần quay
            angle_diff = target_yaw - self.current_yaw

        # Dừng robot
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        
    def get_odom_data(self):
        try:
            data = rospy.wait_for_message('/odom', Odometry, timeout=5)
            return data
        except rospy.ROSException as e:
            rospy.logerr("Không thể nhận dữ liệu odometry: %s" % e)

    def run(self):
        rospy.sleep(1)
        while not rospy.is_shutdown():
            action = input("Nhập hành động (w: đi tới, a: quay trái, d: quay phải, s: quay ra sau): ")
            if action == 'w':
                self.move_forward(1)
            elif action == 'a':
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, False)
            elif action == 'd':
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, True)
            elif action == 's':
                self.adjust_to_right_angle()
                self.rotate(math.pi, True)
            else:
                print("Hành động không hợp lệ!")

if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
