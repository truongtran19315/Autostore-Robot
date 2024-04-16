#!/usr/bin/env python3

import pickle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np

DISTANCE_SPACE = 40
ALPHA_SPACE = 8
LIDAR_LENGH_BIN = [1]

Q_TABLE_PATH = "/home/truong/Documents/DATA/q_table.pkl"

class PLAYER_SETTING:
    PI = math.pi
    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 48
    X_INIT_POS = 9.5
    Y_INIT_POS = 7.5
    X_GOAL = -2.5
    Y_GOAL = 6.5

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0
        self.current_x = PLAYER_SETTING.X_INIT_POS
        self.current_y = PLAYER_SETTING.Y_INIT_POS
        self.current_yaw_Ox = self.convert_current_yaw_to_Ox()
        self.rate = rospy.Rate(10)  # Tần số cập nhật 10 Hz
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.lidars = []
        self.step_counter = 1
        
        print("done init...!")

    def odom_callback(self, msg):
        # Lấy tọa độ x, y từ dữ liệu odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Lấy hướng quay (yaw) từ quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        #! print("topic /odom ")

    def lidar_callback(self, data):
        # self.lidars = self.lidars = [data.ranges[270], data.ranges[0], data.ranges[90]]
        self.lidars = self.lidars = [data.ranges[270], data.ranges[0], data.ranges[90], data.ranges[180]]
        #! print("topic /lidar")

    def move_forward(self, distance):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.15  #! Tốc độ di chuyển về phía trước
        start_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
        last_yaw = self.current_yaw
        self.cmd_vel_pub.publish(twist_cmd)

        while not rospy.is_shutdown():
            current_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
            current_yaw = self.current_yaw
            distance_moved = math.sqrt((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2)
            if distance_moved >= distance:
                break

            # Điều chỉnh tốc độ góc dựa trên sai số góc hiện tại
            error_yaw = self.normalize_angle(current_yaw - last_yaw)
            twist_cmd.angular.z = -0.1 * error_yaw  # Điều chỉnh tốc độ góc nếu cần
            self.cmd_vel_pub.publish(twist_cmd)
            last_yaw = current_yaw
            self.rate.sleep()

        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
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

    def load_q_table(self, filename):
        with open(filename, 'rb') as file:
            self.q_table = pickle.load(file)
            print("Load Q-table complement!")

    def choose_action(self, state):
        # print("q[state] = {}".format(self.q_table[tuple(state)]))
        # print("---end q-state---")
        return np.argmax(self.q_table[tuple(state)])

    def observe(self):
        goal_angle = self.angleBetweenTwoPoints(
            self.current_x, self.current_y, PLAYER_SETTING.X_GOAL, PLAYER_SETTING.Y_GOAL)
        
        alpha = self.calculate_angle_difference(goal_angle)
    
        #! convert lidar length from robot to obstacle
        lidarLength_bin = LIDAR_LENGH_BIN
        lidarLength_digitized = np.digitize(
            self.lidars, lidarLength_bin)

        #! convert distance from robot to target
        distanceGoal_bin = np.linspace(
            PLAYER_SETTING.DISTANCEGOAL_MIN, PLAYER_SETTING.DISTANCEGOAL_MAX, num=DISTANCE_SPACE, endpoint=False)
        distanceGoal_bin = np.delete(distanceGoal_bin, 0)
        infoStateVector = []
        infoStateVector.append(np.digitize(
            self.distance_Robot_to_Goal(), distanceGoal_bin))

        #! convert angular deviation between robot and target
        alphaGoal_bin = np.linspace(-math.pi, math.pi,
                                    num=ALPHA_SPACE, endpoint=False)
        alphaGoal_bin = np.delete(alphaGoal_bin, 0)
        infoStateVector.append(np.digitize(alpha, alphaGoal_bin))

        infoStateVector = np.array(infoStateVector)
        lidarStateVector = np.array(lidarLength_digitized)
        #! distance, alpha, 0, 90, 180
        return np.concatenate((infoStateVector, lidarStateVector))

    def run(self):
        self.load_q_table(Q_TABLE_PATH)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            state = self.observe()
            print("Step {}:".format(self.step_counter))
            print("state = {}".format(state))
            action = self.choose_action(state)
            print("action = {}".format(action))
            if action == 0:
                self.adjust_to_right_angle()
                self.move_forward(1)
            elif action == 1:
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, True)
            elif action == 2:
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, False)
            elif action == 3:
                self.adjust_to_right_angle()
                self.rotate(math.pi, True)
            elif action == 4:
                pass
            # Không cần hành động cho trường hợp action == 4 vì đó là 'không làm gì cả'
            self.step_counter += 1

    def angleBetweenTwoPoints(self, xPointA, yPointA, xPointB, yPointB):
        delta_x = xPointB - xPointA
        delta_y = yPointB - yPointA
        radian_angle = math.atan2(delta_y, delta_x)
        if radian_angle < 0:
            radian_angle += 2 * math.pi
        return radian_angle

    def calculate_angle_difference(self, target_angle):
        angle_difference =  abs(target_angle - self.current_yaw_Ox)
        if angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        return angle_difference
        
    def convert_current_yaw_to_Ox(self):
        current_yaw_Ox = self.current_yaw + math.pi/2
        if current_yaw_Ox > -math.pi/2 and current_yaw_Ox < 0:
            current_yaw_Ox += 2 * math.pi
        return current_yaw_Ox
    
    def distance_Robot_to_Goal(self):
        return math.sqrt((PLAYER_SETTING.X_GOAL - self.current_x)**2 + (PLAYER_SETTING.Y_GOAL - self.current_y)**2)


if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


# =============================================
# Giá trị lớn nhất của yaw: 3.137074060411566
# Giá trị nhỏ nhất của yaw: -3.139431611332625
