#!/usr/bin/env python3

import pickle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np

# Constants
DISTANCE_SPACE = 40
ALPHA_SPACE = 8
LIDAR_LENGH_BIN = [1]

Q_TABLE_PATH = "/home/truong/Documents/Autostore-Robot/HK232/DATA/2024-04-17_V1/q_table.pkl"

class PLAYER_SETTING:
    PI = math.pi
    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 48
    X_INIT_POS = -9.5
    Y_INIT_POS = 9.5
    X_GOAL = 0.5
    Y_GOAL = 0.5

class ACTIONS:
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    TURN_BACK = 3

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0
        self.current_x = PLAYER_SETTING.X_INIT_POS
        self.current_y = PLAYER_SETTING.Y_INIT_POS
        self.current_yaw_2pi = self.convert_current_yaw_2pi()
        self.rate = rospy.Rate(10)  # Update rate 10 Hz
        self.angular_speed = 0.2  # Adjust as needed
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.lidars = []
        self.step_counter = 1
        self.q_table = None
        
        print("Initialization completed...!")

    def odom_callback(self, msg):
        # Get x, y coordinates from odometry data
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Get yaw from quaternion
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, self.current_yaw = euler_from_quaternion(orientation_list)
        
        self.current_yaw_2pi = self.convert_current_yaw_2pi()

    def lidar_callback(self, data):
        # Store lidar data
        self.lidars = [data.ranges[270], data.ranges[0], data.ranges[90], data.ranges[180]]

    def move_forward(self, distance):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0.15  # Forward speed
        start_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
        last_yaw = self.current_yaw
        self.cmd_vel_pub.publish(twist_cmd)
        while not rospy.is_shutdown():
            current_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
            current_yaw = self.current_yaw
            distance_moved = math.sqrt((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2)
            if distance_moved >= distance:
                break
            # Adjust angular speed based on current yaw error
            error_yaw = self.normalize_angle(current_yaw - last_yaw)
            twist_cmd.angular.z = -0.1 * error_yaw  # Adjust angular speed if needed
            self.cmd_vel_pub.publish(twist_cmd)
            last_yaw = current_yaw
            self.rate.sleep()
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate(self, angle, clockwise):
        twist_cmd = Twist()
        twist_cmd.angular.z = -self.angular_speed if clockwise else self.angular_speed
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
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def adjust_to_right_angle(self):
        target_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        twist_cmd = Twist()
        twist_cmd.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
        self.cmd_vel_pub.publish(twist_cmd)
        while abs(angle_diff) > 0.01:
            rospy.sleep(0.1)
            angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def stop_robot(self):
        twist_cmd = Twist()
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def get_odom_data(self):
        try:
            data = rospy.wait_for_message('/odom', Odometry, timeout=5)
            return data
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive odometry data: %s" % e)

    def load_q_table(self, filename):
        with open(filename, 'rb') as file:
            self.q_table = pickle.load(file)
            print("Q-table loaded successfully!")

    def choose_action(self, state):
        return np.argmax(self.q_table[tuple(state)])

    def observe(self):
        try:
            odom_data = rospy.wait_for_message('/odom', Odometry, timeout=5)
            lidar_data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive odometry or lidar data: %s" % e)

        goal_angle = self.angleBetweenTwoPoints(
            self.current_x, self.current_y, PLAYER_SETTING.X_GOAL, PLAYER_SETTING.Y_GOAL)
        print("Angle Goal : {} degree".format(self.convert_degree(goal_angle)))
        alpha = self.calculate_angle_difference(goal_angle)
        print("ALPHA : {} degree".format(self.convert_degree(alpha)))
        
        lidar_length_bin = LIDAR_LENGH_BIN
        lidar_length_digitized = np.digitize(self.lidars, lidar_length_bin)

        distance_goal_bin = np.linspace(
            PLAYER_SETTING.DISTANCEGOAL_MIN, PLAYER_SETTING.DISTANCEGOAL_MAX, num=DISTANCE_SPACE, endpoint=False)
        distance_goal_bin = np.delete(distance_goal_bin, 0)
        info_state_vector = []
        info_state_vector.append(np.digitize(
            self.distance_Robot_to_Goal(), distance_goal_bin))
        print("DISTANCE : {}".format(round(self.distance_Robot_to_Goal(), 2)))

        alpha_goal_bin = np.linspace(-math.pi, math.pi,
                                    num=ALPHA_SPACE, endpoint=False)
        alpha_goal_bin = np.delete(alpha_goal_bin, 0)
        info_state_vector.append(np.digitize(alpha, alpha_goal_bin))

        info_state_vector = np.array(info_state_vector)
        lidar_state_vector = np.array(lidar_length_digitized)
        return np.concatenate((info_state_vector, lidar_state_vector))

    def run(self):
        self.load_q_table(Q_TABLE_PATH)
        rospy.sleep(1)
        while not rospy.is_shutdown():
            print("----- Step {} :".format(self.step_counter))
            state = self.observe()
            print("state = {}".format(state))
            action = self.choose_action(state)
            print("action = {}".format(action))
            if action == ACTIONS.FORWARD:
                # self.adjust_to_right_angle()
                self.move_forward(1)
                self.stop_robot()
            elif action == ACTIONS.TURN_RIGHT:
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, True)
                self.stop_robot()
            elif action == ACTIONS.TURN_LEFT: 
                self.adjust_to_right_angle()
                self.rotate(math.pi/2, False)
                self.stop_robot()
            elif action == ACTIONS.TURN_BACK:
                self.adjust_to_right_angle()
                self.rotate(math.pi, True)
                self.stop_robot()
            
            self.step_counter += 1

    def angleBetweenTwoPoints(self, xPointA, yPointA, xPointB, yPointB):
        delta_x = xPointB - xPointA
        delta_y = yPointB - yPointA
        radian_angle = math.atan2(delta_y, delta_x)
        if radian_angle < 0:
            radian_angle += 2 * math.pi
        return radian_angle

    def calculate_angle_difference(self, target_angle):
        angle_difference =  abs(target_angle - self.current_yaw_2pi)
        if angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        return angle_difference
    
    def convert_current_yaw_2pi(self):
        current_yaw_2pi = self.current_yaw
        if current_yaw_2pi > -math.pi and current_yaw_2pi < 0:
            current_yaw_2pi += 2 * math.pi
        return current_yaw_2pi

    def distance_Robot_to_Goal(self):
        return abs(self.current_x - PLAYER_SETTING.X_GOAL) + abs(self.current_y - PLAYER_SETTING.Y_GOAL)
    
    def convert_degree(self, angle):
        return round(angle * 180 / math.pi, 2)



if __name__ == '__main__':
    try:
        controller = RobotController()
        controller.run()
    except rospy.ROSInterruptException:
        pass