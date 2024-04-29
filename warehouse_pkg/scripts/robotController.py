#!/usr/bin/env python3

import pickle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import math
import numpy as np

Q_TABLE_PATH = "/home/truong/Documents/Autostore-Robot/HK232/DATA/2024-04-27_V1/q_table.pkl"

class SPACE:
    LIDAR_LENGTH_SEGMENT = [0.4]
    DISTANCE_SPACE = 175
    ALPHA_SPACE = 9
    REGION_LIDAR_SPAGE = 3
    SECTIONS_LIDARSPACE = 3 
    ACTION_SPACE = 3


class PLAYER_SETTING:   
    #! init robot point
    X_INIT_POS = -9.9
    Y_INIT_POS = 6.3
    #! Goal point
    X_GOAL = 2.4
    Y_GOAL = 6
    
    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 35
    ALPHAGOAL_MIN = 0
    ALPHAGOAL_MAX = 2 * math.pi    
    # lidar ray
    CASTED_RAYS = 9
    STEP_ANGLE = 180 / (CASTED_RAYS - 1)

class ACTIONS:
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_yaw = 0
        self.current_x = PLAYER_SETTING.X_INIT_POS
        self.current_y = PLAYER_SETTING.Y_INIT_POS
        self.current_yaw_2pi = self.convert_current_yaw_2pi()
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.lidars = []
        self.step_counter = 1
        self.q_table = None
        self.load_q_table(Q_TABLE_PATH)
        
        self.angular_rotate_speed = 0.4    #! Speed rotate robot
        self.angular_adjust_right_speed = 0.08
        self.distance_forward = 0.18        #! move forward robot 20 cm (1 step)
        self.move_forward_speed = 0.10
        
        # Timer with 10 Hz
        self.timer = rospy.Timer(rospy.Duration(0.1), self.control_loop)

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
        lidar_indices = np.round((270 + np.arange(0, PLAYER_SETTING.CASTED_RAYS) * PLAYER_SETTING.STEP_ANGLE)) % 360
        lidar_indices = lidar_indices.astype(int)  
        lidar_data = [data.ranges[i] for i in lidar_indices]
        self.lidars = np.array(lidar_data)

    def move_forward(self, distance):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.move_forward_speed  
        start_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
        last_yaw = self.current_yaw
        self.cmd_vel_pub.publish(twist_cmd)
        while not rospy.is_shutdown():
            current_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
            current_yaw = self.current_yaw
            distance_moved = math.sqrt((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2)
            if distance_moved >= distance:
                break
            error_yaw = self.normalize_angle(current_yaw - last_yaw)
            twist_cmd.angular.z = -0.1 * error_yaw  
            self.cmd_vel_pub.publish(twist_cmd)
            last_yaw = current_yaw
            rospy.sleep(0.1)
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def rotate(self, angle, clockwise):
        twist_cmd = Twist()
        twist_cmd.angular.z = -self.angular_rotate_speed if clockwise else self.angular_rotate_speed
        last_yaw = self.current_yaw
        angle_moved = 0.0
        self.cmd_vel_pub.publish(twist_cmd)
        while not rospy.is_shutdown() and abs(angle_moved) < abs(angle):
            rospy.sleep(0.1)
            delta_yaw = self.normalize_angle(self.current_yaw - last_yaw)
            angle_moved += delta_yaw
            last_yaw = self.current_yaw
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def adjust_to_right_angle(self, speed_rotate=None, target_yaw=None):
        if speed_rotate is None:
            speed_rotate = self.angular_adjust_right_speed
        if target_yaw is None:
            target_yaw = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw)
        twist_cmd = Twist()
        twist_cmd.angular.z = speed_rotate if angle_diff > 0 else -speed_rotate
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
        alpha = self.calculate_angle_difference(goal_angle)
        
        print("current yaw = {} = {} ".format(self.current_yaw, self.convert_degree(self.current_yaw)))
        print("Current Angle : {}, Angle Goal : {}".format(self.convert_degree(self.current_yaw_2pi), self.convert_degree(goal_angle)))
        print("==> ALPHA : {} degree".format(self.convert_degree(alpha)))
        print("DISTANCE : {}".format(round(self.distance_Robot_to_Goal(), 2)))
        
        lidars = np.reshape(self.lidars, (SPACE.REGION_LIDAR_SPAGE,SPACE.SECTIONS_LIDARSPACE))
        lidars_RegionSelected = np.min(lidars, axis=1)   
        lidarLength_digitized = np.digitize(lidars_RegionSelected, SPACE.LIDAR_LENGTH_SEGMENT)

        distanceGoal_bin = np.linspace(
            PLAYER_SETTING.DISTANCEGOAL_MIN, PLAYER_SETTING.DISTANCEGOAL_MAX, 
            num=SPACE.DISTANCE_SPACE, endpoint=False)[1:]

        alphaGoal_bin = np.linspace(
            PLAYER_SETTING.ALPHAGOAL_MIN, PLAYER_SETTING.ALPHAGOAL_MAX,
            num=SPACE.ALPHA_SPACE, endpoint=False)[1:]

        infoStateVector = np.digitize(self.distance_Robot_to_Goal(), distanceGoal_bin)
        infoStateVector = np.append(infoStateVector, np.digitize(alpha, alphaGoal_bin))

        return np.concatenate((infoStateVector, lidarLength_digitized))

    def control_loop(self, event):
        state = self.observe()
        print("state = {}".format(state))
        action = self.choose_action(state)
        print("---------- Step {} ---------------- :".format(self.step_counter))
        print("action = {}".format(action))
        if action == ACTIONS.FORWARD:
            self.adjust_to_right_angle(0.08)
            self.move_forward(self.distance_forward)
            self.stop_robot()
        elif action == ACTIONS.TURN_RIGHT:
            self.adjust_to_right_angle()
            self.rotate(math.pi/2, True)
            self.stop_robot()
        elif action == ACTIONS.TURN_LEFT: 
            self.adjust_to_right_angle()
            self.rotate(math.pi/2, False)
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
        return ((self.current_yaw_2pi - target_angle)) % (2 * math.pi)
    
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
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
