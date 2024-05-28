#!/usr/bin/env python3

import pickle
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import numpy as np
import math
import time
from const import *

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
        
        self.angular_rotate_speed = PARAM_ROBOT.ROTATE_SPEED    
        self.angular_adjust_right_speed = PARAM_ROBOT.ADJUST_ROTATE_SPEED
        self.distance_forward = PARAM_ROBOT.DIATANCE_FORWARD        
        self.move_forward_speed = PARAM_ROBOT.FORWARD_SPEED

        self.collision_count = 0
        self.total_lidar_distance = 0.0
        self.lidar_samples = 0
        self.start_time = 0
        self.end_time = 0
        self.total_distance_traveled = 0.0
        
        self.last_actions_record = []
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
        
        # Update total distance to obstacles
        min_distance = np.min(self.lidars)
        self.total_lidar_distance += min_distance
        self.lidar_samples += 1
        
        # Detect collisions
        min_distance = np.min(self.lidars)
        if min_distance < COLLISION_THRESHOLD:
            self.collision_count += 1

    def move_forward(self, distance, timeout=ACTION_TIMEOUT):
        twist_cmd = Twist()
        twist_cmd.linear.x = self.move_forward_speed  
        start_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
        last_yaw = self.current_yaw
        self.cmd_vel_pub.publish(twist_cmd)
        start_time = time.time()
        while not rospy.is_shutdown():
            current_position = rospy.wait_for_message('/odom', Odometry).pose.pose.position
            current_yaw = self.current_yaw
            distance_moved = math.sqrt((current_position.x - start_position.x)**2 + (current_position.y - start_position.y)**2)
            if distance_moved >= distance or (time.time() - start_time) > timeout:
                
                break
            error_yaw = self.normalize_angle(current_yaw - last_yaw)
            twist_cmd.angular.z = -0.1 * error_yaw  
            self.cmd_vel_pub.publish(twist_cmd)
            last_yaw = current_yaw
            rospy.sleep(0.1)
        twist_cmd.linear.x = 0
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
        self.total_distance_traveled += distance_moved  # Add this line to update the total distance traveled


    def rotate(self, angle, clockwise, timeout=ACTION_TIMEOUT):
        twist_cmd = Twist()
        twist_cmd.angular.z = -self.angular_rotate_speed if clockwise else self.angular_rotate_speed
        last_yaw = self.current_yaw
        angle_moved = 0.0
        self.cmd_vel_pub.publish(twist_cmd)
        start_time = time.time()
        while not rospy.is_shutdown() and abs(angle_moved) < abs(angle):
            if (time.time() - start_time) > timeout:
                self.handle_timeout()
                break
            rospy.sleep(0.1)
            delta_yaw = self.normalize_angle(self.current_yaw - last_yaw)
            angle_moved += delta_yaw
            last_yaw = self.current_yaw
        twist_cmd.angular.z = 0
        self.cmd_vel_pub.publish(twist_cmd)
    
    def handle_timeout(self):
        print("Action timeout. Rotating right twice.")
        self.rotate(math.pi / 2, True, timeout=ACTION_TIMEOUT)
        self.rotate(math.pi / 2, True, timeout=ACTION_TIMEOUT)


    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def adjust_to_right_angle(self, speed_rotate=None, target_yaw=None):
        if speed_rotate is None:
            speed_rotate = self.angular_adjust_right_speed
        if target_yaw is None:
            target_yaw = round(self.current_yaw_2pi / (math.pi / 2)) * (math.pi / 2)
        twist_cmd = Twist()
        # Determine the direction of rotation based on the sign of angle_diff
        angle_diff = self.normalize_angle(target_yaw - self.current_yaw_2pi)
        twist_cmd.angular.z = speed_rotate if angle_diff > 0 else -speed_rotate
        self.cmd_vel_pub.publish(twist_cmd)
        # Adjust the robot until the angle difference is within the threshold
        while abs(angle_diff) > 0.01:
            rospy.sleep(0.1)
            angle_diff = self.normalize_angle(target_yaw - self.current_yaw_2pi)
            # Update the direction of rotation if necessary
            twist_cmd.angular.z = speed_rotate if angle_diff > 0 else -speed_rotate
            self.cmd_vel_pub.publish(twist_cmd)
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
        # In ra giá trị của last_actions_record
        # print("Last actions record before choosing action:", self.last_actions_record)
        
        if len(self.last_actions_record) == 2 and \
            ((ACTIONS.TURN_RIGHT in self.last_actions_record) or (ACTIONS.TURN_LEFT in self.last_actions_record)) \
            and not (ACTIONS.FORWARD in self.last_actions_record):
            
            print("DI THANG O DAYYYYYY!")
            action = ACTIONS.FORWARD  
        else:
            action = np.argmax(self.q_table[tuple(state)])
        
        # In ra hành động đã chọn
        # print("Chosen action:", action)
        
        self.last_actions_record.append(action)
        if len(self.last_actions_record) > 2:
            self.last_actions_record.pop(0)
        
        # In ra giá trị của last_actions_record sau khi cập nhật
        # print("Last actions record after choosing action:", self.last_actions_record)
        
        return action


    def observe(self):
        try:
            odom_data = rospy.wait_for_message('/odom', Odometry, timeout=5)
            lidar_data = rospy.wait_for_message('/scan', LaserScan, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr("Failed to receive odometry or lidar data: %s" % e)

        goal_angle = self.angleBetweenTwoPoints(
            self.current_x, self.current_y, PLAYER_SETTING.X_GOAL, PLAYER_SETTING.Y_GOAL)
        alpha = self.convert_alpha_pi(goal_angle)
        
        print("current yaw = {} = {} ".format(self.current_yaw, self.convert_degree(self.current_yaw)))
        print("Current Angle : {}, Angle Goal : {}".format(self.convert_degree(self.current_yaw_2pi), self.convert_degree(goal_angle)))
        print("==> ALPHA : {} degree".format(self.convert_degree(alpha)))
        print("DISTANCE : {}".format(round(self.distance_Robot_to_Goal(), 2)))
        
        lidars = np.reshape(self.lidars, (SPACE.REGION_LIDAR_SPAGE, SPACE.SECTIONS_LIDARSPACE))
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
        if self.step_counter == 1:
            self.start_time = time.time()
            self.collision_count = 0
            self.total_lidar_distance = 0.0
            self.lidar_samples = 0

        state = self.observe()
        print("state = {}".format(state))
        print("record action: {}".format(self.last_actions_record))
        action = self.choose_action(state)
        print("---------- Step {} ---------------- :".format(self.step_counter))
        print("action = {}".format(action))
        if action == ACTIONS.FORWARD:
            self.adjust_to_right_angle(0.1)
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

        # End of run logic
        if self.distance_Robot_to_Goal() < GOAL_THRESHOLD:
            self.end_time = time.time()
            run_time = self.end_time - self.start_time
            avg_distance_to_obstacle = self.total_lidar_distance / self.lidar_samples if self.lidar_samples > 0 else 0

            print("Run Completed in {} seconds".format(run_time))
            print("Total Collisions: {}".format(self.collision_count))
            print("Average Distance to Obstacles: {:.2f}".format(avg_distance_to_obstacle))
            print("Total Distance Traveled: {:.2f}".format(self.total_distance_traveled))  

            self.stop_robot()
            rospy.signal_shutdown("Goal reached or end of episode")

    def angleBetweenTwoPoints(self, xPointA, yPointA, xPointB, yPointB):
        delta_x = xPointB - xPointA
        delta_y = yPointB - yPointA
        radian_angle = math.atan2(delta_y, delta_x)
        if radian_angle < 0:
            radian_angle += 2 * math.pi
        return radian_angle

    def convert_current_yaw_2pi(self):
        current_yaw_2pi = self.current_yaw
        if current_yaw_2pi > -math.pi and current_yaw_2pi < 0:
            current_yaw_2pi += 2 * math.pi
        return current_yaw_2pi

    def distance_Robot_to_Goal(self):
        return abs(self.current_x - PLAYER_SETTING.X_GOAL) + abs(self.current_y - PLAYER_SETTING.Y_GOAL)
    
    def convert_degree(self, angle):
        return round(angle * 180 / math.pi, 2)
    
    def convert_alpha_pi(self, angle_robot_vs_Goal):
        alpha = abs(self.current_yaw_2pi - angle_robot_vs_Goal)
        if alpha > math.pi:
            alpha = 2 * math.pi - alpha
        return alpha

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
