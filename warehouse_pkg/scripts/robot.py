#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
import math
import numpy as np
import pickle
from consts import * 


class Robot():
    GOAL_X = -2.5
    GOAL_Y = 6.5
    ROBOT_START_X = 8.5
    ROBOT_START_Y = 8.5
    DURATION = 0.1
    MAX_FORWARD_VELO = 1
    MIN_ROTATION_VELO = -math.pi/2
    MAX_ROTATION_VELO = math.pi/2
    INT_INFINITY = 99999

    def __init__(self, robot_x=ROBOT_START_X, robot_y=ROBOT_START_Y, ang_x=0, ang_goal=0, fw_velo=0, ang_velo=0):
        self.ang_x = ang_x
        self.ang_goal = ang_goal
        self.lidars = [self.INT_INFINITY] * 4
        self.robot_x = robot_x
        self.robot_y = robot_y
        with open('/home/truong/Documents/DATA/q_table.pkl', 'rb') as f:
            self.q_table = pickle.load(f)

    def update_robot_position(self):
        if self.ang_velo != 0:
            # Robot rotates but does not move forward in grid-based movement
            pass
        else:
            # Robot moves forward by 1m in the direction it's facing
            self.robot_x += math.cos(self.ang_x)
            self.robot_y += math.sin(self.ang_x)
        print('robot_x: ', self.robot_x, ' robot_y: ', self.robot_y)

    def scan_callback(self, data):
        # Lấy giá trị khoảng cách tại các góc quét phải, trước, trái và sau 
        self.lidars = [data.ranges[270], data.ranges[0], data.ranges[90], data.ranges[180]]

    def modify_angular(self):
        # Robot can only rotate 90 degrees left/right or 180 degrees
        if self.ang_velo == -math.pi/4:
            self.ang_x -= math.pi/2
        elif self.ang_velo == math.pi/4:
            self.ang_x += math.pi/2
        elif self.ang_velo == math.pi/2:
            self.ang_x += math.pi
        self.ang_x %= 2 * math.pi

    def modify_fw_vel(self):
        # Robot can only move forward by 1m or stay still
        if self.fw_velo > self.MAX_FORWARD_VELO:
            self.fw_velo = self.MAX_FORWARD_VELO
        elif self.fw_velo < 0.0:
            self.fw_velo = 0.0

    def modify_ang_vel(self):
        # Robot can only rotate at specific velocities
        if self.ang_velo > self.MAX_ROTATION_VELO:
            self.ang_velo = self.MAX_ROTATION_VELO
        elif self.ang_velo < self.MIN_ROTATION_VELO:
            self.ang_velo = self.MIN_ROTATION_VELO

    def print_scan(self):
        print('scan is: ', self.lidars, ' - len is: ', len(self.lidars))

   
    def observe(self):
        a = self.ang_x
        b = self.angleBetweenTwoPoints(
            self.robot_x, self.robot_y, self.GOAL_X, self.GOAL_Y)
        # alpha = abs(a - b)
        alpha = a - b
        if alpha > PLAYER_SETTING.PI:
            alpha += -2*PLAYER_SETTING.PI
        elif alpha < -PLAYER_SETTING.PI:
            alpha += 2*PLAYER_SETTING.PI

        #! convert lidar length from robot to obstacle
        lidarLength_bin = [20]
        lidarLength_digitized = np.digitize(
            self.lidars, lidarLength_bin)

        #! convert distance from robot to target
        distanceGoal_bin = np.linspace(
            PLAYER_SETTING.DISTANCEGOAL_MIN, PLAYER_SETTING.DISTANCEGOAL_MAX, num=DISTANCE_SPACE, endpoint=False)
        distanceGoal_bin = np.delete(distanceGoal_bin, 0)
        infoStateVector = []
        infoStateVector.append(np.digitize(
            self.distanceFromRobotTo(self.GOAL_X, self.GOAL_Y), distanceGoal_bin))

        #! convert angular deviation between robot and target
        alphaGoal_bin = np.linspace(-math.pi, math.pi,
                                    num=ALPHA_SPACE, endpoint=False)
        alphaGoal_bin = np.delete(alphaGoal_bin, 0)
        infoStateVector.append(np.digitize(alpha, alphaGoal_bin))

        infoStateVector = np.array(infoStateVector)
        lidarStateVector = np.array(lidarLength_digitized)
        #! distance, alpha, 0, 90, 180
        return np.concatenate((infoStateVector, lidarStateVector))    

    # read q_table
    def read_qtable(self):

        state = self.observe()
        # print('q_table[tuple(state)] ', self.q_table[tuple(state)])
        # print(state)
        action = np.argmax(self.q_table[tuple(state)])
        # action = np.argmax(tuple(state))

        self.robot_actions(action)

        self.modify_fw_vel()
        self.modify_ang_vel()
        move = Twist()

        move.linear.x = self.fw_velo
        move.angular.z = self.ang_velo

        # print("angular velocity: ", self.ang_velo)
        # print("linear velocity: ", self.fw_velo)

        self.modify_angular()
        # self.update_robot_position()
        return move, action

    # robot actions
    def robot_actions(self, action):
        # forward
        if action == 0:
            self.ang_velo = 0
            self.fw_velo = 0.5
            print("forward")
        
        # rotate right
        elif action == 1:
            self.fw_velo = 0
            self.ang_velo = -math.pi/4
            print("rotate right")
            
        # rotate left
        elif action == 2:
            self.fw_velo = 0            
            self.ang_velo = math.pi/4
            print("rotate left")
            
        # Turn back
        elif action == 3:
            self.fw_velo = 0
            self.ang_velo = -math.pi/2
            print("Turn back")
            
        # do nothing
        elif action == 4:
            print("do nothing")
            pass

    def angleBetweenTwoPoints(self, xPointA, yPointA, xPointB, yPointB):
        delta_x = xPointB - xPointA
        delta_y = yPointB - yPointA
        # print('a', delta_y, delta_x)
        radian_angle = math.atan2(delta_y, delta_x)
        if radian_angle < 0:
            radian_angle = -radian_angle
        else: radian_angle = math.pi * 2 - radian_angle
        # degree_angle = math.degrees(radian_angle)
        return radian_angle
    
    def distanceFromRobotTo(self, xGoal, yGoal):
        return math.sqrt((xGoal - self.robot_x)**2 + (yGoal - self.robot_y)**2)