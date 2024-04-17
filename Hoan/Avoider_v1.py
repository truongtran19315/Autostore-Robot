#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist #ros msg that deals with moving the robot
import math
import numpy as np
import pickle

class AvoiderV1():

    GOAL_X = 3.10
    GOAL_Y = 0.45
    ROBOT_START_X = -5.6
    ROBOT_START_Y = 0.6
    DURATION = 0.1
    MAX_FORWARD_VELO = 1
    MIN_ROTATION_VELO = -1
    MAX_ROTATION_VELO = 1
    ALPHA_SPACE = 20
    FWVELO_SPACE = 4
    RVELO_SPACE = 4
    INT_INFINITY = 99999

    def __init__(self,robot_x = ROBOT_START_X, robot_y = ROBOT_START_Y, ang_x = 0, ang_goal = 0, fw_velo = 0, ang_velo = 0):
        self.ang_x = ang_x
        self.ang_goal = ang_goal
        self.fw_velo = fw_velo
        self.ang_velo = ang_velo
        self.lidars = [self.INT_INFINITY] * 360
        self.robot_x = robot_x
        self.robot_y = robot_y
        with open('/home/hoan/turtlebot3_ws/src/move_turtlebot3/scripts/q_table.pkl', 'rb') as f:
            self.q_table = pickle.load(f)

    def update_robot_position(self):
        
        s = 0
        if self.ang_velo != 0:
            v_x = self.fw_velo * math.sin(self.ang_velo * self.DURATION)
            v_y = self.fw_velo * math.cos(self.ang_velo * self.DURATION)

            s = math.sqrt(v_x ** 2 + v_y ** 2) * self.DURATION
            # s_x =  -linear * ( 1 / angular ) * math.cos(angular * self.DURATION)
            # s_y =  linear * ( 1 / angular ) * math.sin( angular * self.DURATION)
            # s = math.sqrt(s_x ** 2 + s_y ** 2)
        else:
            s = self.fw_velo * self.DURATION

        self.robot_x += s * math.cos(self.ang_x)
        self.robot_y += s * math.sin(self.ang_x)

        print('robot_x: ', self.robot_x, ' robot_y: ', self.robot_y)
    def indentify_scan_msg(self, scan):
        # self.lidars.clear()
        for i in range(0, 360):
            if scan.ranges[i] == float('inf'):
                pass
            else:
                index = i + 90
                if index > 359:
                    index -= 360
                self.lidars[index] = scan.ranges[i]*100
        
    # modify angular
    def modify_angular(self):
        self.ang_x += self.ang_velo * self.DURATION
        if self.ang_x > 2 * math.pi:
            self.ang_x -= 2 * math.pi
        elif self.ang_x < 0:
            self.ang_x += 2 * math.pi
    
    def modify_fw_vel(self):
        if self.fw_velo > self.MAX_FORWARD_VELO:
            self.fw_velo = self.MAX_FORWARD_VELO
        elif self.fw_velo < 0.0:
            self.fw_velo = 0.0
    
    def modify_ang_vel(self):
        if self.ang_velo > self.MAX_ROTATION_VELO:
            self.ang_velo = self.MAX_ROTATION_VELO
        elif self.ang_velo < self.MIN_ROTATION_VELO:
            self.ang_velo = self.MIN_ROTATION_VELO

    def print_scan(self):
        print('scan is: ', self.lidars,' - len is: ', len(self.lidars))

    # def avoid(self):

    #     move = Twist()
    #     self.fw_velo = 0.5
    #     self.ang_velo = 0.2
    #     move.linear.x = self.fw_velo
    #     move.angular.z = self.ang_velo
    #     self.modify_angular()
    #     print("angular velocity: ", self.ang_velo)
    #     print("linear velocity: ", self.fw_velo)
    #     self.update_robot_position()
    #     return move

    def observe(self):
        # ratioLeft = (self.robot.xPos)/(GAME_SETTING.SCREEN_WIDTH)
        a = self.ang_x
        b = self.angleBetweenTwoPoints(
            self.robot_x, self.robot_y, self.GOAL_X, self.GOAL_Y)
        alpha = abs(a - b)
        if alpha > math.pi:
            alpha = 2 * math.pi - alpha
        # print(a, b, alpha)
        fwVelo = self.fw_velo * 100
        rVelo = self.ang_velo
        lidars = self.lidars

        # TODO
        #! chuyen gia tri tia lidar
        bin_lidarsignal = np.linspace(
            0, 360, num = 4, endpoint=True)

        bin_lidarsignal = np.delete(bin_lidarsignal, 0)

        lidars_digitized = np.digitize(lidars, bin_lidarsignal)

        # bin_lidarspace = np.array([33, 84, 96, 147])
        # bin_lidarspace = np.array([84, 96])
        bin_lidarspace = np.array([45, 135, 180])

        lidars_sections = np.array_split(lidars_digitized, bin_lidarspace)

        section_lidars_min = [np.amin(section) for section in lidars_sections]

        #! chuyen doi state vector
        high, low = 1, 0
        alpha_space = 0, math.pi
        fwVelo_space = 0, self.MAX_FORWARD_VELO * 100
        rVelo_space = self.MIN_ROTATION_VELO, self.MAX_ROTATION_VELO

        lowState = np.array(
                [alpha_space[0], fwVelo_space[0], rVelo_space[0]], dtype=float)
        upState = np.array(
            [alpha_space[1], fwVelo_space[1], rVelo_space[1]], dtype=float)

        infoState_shape = (self.ALPHA_SPACE, self.FWVELO_SPACE, self.RVELO_SPACE)

        bins = []
        for i in range(3):
            item = np.linspace(
                lowState[i],
                upState[i],
                num=infoState_shape[i],
                endpoint=False)
            item = np.delete(item, 0)
            bins.append(item)

        s = (alpha, fwVelo, rVelo)
        infoStateVector = []
        for i in range(3):
            infoStateVector.append(np.digitize(s[i], bins[i]))

        infoStateVector = np.array(infoStateVector)
        lidarStateVector = np.array(section_lidars_min)
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
        return move

    # robot actions
    def robot_actions(self, action):

        # rotate right
        if action == 0:
            self.ang_velo -= 0.05
            print("action 0")
        # rotate left
        elif action == 1:
            self.ang_velo += 0.05
            print("action 1")
        # stop
        elif action == 2:
            self.fw_velo = 0
            self.ang_velo = 0
            print("action 2")
        # accelerate
        elif action == 3:
            self.fw_velo += 0.05
            print("action 3")
        # decelerate
        elif action == 4:
            self.fw_velo -= 0.05
            print("action 4")
        # do nothing
        elif action == 5:
            print("action 5")
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
        

		