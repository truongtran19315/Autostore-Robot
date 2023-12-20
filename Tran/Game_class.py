import cv2
import pygame_2d
from consts import *
import numpy as np
from logVersion import *
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
# plt.ion()

import pickle


class Game:
    def __init__(self, screen, map):
        self.screen = screen
        self.map = map
        self.game = pygame_2d.PyGame2D(screen=self.screen, map=self.map)
        self.lidarspace_shape = tuple(
            [LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
        self.new_observation_shape = (DISTANCE_SPACE, ALPHA_SPACE, FWVELO_SPACE,
                                      RVELO_SPACE) + self.lidarspace_shape
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.alpha = alpha
        self.gamma = gamma
        self.total_states = DISTANCE_SPACE * ALPHA_SPACE * FWVELO_SPACE * RVELO_SPACE * \
            math.pow(LENGTH_LIDARSIGNAL, SECTIONS_LIDARSPACE)
        self.fig = None

        #! create state count change to count the number of state change
        self.record_state_change_filename = "record_state_change.pkl"
        self.record_state_change_path = os.path.join(
            folder_path, self.record_state_change_filename)
        if os.path.exists(self.record_state_change_path):
            with open(self.record_state_change_path, "rb") as f:
                self.record_state_change = pickle.load(f)
                self.state_count_change = self.record_state_change[-1:][0]
        else:
            self.state_count_change = 0
            self.record_state_change = []

        #! create allState table to count the state change
        self.allStates_filename = "allStates.pkl"
        self.allStates_path = os.path.join(
            folder_path, self.allStates_filename)
        if os.path.exists(self.allStates_path):
            with open(self.allStates_path, "rb") as f:
                self.allStates = pickle.load(f)
        else:
            self.allStates = np.zeros(self.new_observation_shape)

        #! create record reward
        self.reward_records_filename = "reward_records.pkl"
        self.reward_records_path = os.path.join(
            folder_path, self.reward_records_filename)
        if os.path.exists(self.reward_records_path):
            with open(self.reward_records_path, "rb") as f:
                self.reward_records = pickle.load(f)
        else:
            self.reward_records = []
            print("Create new reward records!!")

        #! create allState table to count the state change
        self.record_goal_step_count_filename = "record_goal_step_count.pkl"
        self.record_goal_step_count_path = os.path.join(
            folder_path, self.record_goal_step_count_filename)
        if os.path.exists(self.record_goal_step_count_path):
            with open(self.record_goal_step_count_path, "rb") as f:
                self.record_goal_step_count = pickle.load(f)
        else:
            self.record_goal_step_count = []

    def pick_sample(self, state, q_table):
        if np.random.random() > self.epsilon:
            action = np.argmax(q_table[tuple(state)])
        else:
            action = np.random.randint(0, ACTION_SPACE)
        return action

    def run_episode(self, q_table, trackPosition, log_path, counter=COUNTER):
        done = PLAYER_SETTING.ALIVE
        total_reward = 0
        state = self.game.observe()
        reward_records = []
        action_records = []

        firstPosition = self.get_RealPosion()
        turn_left = 0
        turn_right = 0
        ahead = 0
        slowdown = 0
        stop = 0
        nothing = 0
        step_count = 1

        prev_state = None

        while not done and counter > 0:

            action = self.pick_sample(state, q_table)
            action_records.append(action)
            if action == 0:
                turn_right += 1
            elif action == 1:
                turn_left += 1
            elif action == 2:
                stop += 1
            elif action == 3:
                ahead += 1
            elif action == 4:
                slowdown += 1
            elif action == 5:
                nothing += 1
            step_count += 1

            next_state, reward, done = self.step(action)

            cv2.circle(trackPosition, self.get_RealPosion(),
                       PLAYER_SETTING.RADIUS_OBJECT, COLOR.GREEN, 1)

            with open(log_path, 'a') as file:
                print('Curr-State: ' + str(state)
                      + '\nAction:' + str(action) + ' obs: ' +
                      str(next_state) + ' reward: ' + str(reward)
                      + '\nq-table before update: ' + str(q_table[tuple(state)]) + '\n', file=file)

            maxQ = np.max(q_table[tuple(next_state)])
            q_table[tuple(state)][action] += self.alpha * (reward +
                                                           self.gamma * maxQ - q_table[tuple(state)][action])

            if prev_state is not None and not np.array_equal(prev_state, state):
                if self.allStates[tuple(state)] == 0:
                    self.state_count_change += 1
                    self.allStates[tuple(state)] = 1

            prev_state = state
            state = next_state
            total_reward += reward
            reward_records.append(reward)
            counter -= 1

        if done == 2:
            self.record_goal_step_count.append(step_count)

        cv2.circle(trackPosition, firstPosition,
                   PLAYER_SETTING.RADIUS_OBJECT, COLOR.RED, 1)
        state_change_str = 'state change rate: ' + str(self.state_count_change/(
            self.total_states))
        cv2.putText(trackPosition, state_change_str, (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)

        turn_right = 'turn right: ' + str(turn_right)
        turn_left = 'turn left: ' + str(turn_left)
        stop = 'stop: ' + str(stop)
        ahead = 'ahead: ' + str(ahead)
        slowdown = 'slowdown: ' + str(slowdown)
        nothing = 'nothing: ' + str(nothing)
        step_count_str = 'step counter: ' + str(step_count)
        cv2.putText(trackPosition, step_count_str, (50, 400),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, turn_right, (50, 450),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, turn_left, (50, 500),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, stop, (50, 550),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, ahead, (50, 600),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, slowdown, (50, 650),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, nothing, (50, 700),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)

        self.record_state_change.append(self.state_count_change)

        if self.epsilon - self.epsilon_decay >= self.epsilon_min:
            self.epsilon -= self.epsilon_decay
        else:
            self.epsilon = self.epsilon_min

        return float(total_reward), trackPosition, done

    def reset(self):
        del self.game
        self.screen = np.zeros((720, 1280, 3), dtype=np.uint8)
        self.game = pygame_2d.PyGame2D(screen=self.screen, map=self.map)
        # to update the lidar scan state
        self.game.action(action=ACTIONS.DO_NOTHING)
        obs = self.game.observe()
        return obs

    def step(self, action):
        self.game.action(action)
        obs = self.game.observe()
        reward = self.game.evaluate()
        done = self.game.is_done()
        return obs, reward, done

    def render(self):
        self.game.view(self.screen)

    def creat_axes(self, axes, step_number, last_epsilon):
        record_state_change_rate = np.array(
            self.record_state_change) / self.total_states
        axes[0].set_title('Biểu đồ biểu thị độ phủ của state trong Q-table')
        axes[0].plot(np.arange(0, len(record_state_change_rate)), record_state_change_rate,
                     linestyle='-', color='blue', label='Tốc độ cập nhật states')
        axes[0].set_ylabel('Tỉ lệ states đã thay đổi')
        axes[0].set_xlabel('Lần chạy')
        axes[0].set_xlim(0, step_number)
        axes[1].set_title('Tốc độ cập nhật states')
        # axes[1].plot(np.arange(0, len(self.reward_records)), np.array(
        #     self.reward_records), color='red', label='Thay đổi hàm reward')
        # axes[1].set_ylabel('Giá trị reward')
        # axes[1].set_xlabel('số lần chạy')
        # axes[1].set_xlim(0, step_number)

        average_reward = []
        for idx in range(len(self.reward_records)):
            avg_list = np.empty(shape=(1,), dtype=int)
            if idx < 50:
                avg_list = self.reward_records[:idx+1]
            else:
                avg_list = self.reward_records[idx-49:idx+1]
            average_reward.append(np.average(avg_list))
        # Plot
        axes[1].plot(self.reward_records, label='Điểm thưởng')
        axes[1].plot(
            average_reward, label='Điểm thưởng trung bình (trong 50 lần chạy)')
        axes[1].set_title('Biểu đồ điểm thưởng và điểm thưởng trung bình')
        axes[1].set_xlabel('Lần chạy')
        axes[1].set_ylabel('Điểm thưởng')
        # axes[1].legend(loc='upper left')

        # goal step count plot
        average_goal_step_count = []
        for idx in range(len(self.record_goal_step_count)):
            avg_list_goal_step_count = np.empty(shape=(1,), dtype=int)
            if idx < 50:
                avg_list_goal_step_count = self.record_goal_step_count[:idx+1]
            else:
                avg_list_goal_step_count = self.record_goal_step_count[idx-49:idx+1]
            average_goal_step_count.append(
                np.average(avg_list_goal_step_count))
        axes[2].plot(self.record_goal_step_count, label='Số bước đến đích')
        axes[2].plot(average_goal_step_count,
                     label='Số bước trung bình (trong 50 lần tới đích)')
        axes[2].set_title(
            'Biểu đồ số bước và số bước trung bình cần để tới đích')
        axes[2].set_xlabel('Số lần tới đích')
        axes[2].set_ylabel('Số bước tới đích')

    def getEnv(self):
        return self.game.getEnv()

    def get_RealPosion(self):
        return int(self.game.robot.xPos), int(self.game.robot.yPos)
