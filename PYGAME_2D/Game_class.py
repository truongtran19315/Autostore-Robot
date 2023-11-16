import cv2
import pygame_2d
from consts import *
import numpy as np
from logVersion import *
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
# plt.ion()


class Game:
    def __init__(self, screen):
        self.screen = screen
        self.game = pygame_2d.PyGame2D(screen=self.screen)
        self.lidarspace_shape = tuple(
            [LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
        self.new_observation_shape = (ALPHA_SPACE, FWVELO_SPACE,
                                      RVELO_SPACE) + self.lidarspace_shape
        self.epsilon = epsilon
        self.epsilon_min = epsilon_min
        self.epsilon_decay = epsilon_decay
        self.alpha = alpha
        self.gamma = gamma
        # self.count_state_change = 0
        self.record_state_change = []
        self.total_states = ALPHA_SPACE * FWVELO_SPACE * RVELO_SPACE * \
            math.pow(LENGTH_LIDARSIGNAL, SECTIONS_LIDARSPACE) * ACTION_SPACE
        self.fig = None

        self.videoFile_path = getlogVideo_path(getlogVersion(base_path))
        self.recordVideo = cv2.VideoWriter(self.videoFile_path,
                                           cv2.VideoWriter_fourcc(*'MJPG'),
                                           GAME_SETTING.FPS,
                                           (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))

    def pick_sample(self, state, q_table):
        if np.random.random() > self.epsilon:
            action = np.argmax(q_table[tuple(state)])
        else:
            action = np.random.randint(0, ACTION_SPACE)
        return action

    def run_episode(self, q_table, counter=COUNTER):
        done = False
        total_reward = 0
        state = self.game.observe()
        reward_records = []
        action_records = []        
        screenRecord = []

        prev_state = None
        state_count_change = 0
        print(f"Start Real Posion: {self.get_RealPosion()}")
        while not done and counter > 0:
            action = self.pick_sample(state, q_table)
            action_records.append(action)
            next_state, reward, done = self.step(action)
            screenRecord.append(self.trackGame(counter, action))
            maxQ = np.max(q_table[tuple(next_state)])
            q_table[tuple(state)][action] += self.alpha * (reward +
                                                           self.gamma * maxQ - q_table[tuple(state)][action])

            if prev_state is not None and not np.array_equal(prev_state, state):
                state_count_change += 1

            prev_state = state
            state = next_state
            total_reward += reward
            reward_records.append(reward)
            counter -= 1
        print(f"End Real Posion: {self.get_RealPosion()}")

        self.record_state_change.append(state_count_change)

        if self.epsilon - self.epsilon_decay >= self.epsilon_min:
            self.epsilon -= self.epsilon_decay
        # print(f"Total reward each epsilon : {reward}")
        # print(f"List record reward epsilon : {reward_records}")

        # print(f"List record action: {action_records}")
        print(f"Total reward each epsilon : {total_reward}")
        # print(f"List record reward: {reward_records}")
        return total_reward, screenRecord

    def reset(self):
        del self.game
        self.game = pygame_2d.PyGame2D(screen=self.screen)
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

    def creat_axes(self, axes, step_number):
        record_state_change_rate = np.array(
            self.record_state_change) / self.total_states

        axes.plot(np.arange(0, step_number + 1), record_state_change_rate,
                  marker='o', linestyle='-', color='b', label='rate states change')

        axes.set_ylabel('states change')
        axes.set_xlabel('n epsilon')
        # axes.set_ylim(0, 1)
        axes.set_xlim(0, step_number)

    def trackGame(self, counter, action):
        # key = cv2.waitKey(delay=1)
        # if key == 27 or (key & 0xFF == 'q'):
        #     input = 27
        # else:
        #     input = -1
        screen = self.game.view(counter, action)
        
        return screen

    def get_RealPosion(self):
        return self.game.robot.xPos, self.game.robot.yPos

# screen = np.zeros((720, 1280, 3), dtype=np.uint8)
# game = Game(screen)
# game.trackGame()
