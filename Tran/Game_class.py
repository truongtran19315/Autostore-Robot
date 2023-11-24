import cv2
import pygame_2d
from consts import *
import numpy as np
from logVersion import *
import matplotlib.pyplot as plt
import math
from matplotlib.animation import FuncAnimation
# plt.ion()


# folder_path = getlogVersion(base_path)
# logFolderPath = os.path.join(folder_path, 'LOG')
# log_path = getLog_path(logFolderPath)

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
        self.state_count_change = 0
        self.record_state_change = []
        self.total_states = ALPHA_SPACE * FWVELO_SPACE * RVELO_SPACE * \
            math.pow(LENGTH_LIDARSIGNAL, SECTIONS_LIDARSPACE)
        self.fig = None

    def pick_sample(self, state, q_table):
        if np.random.random() > self.epsilon:
            action = np.argmax(q_table[tuple(state)])
        else:
            action = np.random.randint(0, ACTION_SPACE)
        return action

    def run_episode(self, q_table, video, trackPosition, log_path, all_States, counter=COUNTER):
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
        step_count = 0
        
        prev_state = None
        # state_count_change = 0
        # print(f"Start Real Posion: {firstPosition}")
        while not done and counter > 0:
        # while done == 0:
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

            # screenRecord.append(self.trackGame(counter, action))
            # video.write(self.trackGame(counter, action))
            cv2.circle(trackPosition, self.get_RealPosion(), PLAYER_SETTING.RADIUS_OBJECT, COLOR.GREEN, 1)
            
            maxQ = np.max(q_table[tuple(next_state)])
            q_table[tuple(state)][action] += self.alpha * (reward +
                                                           self.gamma * maxQ - q_table[tuple(state)][action])

            with open(log_path, 'a') as file:
                print('Curr-State: ' + str(state) \
                    + '\nAction:' + str(action) + ' obs: ' + str(next_state) + ' reward: ' + str(reward) \
                    + '\nq-table at state: ' + str(q_table[tuple(state)]) + '\n', file=file)
                
            if prev_state is not None and not np.array_equal(prev_state, state):
                if all_States[tuple(state)] == 0:
                    self.state_count_change += 1
                    all_States[tuple(state)] = 1

            prev_state = state
            state = next_state
            total_reward += reward
            reward_records.append(reward)
            counter -= 1
        # print(f"End Real Posion: {self.get_RealPosion()}")
        cv2.circle(trackPosition, firstPosition, PLAYER_SETTING.RADIUS_OBJECT, COLOR.RED, 1)
        state_change_str = 'state change rate: ' + str(self.state_count_change/(ALPHA_SPACE*FWVELO_SPACE*RVELO_SPACE*ACTION_SPACE*LENGTH_LIDARSIGNAL**SECTIONS_LIDARSPACE))
        cv2.putText(trackPosition, state_change_str, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        
        turn_right = 'turn right: ' + str(turn_right)
        turn_left = 'turn left: ' + str(turn_left)
        stop = 'stop: ' + str(stop)
        ahead = 'ahead: ' + str(ahead)
        slowdown = 'slowdown: ' + str(slowdown)
        nothing = 'nothing: ' + str(nothing)
        step_count = 'step counter: ' + str(step_count)
        cv2.putText(trackPosition, step_count, (50, 400), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, turn_right, (50, 450), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, turn_left, (50, 500), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, stop, (50, 550), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, ahead, (50, 600), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, slowdown, (50, 650), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        cv2.putText(trackPosition, nothing, (50, 700), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
        
        self.record_state_change.append(self.state_count_change)

        if self.epsilon - self.epsilon_decay >= self.epsilon_min:
            self.epsilon -= self.epsilon_decay
        # print(f"Total reward each epsilon : {reward}")
        # print(f"List record reward epsilon : {reward_records}")

        # print(f"List record action: {action_records}")
        # print(f"Total reward each epsilon : {total_reward}")
        # print(f"List record reward: {reward_records}")
        # return total_reward, screenRecord
        return float(total_reward), video, trackPosition, done

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

    def getEnv(self):
        return self.game.getEnv()
    
    def get_RealPosion(self):
        return int(self.game.robot.xPos), int(self.game.robot.yPos)

# screen = np.zeros((720, 1280, 3), dtype=np.uint8)
# game = Game(screen)
# game.trackGame()
