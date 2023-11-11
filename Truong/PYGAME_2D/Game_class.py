import pygame_2d
from const import *
import numpy as np
import os
import pickle
from logVersion import *


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

    def pick_sample(self, state, q_table):
        if np.random.random() > self.epsilon:
            action = np.argmax(q_table[tuple(state)])
        else:
            action = np.random.randint(0, ACTION_SPACE)
        return action

    def run_episode(self, q_table, couter=100):
        done = False
        total_reward = 0
        state = self.game.observe()
        reward_records = []

        while not done and couter > 0:
            action = self.pick_sample(state, q_table)
            next_state, reward, done = self.step(action)

            maxQ = np.max(q_table[tuple(next_state)])
            q_table[tuple(state)][action] += self.alpha * (reward +
                                                           self.gamma * maxQ - q_table[tuple(state)][action])

            state = next_state
            total_reward += reward

            couter -= 1
            print(f"Couter: {couter} -- reward: {reward}")

        if self.epsilon - self.epsilon_decay >= self.epsilon_min:
            self.epsilon -= self.epsilon_decay

        reward_records.append(total_reward)
        return reward_records

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
