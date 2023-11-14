import pygame_2d
from const import *
import numpy as np
import os
import pickle
from logVersion import *


screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = pygame_2d.PyGame2D(screen=screen)

lidarspace_shape = tuple([LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
new_observation_shape = (ALPHA_SPACE, FWVELO_SPACE,
                         RVELO_SPACE) + lidarspace_shape


def pick_sample(state, q_table):
    if np.random.random() > epsilon:
        action = np.argmax(q_table[tuple(state)])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action


def run_episode(q_table, couter=100):
    global epsilon
    done = False
    total_reward = 0
    state = game.observe()
    reward_records = []

    while not done and couter <= 0:
        action = pick_sample(state, q_table)
        next_state, reward, done = step(action)

        maxQ = np.max(q_table[tuple(next_state)])
        q_table[tuple(state)][action] += alpha * (reward +
                                                  gamma * maxQ - q_table[tuple(state)][action])

        state = next_state
        total_reward += reward

        couter -= 1

    if epsilon - epsilon_decay >= epsilon_min:
        epsilon -= epsilon_decay

    reward_records.append(total_reward)
    return reward_records


def reset():
    del game
    game = pygame_2d.PyGame2D(screen=screen)
    obs = game.observe()
    return obs


def step(action):
    game.action(action)
    obs = game.observe()
    reward = game.evaluate()
    done = game.is_done()
    return obs, reward, done


def render():
    game.view(screen)
