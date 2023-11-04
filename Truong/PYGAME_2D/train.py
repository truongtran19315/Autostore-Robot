import pygame_2d
from const import *
import numpy as np

screen = np.zeros((720, 1280, 3), dtype=np.uint8)


gamma = 0.99
alpha = 0.1
epsilon = 1
epsilon_decay = epsilon / 4000
n_epsilondes = 10000


game = pygame_2d.PyGame2D(screen=screen)


lidarspace_shape = tuple([LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
new_observation_shape = (ALPHA_SPACE, FWVELO_SPACE,
                         RVELO_SPACE) + lidarspace_shape
q_table = np.zeros(new_observation_shape + (ACTION_SPACE,))  # ! random

print(q_table.shape)
print(game.observe())


def pick_sample(state):
    if np.random.random() > epsilon:
        action = np.argmax(q_table[tuple(state)])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action


reward_records = []
for i in range(n_epsilondes):
    done = False
    total_reward = 0
    state = game.observe()

    stepcounter = 50
    while (not done) and (stepcounter < 50):
        stepcounter += 1

        action = pick_sample(state)
        game.action(action)

        next_state = game.observe()
        reward = game.evaluate()
        done = game.is_done()

        # Update Q-Table
        maxQ = np.max(q_table[tuple(next_state)])
        q_table[tuple(state)][action] += alpha * (reward +
                                                  gamma * maxQ - q_table[tuple(state)][action])

        state = next_state
        total_reward += reward

    # Update epsilon for each episode
    if epsilon - epsilon_decay >= 0:
        epsilon -= epsilon_decay

    # Record total rewards in episode
    print("Run episode {} with rewards {}".format(i, total_reward), end="\r")
    reward_records.append(total_reward)

print("\nDone")
