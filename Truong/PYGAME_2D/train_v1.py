import pygame_2d
from const import *
import numpy as np

screen = np.zeros((720, 1280, 3), dtype=np.uint8)

# Hyperparameters
gamma = 0.99
alpha = 0.1
epsilon = 1.0
epsilon_min = 0.01
epsilon_decay = (epsilon - epsilon_min) / 10000
n_episodes = 10000

game = pygame_2d.PyGame2D(screen=screen)

# Define the shape of the observation space and action space
lidarspace_shape = tuple([LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
observation_shape = (ALPHA_SPACE, FWVELO_SPACE, RVELO_SPACE) + lidarspace_shape
q_table = np.zeros(observation_shape + (ACTION_SPACE,))

print("Q-table shape:", q_table.shape)
print("Initial observation:", game.observe())

# Function to pick an action based on the current state


def pick_action(state, epsilon):
    if np.random.rand() < epsilon:
        return np.random.randint(0, ACTION_SPACE)
    else:
        return np.argmax(q_table[state])


# Training loop
reward_records = []
for episode in range(n_episodes):
    total_reward = 0
    state = game.observe()
    done = False

    while not done:
        action = pick_action(state, epsilon)
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

    # Epsilon decay
    epsilon = max(epsilon_min, epsilon - epsilon_decay)

    # Logging
    if episode % 100 == 0:
        print(f"Episode: {episode}, Total reward: {
              total_reward}, Epsilon: {epsilon}")
    print(f"step: {episode}")
    reward_records.append(total_reward)

print("Training completed")
