import os
import pickle
import numpy as np
import pygame_2d
import matplotlib.pyplot as plt
from const import *
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

    def run_episode(self, q_table, couter=COUNTER):
        done = False
        total_reward = 0
        state = self.game.observe()
        reward_records = []
        changed_states_count = 0  # To count the number of states changed

        while not done and couter > 0:
            action = self.pick_sample(state, q_table)
            next_state, reward, done = self.step(action)

            maxQ = np.max(q_table[tuple(next_state)])
            old_q_value = q_table[tuple(state)][action]
            new_q_value = old_q_value + self.alpha * \
                (reward + self.gamma * maxQ - old_q_value)

            # Check if the Q-value changed
            if old_q_value != new_q_value:
                changed_states_count += 1

            q_table[tuple(state)][action] = new_q_value

            state = next_state
            total_reward += reward

            couter -= 1

        if self.epsilon - self.epsilon_decay >= self.epsilon_min:
            self.epsilon -= self.epsilon_decay

        reward_records.append(total_reward)
        return reward_records, changed_states_count

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


folder_path = getlogVersion(base_path)
os.makedirs(folder_path, exist_ok=True)

q_table_filename = "q_table.pkl"
q_table_path = os.path.join(folder_path, q_table_filename)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = Game(screen)

if os.path.exists(q_table_path):
    with open(q_table_path, "rb") as f:
        q_table = pickle.load(f)
    print("Q-table loaded successfully!")
else:
    q_table = np.random.rand(*game.new_observation_shape + (ACTION_SPACE,))
    print("No existing Q-table found. Creating a new Q-table ... ")

print(f"Shape of Q-table: {q_table.shape}")
print(f"Init state:  {game.reset()}")
print("Start training....")


changed_states_list = []
episodes_list = []

for i in range(n_epsilondes):
    print(f"Episode {i}:")
    game.reset()
    reward_records, changed_states_count = game.run_episode(q_table)

    changed_states_list.append(changed_states_count)
    episodes_list.append(i)

    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)
    print(f"Q-table saved to: {q_table_path}")

    if i % 100 == 0:
        avg_changed_states = np.mean(changed_states_list) / (
            game.new_observation_shape[0] * game.new_observation_shape[1] * ACTION_SPACE) * 100
        print(f"Average percentage of states changed: {avg_changed_states}%")
        plt.plot(episodes_list, changed_states_list)
        plt.xlabel('Number of Episodes')
        plt.ylabel('Percentage of States Changed')
        plt.title('Q-table State Changes Over Episodes')
        plt.savefig(os.path.join(folder_path, 'q_table_changes_graph.png'))
        plt.show()

folder_path_done = folder_path + "_DONE"
os.rename(folder_path, folder_path_done)
folder_path = folder_path_done

with open(os.path.join(folder_path, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)


changed_states_list = np.array(changed_states_list)
plt.plot(episodes_list, changed_states_list)
plt.xlabel('Number of Episodes')
plt.ylabel('Percentage of States Changed')
plt.title('Q-table State Changes Over Episodes')
plt.savefig(os.path.join(folder_path, 'q_table_changes_graph.png'))
plt.show()
