import pygame_2d
from const import *
import numpy as np
import os
import datetime
import pickle  # Thêm thư viện pickle để lưu và tải Q-table
import time

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = pygame_2d.PyGame2D(screen=screen)

lidarspace_shape = tuple([LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)
new_observation_shape = (ALPHA_SPACE, FWVELO_SPACE,
                         RVELO_SPACE) + lidarspace_shape


base_path = "C:/Users/truon/PROJECTS/PYTHON/do-an-hk231/DATA"  # Đường dẫn cơ sở

# Tạo một thư mục với ngày và giờ hiện tại
current_datetime = datetime.datetime.now()
folder_name = current_datetime.strftime("%Y-%m-%d")
folder_path = os.path.join(base_path, folder_name)
os.makedirs(folder_path, exist_ok=True)

# Định nghĩa đường dẫn file Q-table
q_table_filename = "q_table.pkl"  # Đổi đuôi file thành .pkl
q_table_path = os.path.join(folder_path, q_table_filename)

# Khởi tạo Q-table hoặc tải từ file nếu có
if os.path.exists(q_table_path):
    with open(q_table_path, "rb") as f:  # Sử dụng pickle để tải Q-table
        q_table = pickle.load(f)
    print("Q-table loaded successfully.")
else:
    q_table = np.zeros(new_observation_shape + (ACTION_SPACE,))  # ! random
    print("No existing Q-table found. Creating a new Q-table.")

# Rest of your code...
print(q_table.shape)
print(game.observe())
print("Start training....")
Start_time = time.now()


def pick_sample(state):
    if np.random.random() > epsilon:
        action = np.argmax(q_table[tuple(state)])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action


def run_episode():
    global epsilon
    done = False
    total_reward = 0
    state = game.observe()
    reward_records = []  # Khai báo biến cục bộ

    while not done:
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
    if epsilon - epsilon_decay >= epsilon_min:  # Sử dụng hằng số epsilon_min
        epsilon -= epsilon_decay

    # Record total rewards in episode
    print("Run episode {} with rewards {}".format(i, total_reward), end="\r")
    reward_records.append(total_reward)
    return reward_records


for i in range(n_epsilondes):
    training_completed = False
    reward_records = run_episode()
    if i == n_epsilondes - 1:
        training_completed = True

    # Lưu Q-table sau mỗi episode
    with open(q_table_path, "wb") as f:  # Sử dụng pickle để lưu Q-table
        pickle.dump(q_table, f)
    print("Q-table saved to:", q_table_path)


# Khi huấn luyện xong, thêm "done" vào tên thư mục
if training_completed:
    folder_name += "_DONE"
    folder_path_done = os.path.join(base_path, folder_name)
    os.rename(folder_path, folder_path_done)
    folder_path = folder_path_done

# Lưu Q-table cuối cùng
with open(os.path.join(folder_path, q_table_filename), "wb") as f:  # Sử dụng pickle để lưu Q-table
    pickle.dump(q_table, f)
