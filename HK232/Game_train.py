import os
import pickle
from Game_class import Game
import cv2
from consts import *
import numpy as np
import matplotlib.pyplot as plt
from logVersion import *

# Đường dẫn và tên file cho bảng Q và biểu đồ
q_table_filename = "q_table.pkl"
diagram_filename = "diagram.png"


# Đường dẫn và tên file cho bảng Q
q_table_path = os.path.join(log_folder, q_table_filename)

# Đường dẫn và tên file cho biểu đồ
diagram_path = os.path.join(log_folder, diagram_filename)

# Khởi tạo môi trường trò chơi
screen = np.ones((GAME_SETTING.SCREEN_HEIGHT, GAME_SETTING.SCREEN_WIDTH, 3), dtype=np.uint8) * 255
map = MAP_SETTING.MAP_DEFAULT
game = Game(screen, map)
Env = game.getEnv()

# Kiểm tra xem bảng Q đã tồn tại chưa, nếu không sẽ tạo mới
if os.path.exists(q_table_path):
    with open(q_table_path, "rb") as f:
        q_table = pickle.load(f)
    print("Q-table loaded successfully!")
else:
    q_table = np.random.uniform(low=-1, high=1, size=(*game.new_observation_shape, ACTION_SPACE))
    print("No existing Q-table found. Creating a new Q-table ... ")

print(f"Shape of Q-table: {q_table.shape}")
print("Start training....")

# Lưu lại epsilon trước khi chương trình kết thúc
last_epsilon_filename = "last_epsilon.pkl"
last_epsilon_path = os.path.join(log_folder, last_epsilon_filename)
if os.path.exists(last_epsilon_path):
    with open(last_epsilon_path, "rb") as f:
        last_epsilon = pickle.load(f)
else:
    last_epsilon = 0

# Khởi tạo các biến cho việc vẽ biểu đồ và số lần đạt đích
game.fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 18), dpi=300)
goal_count = 0

# Bắt đầu vòng lặp huấn luyện
for i in range(last_epsilon + 1, n_epsilondes + last_epsilon + 1):
    print("Running Episode {}".format(i), end="\r")

    # Reset trạng thái của môi trường
    game.reset()
    Env = game.getEnv()
    trackPos = Env.copy()

    # Đường dẫn và tên file log cho vòng lặp hiện tại
    log_path = logger._get_log_path(log_folder_path, last_epsilon)

    # Chạy một episode của trò chơi
    reward, trackPosition, done = game.run_episode(q_table, trackPos, log_path)
    if done == PLAYER_SETTING.GOAL:
        goal_count += 1

    # Lưu các điểm thưởng và số bước cần đến đích
    game.reward_records.append(reward)

    # Lưu số lần đạt đích vào trackPosition
    goal_count_str = 'Goal counter: ' + str(goal_count)
    cv2.putText(trackPosition, goal_count_str, (300, 440), cv2.FONT_HERSHEY_TRIPLEX, 9/16, COLOR.BLACK, 1)

    # Đường dẫn và tên file cho hình ảnh log
    trackPosition_path = logger._get_position_path(image_folder_path, done, last_epsilon)

    # Lưu hình ảnh log
    cv2.imwrite(trackPosition_path, trackPosition)

    # Ghi frame vào video
    video_writer.write(trackPosition)

    # Lưu lại bảng Q
    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)

    # Kiểm tra điều kiện dừng
    if cv2.waitKey(1) & 0xFF == 'q':
        video_writer.release()
        break

    # Lưu lại thông tin sau mỗi 1000 episodes
    if i % 1000 == 0:
        with open(os.path.join(log_folder, q_table_filename), "wb") as f:
            pickle.dump(q_table, f)

        # Lưu các thông tin khác
        with open(os.path.join(log_folder, game.allStates_filename), "wb") as f:
            pickle.dump(game.allStates, f)
        with open(os.path.join(log_folder, game.record_state_change_filename), "wb") as f:
            pickle.dump(game.record_state_change, f)
        with open(os.path.join(log_folder, last_epsilon_filename), "wb") as f:
            pickle.dump(last_epsilon, f)
        with open(os.path.join(log_folder, game.reward_records_filename), "wb") as f:
            pickle.dump(game.reward_records, f)
        with open(os.path.join(log_folder, game.record_goal_step_count_filename), "wb") as f:
            pickle.dump(game.record_goal_step_count, f)

    # Tăng epsilon
    last_epsilon += 1

# Lưu biểu đồ
game.creat_axes(axes, i, last_epsilon)
axes[1].legend(loc='upper left')
axes[2].legend(loc='upper left')
plt.savefig(diagram_path)
print(f"Diagram saved to: {diagram_path}")

# Lưu các thông tin còn lại
with open(os.path.join(log_folder, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)
with open(os.path.join(log_folder, game.allStates_filename), "wb") as f:
    pickle.dump(game.allStates, f)
with open(os.path.join(log_folder, game.record_state_change_filename), "wb") as f:
    pickle.dump(game.record_state_change, f)
with open(os.path.join(log_folder, last_epsilon_filename), "wb") as f:
    pickle.dump(last_epsilon, f)
with open(os.path.join(log_folder, game.reward_records_filename), "wb") as f:
    pickle.dump(game.reward_records, f)
with open(os.path.join(log_folder, game.record_goal_step_count_filename), "wb") as f:
    pickle.dump(game.record_goal_step_count, f)

print('Done')
