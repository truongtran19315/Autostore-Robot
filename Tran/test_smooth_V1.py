import matplotlib.pyplot as plt
import os
import pickle
import numpy as np

date_train = '1Mil_Completed'
base_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'DATA')
allState_path = os.path.join(base_path, date_train, 'allStates.pkl')
foler_train_path = os.path.join(base_path, date_train, 'q_table.pkl')
record_goal_step_count_path = os.path.join(
    base_path, date_train, 'record_goal_step_count.pkl')
record_state_change_path = os.path.join(
    base_path, date_train, 'record_state_change.pkl')
reward_records_path = os.path.join(base_path, date_train, 'reward_records.pkl')

diagram_path = os.path.join(base_path, date_train, 'diagram_V1.png')

with open(foler_train_path, "rb") as f:
    q_table = pickle.load(f)

with open(record_state_change_path, "rb") as f:
    record_state_change = pickle.load(f)

with open(reward_records_path, "rb") as f:
    reward_records = pickle.load(f)

with open(record_goal_step_count_path, "rb") as f:
    record_goal_step_count = pickle.load(f)


def smooth_data(data, window_size=50):
    smoothed_data = np.convolve(data, np.ones(
        window_size)/window_size, mode='valid')
    return smoothed_data


fig, axes = plt.subplots(nrows=3, ncols=1, figsize=(10, 18), dpi=300)
record_state_change_rate = np.array(record_state_change) / (5*10*(4**5))
axes[0].set_title('Biểu đồ biểu thị độ phủ của state trong Q-table')
axes[0].plot(np.arange(0, len(record_state_change_rate)), record_state_change_rate,
             linestyle='-', color='blue', label='Tốc độ cập nhật states')
axes[0].set_ylabel('Tỉ lệ states đã thay đổi')
axes[0].set_xlabel('Lần chạy')
# axes[0].set_xlim(0, step_number)
# !----------------------------------
average_reward = []
for idx in range(len(reward_records)):
    avg_list = np.empty(shape=(1,), dtype=int)
    if idx < 10000:
        avg_list = reward_records[:idx+1]
    else:
        avg_list = reward_records[idx-9999:idx+1]
    average_reward.append(np.average(avg_list))
# Plot
axes[1].plot(reward_records, label='Điểm thưởng')
axes[1].plot(
    average_reward, label='Điểm thưởng trung bình')
axes[1].set_title('Biểu đồ điểm thưởng và điểm thưởng trung bình')
axes[1].set_xlabel('Lần chạy')
axes[1].set_ylabel('Điểm thưởng')
# axes[1].legend(loc='upper left')

# goal step count plot
average_goal_step_count = []
for idx in range(len(record_goal_step_count)):
    avg_list_goal_step_count = np.empty(shape=(1,), dtype=int)
    if idx < 10000:
        avg_list_goal_step_count = record_goal_step_count[:idx+1]
    else:
        avg_list_goal_step_count = record_goal_step_count[idx-9999:idx+1]
    average_goal_step_count.append(
        np.average(avg_list_goal_step_count))
axes[2].plot(record_goal_step_count, label='Số bước đến đích')
axes[2].plot(average_goal_step_count,
             label='Số bước trung bình')
axes[2].set_title(
    'Biểu đồ số bước và số bước trung bình cần để tới đích')
axes[2].set_xlabel('Số lần tới đích')
axes[2].set_ylabel('Số bước tới đích')

axes[1].legend(loc='upper left')
axes[2].legend(loc='upper left')
# !---------------------------------------------------------

plt.tight_layout()

plt.savefig(diagram_path)
print(f"Diagram saved to: {diagram_path}")
