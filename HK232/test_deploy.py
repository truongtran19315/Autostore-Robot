import pickle
from consts import GAME_SETTING
import numpy as np
from Game_class import *
from datetime import datetime
from logVersion import base_path
import matplotlib.pyplot as plt

date_train = '2024-05-28_V1_DONE'
folder_train_path = os.path.join(base_path, date_train, 'q_table.pkl')

with open(folder_train_path, "rb") as f:
    q_table = pickle.load(f)

numberRun = 10000  # Số lần chạy
average_number = 100
arrRun = []
print('================ Start Run =========================')
for i in range(1, numberRun + 1):
    screen = np.ones((GAME_SETTING.SCREEN_HEIGHT,
                      GAME_SETTING.SCREEN_WIDTH, 3), dtype=np.uint8) * 255
    robot = Game(screen, MAP_SETTING.MAP_DEFAULT)
    Env = robot.getEnv()

    state = robot.reset()
    done = 0
    counter = 1000
    goal = 0
    while done == 0 and counter > 0:
        action = np.argmax(q_table[tuple(state)])
        next_state, _, done = robot.step(action)
        state = next_state
        counter -= 1
        if done == 2:
            goal += 1
            break
    print('At step {}, Total robot arrival goal is number {}'.format(
        i, goal), end='\r')
    arrRun.append(goal)

fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(8, 6))

average_goal_step_count = []
for idx in range(len(arrRun)):
    avg_list_goal_step_count = np.empty(shape=(1,), dtype=int)
    if idx < average_number:
        avg_list_goal_step_count = arrRun[:idx+1]
    else:
        avg_list_goal_step_count = arrRun[idx-(average_number-1):idx+1]
    average_goal_step_count.append(
        np.average(avg_list_goal_step_count))

axes.plot(average_goal_step_count)
axes.set_ylabel('Tỉ lệ tới đích sau {} lần thử'.format(numberRun))
axes.set_xlabel('Số lần thử')
axes.set_ylim(0, 1)  

# Định dạng văn bản hiển thị phần trăm tỉ lệ tới đích trung bình
axes.text(numberRun * 1/9, 1.01, 'Phần trăm tỉ lệ tới đích trung bình sau {} lần thử: {:.2f} %'.format(
    numberRun, round(np.mean(np.array(arrRun)) * 100, 4)), fontsize=12, ha='left')


deploy_chartRun = os.path.join(base_path, date_train, ('chartRundeploy' + '_' + str(
    datetime.now().hour) + 'h' + str(datetime.now().minute) + 'p'))

plt.savefig(deploy_chartRun)
plt.show()
