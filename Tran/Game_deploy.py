import pickle
from consts import *
import numpy as np
from Game_class import *
from datetime import datetime

date_train = '2023-12-06_V1'
foler_train_path = os.path.join(base_path, date_train, 'q_table.pkl')

with open(foler_train_path, "rb") as f:
    q_table = pickle.load(f)

numberRun = 100  # ! số lần chạy
arrRun = []
print('================ Start Run =========================')
for i in range(1, numberRun + 1):
    print('At attempt number {}'.format(i))
    goal = 0
    total_eps = 1000  # ! số bước thử trong mỗi lần chạy
    eps = 1
    while eps <= total_eps:
        screen = np.zeros((720, 1280, 3), dtype=np.uint8)
        robot = Game(screen)
        Env = robot.getEnv()

        state = robot.reset()
        done = 0
        counter = 200
        while done == 0 and counter > 0:
            action = np.argmax(q_table[tuple(state)])
            # print(f'Action: {action}')
            next_state, _, done = robot.step(action)
            # robot.game.action(action)
            # robot.game.view()
            state = next_state
            counter -= 1
            if done == 2:
                goal += 1
                break
        print('At step {}, Total robot arrival goal is number {}'.format(
            eps, goal), end='\r')
        eps += 1

    print('\nPercent to goal: {} %'.format(goal * 100/total_eps))
    print('====================================================')
    arrRun.append(goal * 100/total_eps)


fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(8, 6))
axes.plot(np.arange(1, numberRun + 1), np.array(arrRun), marker='o')
axes.set_ylabel(
    'Tỉ lệ tới đích sau {} lần thử (trong 1 lần chạy)'.format(total_eps))
axes.set_xlabel('Số lần chạy')
axes.set_xlim(0, numberRun + 1)
axes.set_xticks(np.arange(0, numberRun + 1, 1))
axes.set_ylim(0, 101)
axes.text(numberRun * 1/9, 30, 'Phần trăm tỉ lệ tới đích trung bình sau {} lần chạy: {} %'.format(
    numberRun, np.mean(np.array(arrRun))), fontsize=12, ha='left')

deploy_chartRun = os.path.join(folder_path, ('chartRundeploy' + '_' + str(
    datetime.now().hour) + 'h' + str(datetime.now().minute) + 'p'))
plt.savefig(deploy_chartRun)
print(f"Diagram saved to: {deploy_chartRun}")
plt.show()
