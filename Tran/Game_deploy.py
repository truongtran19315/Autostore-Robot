import pickle
from consts import *
import numpy as np
from Game_class import *

date_train = '2023-12-06_V1'
foler_train_path = os.path.join(base_path, date_train, 'q_table.pkl')

with open(foler_train_path, "rb") as f:
    q_table = pickle.load(f)

goal = 0
total_eps = 1000
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
    print('At step {}, robot arrival goal is number {}'.format(eps, goal), end='\r')
    eps += 1

print('\nTotal goal count: {} %'.format(goal * 100/total_eps))
