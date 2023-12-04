import pickle
from consts import *
import numpy as np
from Game_class import *

date_train = '2023-12-04'
vesion_day = 4
foler_train = date_train + '_V' + str(vesion_day)
foler_train_path = os.path.join(base_path, foler_train, 'q_table.pkl')

with open(foler_train_path, "rb") as f:
    q_table = pickle.load(f)

# screen = np.zeros((720, 1280, 3), dtype=np.uint8)
# robot = Game(screen)
# Env = robot.getEnv()

# state = robot.game.observe()
# done = 0
# counter = 5000
# while done == 0 and counter > 0:
#     action = np.argmax(q_table[tuple(state)])
#     print(f'Action: {action}')
#     next_state, _, done = robot.step(action)
#     # robot.game.action(action)
#     robot.game.view()
#     state = next_state
#     counter -= 1

goal = 0
eps = 1000
while eps > 0:
    screen = np.zeros((720, 1280, 3), dtype=np.uint8)
    robot = Game(screen)
    Env = robot.getEnv()
    
    state = robot.game.observe()
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
    eps -= 1
    
print('goal count: ' + str(goal))