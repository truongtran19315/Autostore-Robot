import pickle
from consts import *
import numpy as np
from Game_class import *

date_train = '2023-11-29'
vesion_day = 2
foler_train = date_train + '_V' + str(vesion_day)
foler_train_path = os.path.join(base_path, foler_train, 'q_table.pkl')

with open(foler_train_path, "rb") as f:
    q_table = pickle.load(f)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
robot = Game(screen)
Env = robot.getEnv()

state = robot.game.observe()
done = 0
counter = 5000
while done == 0 and counter > 0:
    action = np.argmax(q_table[tuple(state)])
    print(f'Action: {action}')
    next_state, _, done = robot.step(action)
    # robot.game.action(action)
    robot.game.view()
    state = next_state
    counter -= 1
