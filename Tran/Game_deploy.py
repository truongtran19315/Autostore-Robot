import pickle
from consts import *
import numpy as np
from Game_class import *


with open("D:\\Workspace\\Github\\Autostore-Robot\\Tran\\DATA\\2023-11-21_V2\\q_table.pkl", "rb") as f:
    q_table = pickle.load(f)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
robot = Game(screen)
Env = robot.getEnv()

state = robot.game.observe()
done = 0

while done == 0:
    action = np.argmax(q_table[tuple(state)])
    print(f'Action: {action}')
    next_state, _, done = robot.step(action)
    # robot.game.action(action)
    robot.game.view()
    state = next_state
