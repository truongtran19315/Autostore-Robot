import pickle
from consts import *
import numpy as np
from Game_class import *


with open("C:\\Users\\truon\\PROJECTS\\PYTHON\\do-an-hk231\\Autostore-Robot\\Tran\\DATA\\2023-11-18_V1\\q_table.pkl", "rb") as f:
    q_table = pickle.load(f)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
robot = Game(screen)
Env = robot.getEnv()

state = robot.game.observe()
while True:
    action = np.argmax(q_table[tuple(state)])
    print(f'Action: {action}')
    robot.game.action(action)
    next_state = robot.game.observe()
    state = next_state
