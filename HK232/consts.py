import math
import random
import os


class GAME_SETTING:
    SCREEN_WIDTH = 500
    SCREEN_HEIGHT = 500
    GRID_WIDTH = 20
    GOAL_WIDTH = 20
    FPS = 60
    DT = 0.5


class PLAYER_SETTING:
    RADIUS_OBJECT = 9
    RADIUS_LIDAR = 360  # độ dài của mỗi tia lidar

    INITIAL_X = 30
    INITIAL_Y = 30

    CASTED_RAYS = 45
    CASTED_RAYS = 90
    CASTED_RAYS = 360   # số lượng tia lidar
    CASTED_RAYS = 180
    PI = math.pi
    HALF_PI = PI/2  # pi/2
    STEP_ANGLE = 2 * PI / CASTED_RAYS   # góc mỗi tia lidar
    STEP_ANGLE = PI / CASTED_RAYS   # góc mỗi tia lidar 

    # MAX_STEP_PER_EPOCH = 5000
    GOAL = 2
    GONE = 1
    ALIVE = 0

    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 1000

class OBSTACLE_SETTING:
    # MAX_INSTANCES = 15
    # RADIUS_OBJECT = 10
    # PROBABILITIES_ACTION = [0.1,
    #                         0.1,
    #                         0.1,
    #                         0.4,
    #                         0.2,
    #                         0.1]
    NUM_OBSTACLE = 10
    RECT_OBSTACLE = NUM_OBSTACLE//2
    CIRCLE_OBSTACLE = NUM_OBSTACLE//2
    
    MAX_RADIUS = 100
    MIN_RADIUS = PLAYER_SETTING.RADIUS_OBJECT
    
    MAX_HEIGHT = 100
    MIN_HEIGHT = PLAYER_SETTING.RADIUS_OBJECT
    MAX_WIDTH = 100
    MIN_WIDTH = PLAYER_SETTING.RADIUS_OBJECT
    
    MIN_DISTANCE_GOAL_VS_OBS = 50
    
    
class MAP_SETTING:
    MAP_DEFAULT = 0 
    RANDOM_MAP_OFF = 1
    RANDOM_MAP_ON = 2


class COLOR:
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    CYAN = (0, 255, 255)
    PINK = (255, 255, 0)
    PURPLE = (255, 0, 255)


# RELATED TO REINFORCEMENT LEARNING
class ACTIONS:
    TURN_RIGHT = 0
    TURN_LEFT = 1
    # STOP = 2
    FORWARD = 2
    BACKWARD = 3
    DO_NOTHING = 4  # DIFFERENT WITH STOP


class EQUATION:
    NO_SOLUTION = 0
    ONE_SOLUTION = 1
    TWO_SOLUTION = 2


ACTION_SPACE = 5
ACTIONS_LIST = [
    ACTIONS.TURN_RIGHT,
    ACTIONS.TURN_LEFT,
    # ACTIONS.STOP,
    ACTIONS.FORWARD,
    ACTIONS.BACKWARD,
    ACTIONS.DO_NOTHING,
]
MAX_EPISODE = 100000
INT_INFINITY = 99999


#! observe/ chuyen gia tri
LENGTH_LIDARSIGNAL = 4   # 0,1,2,3
SECTIONS_LIDARSPACE = 3  # ! chia lidar thành 4 vùng 0,1,2,3,4

# set các khoảng rời rạc
DISTANCE_SPACE = 5
ALPHA_SPACE = 10
FWVELO_SPACE = 4
RVELO_SPACE = 4

# Định nghĩa các tham số đầu vào
n_epsilondes = 1000000 - 256000 # Số lượng episode
alpha = 0.1  # Hệ số học
gamma = 0.99  # Hệ số giảm
# epsilon_decay = 10/n_epsilondes  # Hệ số giảm epsilon
epsilon_decay = 10/1000000
epsilon = 0.9 - 256000*epsilon_decay  # ! Xác suất khám phá
epsilon_min = 0.001  # Giá trị nhỏ nhất của epsilon

COUNTER = 200
