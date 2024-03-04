import math
import random
import os


class GAME_SETTING:
    # 20px = 1m
    # map 25x25 m
    SCREEN_WIDTH = 500
    SCREEN_HEIGHT = 500
    GRID_WIDTH = 20
    GOAL_WIDTH = 20
    FPS = 60
    DT = 0.5


class PLAYER_SETTING:
    RADIUS_OBJECT = 5
    RADIUS_LIDAR = 72  # độ dài của mỗi tia lidar 3m6 / 20px

    CASTED_RAYS = 360   # số lượng tia lidar
    CASTED_RAYS = 4   # số lượng tia lidar
    PI = math.pi
    HALF_PI = PI/2  # pi/2
    STEP_ANGLE = 2 * PI / CASTED_RAYS   # góc mỗi tia lidar
    
    INITIAL_X = 10
    INITIAL_Y = 10
    
    CURR_ANGLE = [0, PI/2, PI, 3*PI/2]
    INITIAL_ANGLE_INDEX = 0

    # MAX_STEP_PER_EPOCH = 5000
    GOAL = 2
    GONE = 1
    ALIVE = 0

    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 700


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
    GRID_COLOR = (220, 220, 220)
    LIDAR_DEF_COLOR = (0, 204, 204)


# RELATED TO REINFORCEMENT LEARNING
class ACTIONS:
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    TURN_BACK = 3
    DO_NOTHING = 4  # DIFFERENT WITH STOP


class EQUATION:
    NO_SOLUTION = 0
    ONE_SOLUTION = 1
    TWO_SOLUTION = 2


ACTION_SPACE = 5
ACTIONS_LIST = [
    ACTIONS.FORWARD,
    ACTIONS.TURN_RIGHT,
    ACTIONS.TURN_LEFT,
    ACTIONS.TURN_BACK,
    ACTIONS.DO_NOTHING,
]
MAX_EPISODE = 100000
INT_INFINITY = 99999


#! observe/ chuyen gia tri
LENGTH_LIDARSIGNAL = 2   # 0,1 #! edit for new map
SECTIONS_LIDARSPACE = 4  # ! 3 rays are: 0, 90 , 180

# set các khoảng rời rạc
DISTANCE_SPACE = 14
ALPHA_SPACE = 10

# Định nghĩa các tham số đầu vào
n_epsilondes = 10000  # Số lượng episode
alpha = 0.1  # Hệ số học
gamma = 0.99  # Hệ số giảm
epsilon_decay = 10/n_epsilondes  # Hệ số giảm epsilon
# epsilon_decay = 10/10000
epsilon = 0.9 - 0*epsilon_decay  # ! Xác suất khám phá
epsilon_min = 0.001  # Giá trị nhỏ nhất của epsilon

COUNTER = 200
