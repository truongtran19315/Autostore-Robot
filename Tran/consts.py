import math
import random
import os


class GAME_SETTING:
    SCREEN_WIDTH = 1280
    SCREEN_HEIGHT = 720
    FPS = 60


class PLAYER_SETTING:
    RADIUS_OBJECT = 10
    RADIUS_LIDAR = 360  # độ dài của mỗi tia lidar

    INITIAL_X = GAME_SETTING.SCREEN_WIDTH//2
    INITIAL_Y = GAME_SETTING.SCREEN_HEIGHT//2

    # INITIAL_X = random.randint(15, 1265)
    # INITIAL_Y = random.randint(15, 705)

    # INITIAL_X = 603
    # INITIAL_Y = 384

    INITIAL_X = 963
    INITIAL_Y = 477

    INITIAL_X = 150
    INITIAL_Y = 500

    MAX_FORWARD_VELO = 50  # 0.22 m/s -> 22px/s  1m = 100px
    MAX_ROTATION_VELO = 0.5  # rad/s
    MIN_ROTATION_VELO = -MAX_ROTATION_VELO

    ACCELERATION_FORWARD = 5
    ACCELERATION_ROTATE = 0.05
    
    ACCELERATION_FORWARD = MAX_FORWARD_VELO/3
    ACCELERATION_ROTATE = MAX_ROTATION_VELO/3

    CASTED_RAYS = 45
    CASTED_RAYS = 90
    CASTED_RAYS = 360   # số lượng tia lidar
    CASTED_RAYS = 180
    PI = math.pi
    HALF_PI = PI/2  # pi/2
    STEP_ANGLE = 2 * PI / CASTED_RAYS   # góc mỗi tia lidar
    STEP_ANGLE = PI / CASTED_RAYS   # góc mỗi tia lidar

    Y_GOAL_POSITION = 10
    GOAL_POSITION = {"x": 950, "y": 315}
    GOAL_RADIUS = 30

    # MAX_STEP_PER_EPOCH = 5000
    GOAL = 2
    GONE = 1
    ALIVE = 0

    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 1000


class LANE_SETTING:
    WIDTH_OF_LANE_BORDER = 3

    OUTSIDE_LEFT_PADDING = 150
    OUTSIDE_RIGHT_PADDING = OUTSIDE_LEFT_PADDING
    OUTSIDE_TOP_PADDING = 150
    OUTSIDE_BOTTOM_PADDING = OUTSIDE_TOP_PADDING

    INSIDE_LEFT_PADDING = OUTSIDE_LEFT_PADDING + \
        int(2*PLAYER_SETTING.RADIUS_OBJECT*3)
    INSIDE_RIGHT_PADDING = INSIDE_LEFT_PADDING
    INSIDE_TOP_PADDING = OUTSIDE_TOP_PADDING + \
        int(2*PLAYER_SETTING.RADIUS_OBJECT*3)
    INSIDE_BOTTOM_PADDING = INSIDE_TOP_PADDING


class OBSTACLE_SETTING:
    MAX_INSTANCES = 15
    RADIUS_OBJECT = 10
    PROBABILITIES_ACTION = [0.1,
                            0.1,
                            0.1,
                            0.4,
                            0.2,
                            0.1]


class COLOR:
    BLACK = (0, 0, 0)
    WHITE = (255, 255, 255)
    RED = (0, 0, 255)
    GREEN = (0, 255, 0)
    BLUE = (255, 0, 0)
    CYAN = (0, 255, 255)
    PINK = (255, 0, 255)
    PURPLE = (255, 255, 0)


# RELATED TO REINFORCEMENT LEARNING
class ACTIONS:
    TURN_RIGHT_ACCELERATION = 0
    TURN_LEFT_ACCELERATION = 1
    STOP = 2
    FORWARD_ACCELERATION = 3
    BACKWARD_ACCELERATION = 4
    DO_NOTHING = 5  # DIFFERENT WITH STOP


class EQUATION:
    NO_SOLUTION = 0
    ONE_SOLUTION = 1
    TWO_SOLUTION = 2


ACTION_SPACE = 6
ACTIONS_LIST = [
    ACTIONS.TURN_RIGHT_ACCELERATION,
    ACTIONS.TURN_LEFT_ACCELERATION,
    ACTIONS.STOP,
    ACTIONS.FORWARD_ACCELERATION,
    ACTIONS.BACKWARD_ACCELERATION,
    ACTIONS.DO_NOTHING,
]
MAX_EPISODE = 100000
INT_INFINITY = 99999


#! observe/ chuyen gia tri
LENGTH_LIDARSIGNAL = 4   # 0,1,2,3
SECTIONS_LIDARSPACE = 3  # ! chia lidar thành 4 vùng 0,1,2,3,4

# set các khoảng rời rạc
DISTANCE_SPACE = 10  # ! khoang cach tu robot -> goal
DISTANCE_SPACE = 5  
ALPHA_SPACE = 10
# ALPHA_SPACE = 20
FWVELO_SPACE = 4
RVELO_SPACE = 4

# Định nghĩa các tham số đầu vào
ed = 0
n_epsilondes = 10000 - ed  # Số lượng episode
alpha = 0.1  # Hệ số học
gamma = 0.99  # Hệ số giảm
epsilon_decay = 10/n_epsilondes  # Hệ số giảm epsilon
epsilon = 0.9 - epsilon_decay*ed  # Xác suất khám phá
epsilon_min = 0.001  # Giá trị nhỏ nhất của epsilon

COUNTER = 200
