import math
import random


class GAME_SETTING:
    SCREEN_WIDTH = 1280
    SCREEN_HEIGHT = 720
    FPS = 60


class PLAYER_SETTING:
    RADIUS_OBJECT = 10
    RADIUS_LIDAR = 360  # độ dài của mỗi tia lidar

    INITIAL_X = GAME_SETTING.SCREEN_WIDTH//2
    INITIAL_Y = GAME_SETTING.SCREEN_HEIGHT//2

    INITIAL_X = random.randint(15, 1265)
    INITIAL_Y = random.randint(15, 705)

    # INITIAL_X = 603
    # INITIAL_Y = 384

    # INITIAL_X = 963
    # INITIAL_Y = 477

    MAX_FORWARD_VELO = 100
    MAX_ROTATION_VELO = 1
    MIN_ROTATION_VELO = -MAX_ROTATION_VELO

    ACCELERATION_FORWARD = 5
    ACCELERATION_ROTATE = 0.05

    CASTED_RAYS = 45
    CASTED_RAYS = 90
    CASTED_RAYS = 360   # số lượng tia lidar
    PI = math.pi
    HALF_PI = PI/2  # pi/2
    STEP_ANGLE = 2 * PI / CASTED_RAYS   # góc mỗi tia lidar

    Y_GOAL_POSITION = 10
    GOAL_POSITION = {"x": 200, "y": 50}
    GOAL_RADIUS = 20

    # MAX_STEP_PER_EPOCH = 5000


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
SECTIONS_LIDARSPACE = 4  # ! chia lidar thành 4 vùng 0,1,2,3

# set các khoảng rời rạc
ALPHA_SPACE = 20
FWVELO_SPACE = 4
RVELO_SPACE = 4

# gamma = 0.99
# alpha = 0.1
# epsilon = 1
# epsilon_decay = epsilon / 4000
# n_epsilondes = 10000


# Định nghĩa các tham số đầu vào
n_epsilondes = 10000  # Số lượng episode
alpha = 0.1  # Hệ số học
gamma = 0.9  # Hệ số giảm
epsilon = 0.9  # Xác suất khám phá
epsilon_decay = 0.0001  # Hệ số giảm epsilon
epsilon_min = 0.001  # Giá trị nhỏ nhất của epsilon

base_path = "C:/Users/truon/PROJECTS/PYTHON/do-an-hk231/DATA"
COUNTER = 100
