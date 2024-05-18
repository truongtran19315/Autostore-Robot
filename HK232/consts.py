import math

class GAME_SETTING:
    SCREEN_WIDTH = 1100
    SCREEN_HEIGHT = 700
    GRID_WIDTH = 10
    GOAL_WIDTH = 10
    FPS = 60
    DT = 0.5

class PLAYER_SETTING:
    RADIUS_OBJECT = 5
    RADIUS_LIDAR = 180
    CASTED_RAYS = 9
    PI = math.pi
    HALF_PI = PI / 2
    # STEP_ANGLE = 2 * PI / CASTED_RAYS
    STEP_ANGLE = PI / (CASTED_RAYS - 1)
    INITIAL_X = 55
    INITIAL_Y = 35
    CURR_ANGLE = [0, PI/2, PI, 3*PI/2]
    INITIAL_ANGLE_INDEX = 0
    GOAL = 2
    GONE = 1
    ALIVE = 0
    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 1500 #!1120 #!1750
    ALPHAGOAL_MIN = 0
    ALPHAGOAL_MAX = PI

class OBSTACLE_SETTING:
    NUM_OBSTACLE = 20
    MAX_RADIUS = 100
    MIN_RADIUS = PLAYER_SETTING.RADIUS_OBJECT
    MAX_HEIGHT = 100
    MIN_HEIGHT = PLAYER_SETTING.RADIUS_OBJECT
    MAX_WIDTH = 100
    MIN_WIDTH = PLAYER_SETTING.RADIUS_OBJECT
    MIN_DISTANCE_GOAL_VS_OBS = 0

class MAP_SETTING:
    MAP_DEFAULT = 0
    RANDOM_MAP = 1
    MAP_DEMO = 2

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

class ACTIONS:
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2
    # TURN_BACK = 3

class EQUATION:
    NO_SOLUTION = 0
    ONE_SOLUTION = 1
    TWO_SOLUTION = 2

MAX_EPISODE = 100000
INT_INFINITY = 99999

class SPACE:
    LIDAR_LENGTH_SEGMENT = [24, 50]
    DISTANCE_SPACE = 30 #!60 #!175
    ALPHA_SPACE = 6 #9
    REGION_LIDAR_SPAGE = 3
    SECTIONS_LIDARSPACE = 3  
    ACTION_SPACE = 3


n_epsilondes = 100000
alpha = 0.1
gamma = 0.99
epsilon_decay = 10 / n_epsilondes
epsilon = 0.9 - 0 * epsilon_decay
epsilon_min = 0.001

COUNTER = 400