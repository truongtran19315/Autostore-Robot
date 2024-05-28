import math
from telnetlib import FORWARD_X


Q_TABLE_PATH = "/home/truong/Documents/Autostore-Robot/HK232/DATA/2024-05-27_V1_DONE/q_table.pkl"

class SPACE:
    LIDAR_LENGTH_SEGMENT = [0.48, 1]
    DISTANCE_SPACE = 30
    ALPHA_SPACE = 4
    REGION_LIDAR_SPAGE = 3
    SECTIONS_LIDARSPACE = 3 
    ACTION_SPACE = 3


class PLAYER_SETTING:   
    #! init robot point
    X_INIT_POS = -9.9
    Y_INIT_POS = 6.3
    #! Goal point
    X_GOAL = 4 # 0.9
    Y_GOAL = 2.5 # -2.1
    
    DISTANCEGOAL_MIN = 0
    DISTANCEGOAL_MAX = 30
    ALPHAGOAL_MIN = 0
    ALPHAGOAL_MAX = math.pi    
    # lidar ray
    CASTED_RAYS = 9
    STEP_ANGLE = 180 / (CASTED_RAYS - 1)

class ACTIONS:
    FORWARD = 0
    TURN_RIGHT = 1
    TURN_LEFT = 2

class PARAM_ROBOT:
    ROTATE_SPEED = 0.4          # Tốc độ quay
    ADJUST_ROTATE_SPEED = 0.08   # Tốc độ chiều chỉnh góc quay
    DIATANCE_FORWARD = 0.18     # Khoảng tiến tới trước của robot
    FORWARD_SPEED = 0.10         # Tốc độ tiến
    
    
# Ngưỡng va chạm
COLLISION_THRESHOLD = 0.1

# Ngưỡng khi đạt mục tiêu
GOAL_THRESHOLD = 1

ACTION_TIMEOUT = 10