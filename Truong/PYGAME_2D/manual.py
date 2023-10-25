import gym
from dynamic_obstacle_avoidance import DynamicObstacleAvoidance  # Import môi trường
import numpy as np

screen = np.zeros((720, 1280, 3), dtype = np.uint8) 

# Khởi tạo môi trường
env = DynamicObstacleAvoidance(screen)


# Reset môi trường để lấy trạng thái khởi đầu
obs = env.reset()

