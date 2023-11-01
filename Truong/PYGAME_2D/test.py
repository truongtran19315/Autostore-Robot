import numpy as np
import random

random.seed(14)
lidarVisualize = [random.randint(0, 360) for i in range(0, 360)]
lidarVisualize = np.array(lidarVisualize)
print(lidarVisualize)
lidarVisualize[(lidarVisualize <= 120)] = 2
lidarVisualize[(lidarVisualize > 120) & (lidarVisualize <= 240)] = 1
lidarVisualize[(lidarVisualize > 240)] = 0
print(lidarVisualize)