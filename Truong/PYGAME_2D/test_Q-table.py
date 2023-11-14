from const import *
import numpy as np

lidarspace_shape = tuple(
    [LENGTH_LIDARSIGNAL] * SECTIONS_LIDARSPACE)

new_observation_shape = (ALPHA_SPACE, FWVELO_SPACE,
                         RVELO_SPACE) + lidarspace_shape

print(new_observation_shape + (ACTION_SPACE,))

q_table = np.random.rand(*new_observation_shape + (ACTION_SPACE,))
print(q_table)
