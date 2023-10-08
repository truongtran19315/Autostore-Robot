import gym
from collections import deque
from gym import spaces
import numpy as np

class ConcatObs(gym.Wrapper):
    def __init__(self, env, k):
        gym.Wrapper.__init__(self, env)
        self.k = k
        self.frames = deque([], maxlen=k)
        shp = env.observation_space.shape
        self.observation_space = \
            spaces.Box(low=0, high=255, shape=((k,) + shp), dtype=env.observation_space.dtype)


def reset(self):
    ob = self.env.reset()
    for _ in range(self.k):     # _ là một biến lặp không sử dụng 
        self.frames.append(ob)  # thêm quan sát ob vào deque self.frames
    return self._get_ob()   

def step(self, action):
    ob, reward, done, info = self.env.step(action)
    self.frames.append(ob)     
    return self._get_ob(), reward, done, info

def _get_ob(self):
    return np.array(self.frames)


env = gym.make("BreakoutNoFrameskip-v4")
wrapped_env = ConcatObs(env, 4)
print("The new observation space is", wrapped_env.observation_space)    # Box(0, 255, (4, 210, 160, 3), uint8)

# Reset the Env
obs = wrapped_env.reset()
print("Intial obs is of the shape {}".format(obs.shape))

# Take one step
obs, _, _, _, _  = wrapped_env.step(2)
print("Obs after taking a step is {}".format(obs.shape))


import random 

class ObservationWrapper(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
    
    def observation(self, obs):
        # Normalise observation by 255
        return obs / 255.0

class RewardWrapper(gym.RewardWrapper):
    def __init__(self, env):
        super().__init__(env)
    
    def reward(self, reward):
        # Clip reward between 0 to 1
        return np.clip(reward, 0, 1)
    
class ActionWrapper(gym.ActionWrapper):
    def __init__(self, env):
        super().__init__(env)
    
    def action(self, action):
        if action == 3:
            return random.choice([0,1,2])
        else:
            return action

