import gym
import numpy as np

class QLearningAgent:
    def __init__(self, env, num_bins=20):
        self.env = env
        self.num_bins = num_bins
        self.q_table = np.zeros((num_bins, env.action_space.n))
        
    def discretize_state(self, state):
        state_min = self.env.observation_space.low
        state_max = self.env.observation_space.high
        state_range = state_max - state_min
        bin_width = state_range / self.num_bins
        discrete_state = ((state - state_min) / bin_width).astype(int)
        return tuple(discrete_state)

    def discretize_action(self, action):
        action_min = self.env.action_space.low
        action_max = self.env.action_space.high
        action_range = action_max - action_min
        bin_width = action_range / self.num_bins
        discrete_action = ((action - action_min) / bin_width).astype(int)
        return max(0, min(self.num_bins - 1, discrete_action))

    def act(self, state):
        discrete_state = self.discretize_state(state)
        best_action = np.argmax(self.q_table[discrete_state])
        return best_action

    def learn(self, state, action, reward, next_state):
        self.q_table[state][action] += reward + np.max(self.q_table[next_state])

if __name__ == "__main__":
    env = gym.make("CartPole-v1", render_mode="human")
    agent = QLearningAgent(env)

    for episode in range(10000):
        state = env.reset()
        done = False
        while not done:
            action = agent.act(state)
            next_state, reward, done, info = env.step(action)  # Removed the extra underscore here
            agent.learn(state, action, reward, next_state)
            state = next_state  # Update the state with the new observation

    score = 0
    for _ in range(100):
        state = env.reset()
        done = False
        while not done:
            action = agent.act(state)
            next_state, reward, done, _ = env.step(action)
            score += reward
            state = next_state  # Update the state with the new observation
    print(score)
