import gym
import numpy as np

env = gym.make('CartPole-v1', render_mode= "human")

n_states = 10
states = np.linspace(-1, 1, n_states)

Q = np.zeros((n_states, n_states, env.action_space.n))

# Define the policy
def policy(state, Q, action_space_size):
    return np.argmax(Q[state])

# Discretize the states
def discretize(state):
    return tuple(discretize_helper(state[i], states) for i in range(len(state)))

def discretize_helper(val, array):
    return np.digitize(x=val, bins=array)

alpha = 0.5
gamma = 0.9
epsilon = 0.1
n_episodes = 5000

for e in range(n_episodes):
    # Initialize the state
    state = discretize(env.reset())

    done = False
    while not done:
        # Select the action
        if np.random.uniform(0, 1) < epsilon:
            action = env.action_space.sample()  # Explore action space
        else:
            action = policy(state, Q, env.action_space.n)  # Exploit learned values

        # Execute the action and get feedback
        next_state, reward, done, info, _ = env.step(action)
        next_state = discretize(next_state)

        old_value = Q[state][action]
        next_max = np.max(Q[next_state])

        # Update Q value for current state 
        new_value = (1 - alpha) * old_value + alpha * (reward + gamma * next_max)
        Q[state][action] = new_value

        state = next_state

env.close()
