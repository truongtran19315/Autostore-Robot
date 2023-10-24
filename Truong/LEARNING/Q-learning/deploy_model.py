import gym
import numpy as np

# Hàm chuyển đổi từ real state về q_state
def convert_state(real_state, q_table_size, q_table_segment_size):
    q_state = (real_state - env.observation_space.low) // q_table_segment_size
    return tuple(q_state.astype(np.int64))

def deploy_q_learning(env, q_table, max_start_state):
    env.reset()
    env.state = max_start_state

    done = False
    while not done:
        current_state = convert_state(env.state, q_table_size, q_table_segment_size)
        action = np.argmax(q_table[current_state])
        _, _, done, _ = env.step(action)
        env.render()

if __name__ == "__main__":
    env = gym.make("MountainCar-v0")
    env.reset()

    q_table_size = [20, 20]
    q_table_segment_size = (env.observation_space.high - env.observation_space.low) / q_table_size
    q_table = np.load("Truong/LEARNING/Q-learning/q_table.npy")
    max_start_state = np.load("Truong/LEARNING/Q-learning/max_start_state.npy")

    deploy_q_learning(env, q_table, max_start_state)

    env.close()
