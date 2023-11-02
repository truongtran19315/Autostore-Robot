from pygame_2d import *

screen = np.zeros((720, 1280, 3), dtype=np.uint8)


gamma = 0.99
alpha = 0.1
epsilon = 1
epsilon_decay = epsilon / 4000
n_epsilondes = 10000


def pick_sample(state):
    if np.random.random() > epsilon:
        action = np.argmax(q_table[tuple(state)])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action


game = PyGame2D(screen=screen)
new_observation_shape = game.observe().shape

q_table = np.zeros(new_observation_shape + ACTION_SPACE)


reward_records = []
for i in range(n_epsilondes):
    done = False
    total_reward = 0
    state = game.observe()

    while not done:
        action = pick_sample(state, i)
        game.action(action)
        next_state, reward, done = game.observe(), game.evaluate(), game.is_done()

        # Update Q-Table
        maxQ = np.max(q_table[tuple(next_state)])
        q_table[tuple(state)][action] += alpha * (reward +
                                                  gamma * maxQ - q_table[tuple(state)][action])

        state = next_state
        total_reward += reward

    # Update epsilon for each episode
    if epsilon - epsilon_decay >= 0:
        epsilon -= epsilon_decay

    # Record total rewards in episode
    print("Run episode {} with rewards {}".format(i, total_reward), end="\r")
    reward_records.append(total_reward)

print("\nDone")
