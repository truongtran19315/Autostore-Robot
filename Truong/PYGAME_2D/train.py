from pygame_2d import *

screen = np.zeros((720, 1280, 3), dtype=np.uint8)


high, low = 1, 0

alpha = 0, 2*math.pi    # góc quay
rVelo = PLAYER_SETTING.MIN_ROTATION_VELO, PLAYER_SETTING.MAX_ROTATION_VELO  # vận tốc quay
fVelo = -PLAYER_SETTING.MAX_FORWARD_VELO, PLAYER_SETTING.MAX_FORWARD_VELO    # vận tốc tiến
lowerBoundLidar = np.full((PLAYER_SETTING.CASTED_RAYS,), 0, dtype=float)
upperBoundLidar = np.full(
    (PLAYER_SETTING.CASTED_RAYS,), INT_INFINITY, dtype=float)

lowState = np.array[alpha[0], rVelo[0], fVelo[0], lowerBoundLidar]
upState = np.array[alpha[1], rVelo[1], fVelo[1], upperBoundLidar]


gamma = 0.99
alpha = 0.1
epsilon = 1
epsilon_decay = epsilon / 4000
n_epsilondes = 10000


new_observation_shape = (20, 20, 20, 20, 20)

bins = []
# for i in range(4):
#     item = np.linspace(
#         env.observation_space.low[i] if (i == 0) or (i == 2) else -4,
#         env.observation_space.high[i] if (i == 0) or (i == 2) else 4,
#         num=new_observation_shape[i],
#         endpoint=False)
#     item = np.delete(item, 0)
#     bins.append(item)
#     print(bins[i])

# define function to convert to discrete state


def get_discrete_state(s):
    # new_s = []
    # for i in range(4):
    #     new_s.append(np.digitize(s[i], bins[i]))
    # return new_s
    pass


q_table = np.zeros(new_observation_shape + ACTION_SPACE)
print(q_table)


def pick_sample(state):
    if np.random.random() > epsilon:
        action = np.argmax(q_table[tuple(state)])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action


def step(action):
    pass


reward_records = []
for i in range(n_epsilondes):
    done = False
    total_reward = 0

    game = PyGame2D(screen=screen)
    state = game.observe()

    s_dis = get_discrete_state(state)
    while not done:
        action = pick_sample(s_dis, i)
        #! ... (thêm) sau khi thực hiện action thì cập nhật lại...
        #! sửa lại hàm action bên PyGame2D
        state, reward, done = game.observe(), game.evaluate(), game.is_done()

        s_dis_next = get_discrete_state(state)

        # Update Q-Table
        maxQ = np.max(q_table[tuple(s_dis_next)])
        q_table[tuple(s_dis)][action] += alpha * (reward +
                                                  gamma * maxQ - q_table[tuple(s_dis)][action])

        s_dis = s_dis_next
        total_reward += reward

    # Update epsilon for each episode
    if epsilon - epsilon_decay >= 0:
        epsilon -= epsilon_decay

    # Record total rewards in episode
    print("Run episode {} with rewards {}".format(i, total_reward), end="\r")
    reward_records.append(total_reward)

print("\nDone")
