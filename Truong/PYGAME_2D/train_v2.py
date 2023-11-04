from pygame_2d import *

screen = np.zeros((720, 1280, 3), dtype=np.uint8)

# Khởi tạo các tham số
gamma = 0.99
min_alpha = 0.01
max_alpha = 0.1
min_epsilon = 0.01
max_epsilon = 1
decay_rate = 0.01
n_epsilondes = 10000

# Khởi tạo bảng q_table với các giá trị ngẫu nhiên nhỏ
game = PyGame2D(screen=screen)
new_observation_shape = game.observe().shape + (ACTION_SPACE, )
q_table = np.random.uniform(low=-1, high=1, size=new_observation_shape)

# Hàm epsilon-greedy để chọn hành động


def pick_sample(state, epsilon):
    if np.random.random() > epsilon:
        # Thêm slice(None) để lấy tất cả các hành động
        action = np.argmax(q_table[tuple(state) + (slice(None), )])
    else:
        action = np.random.randint(0, ACTION_SPACE)
    return action

# Hàm học tăng cường để cập nhật hệ số học


def update_alpha(i):
    alpha = min_alpha + (max_alpha - min_alpha) * np.exp(-decay_rate * i)
    return alpha


reward_records = []
for i in range(n_epsilondes):
    done = False
    total_reward = 0
    state = game.observe()

    # Cập nhật epsilon và alpha theo số lần lặp
    epsilon = min_epsilon + (max_epsilon - min_epsilon) * \
        np.exp(-decay_rate * i)
    alpha = update_alpha(i)

    while not done:
        action = pick_sample(state, epsilon)
        game.action(action)
        next_state, reward, done = game.observe(), game.evaluate(), game.is_done()

        # Cập nhật bảng q_table theo công thức Q-learning
        # Thêm slice(None) để lấy tất cả các hành động
        maxQ = np.max(q_table[tuple(next_state) + (slice(None), )])
        q_table[tuple(state) + (action, )] += alpha * (reward +  # Thêm action để chỉ cập nhật hành động được chọn
                                                       gamma * maxQ - q_table[tuple(state) + (action, )])

        state = next_state
        total_reward += reward

    # Ghi lại tổng điểm thưởng trong mỗi lần lặp
    print("Run episode {} with rewards {}".format(i, total_reward), end="\r")
    reward_records.append(total_reward)

print("\nDone")
