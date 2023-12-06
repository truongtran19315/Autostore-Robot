import os
import pickle
from Game_class import *
import cv2

videoFile_path = getlogVideo_path(videoFolderPath)
recordVideo = cv2.VideoWriter(videoFile_path,
                              cv2.VideoWriter_fourcc(*'XVID'),
                              10,
                              (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))

q_table_filename = "q_table.pkl"
q_table_path = os.path.join(folder_path, q_table_filename)
diagram_filename = "diagram.png"
diagram_path = os.path.join(folder_path, diagram_filename)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = Game(screen)


Env = game.getEnv()


if os.path.exists(q_table_path):
    with open(q_table_path, "rb") as f:
        q_table = pickle.load(f)
    print("Q-table loaded successfully!")
else:
    q_table = np.random.uniform(
        low=-1, high=1, size=(*game.new_observation_shape, ACTION_SPACE))
    print("No existing Q-table found. Creating a new Q-table ... ")


print(f"Shape of Q-table: {q_table.shape}")
print(f"Init state:  {game.reset()}")
print("Start training....")

game.fig, axes = plt.subplots()
reward_records = []

goal_count = 0

# to store last epsilon before program crash
last_epsilon_filename = "last_epsilon.pkl"
last_epsilon_path = os.path.join(folder_path, last_epsilon_filename)

if os.path.exists(last_epsilon_path):
    with open(last_epsilon_path, "rb") as f:
        last_epsilon = pickle.load(f)
else:
    last_epsilon = 0

all_States = np.zeros(game.new_observation_shape)

for i in range(last_epsilon + 1, n_epsilondes + last_epsilon + 1):
    # print(f"Epsilon {i}")
    print("Running Episode {}".format(i), end="\r")

    game.reset()

    trackPos = Env.copy()

    log_path = getLog_path(logFolderPath, last_epsilon)
    with open(log_path, 'a') as file:
        print(
            f'\n******************** Epsilon {i} ***********************', file=file)

    reward, trackPosition, done = game.run_episode(
        q_table, trackPos, log_path)
    if done == PLAYER_SETTING.GOAL:
        goal_count += 1
    reward_records.append(reward)
    goal_count_str = 'goal counter: ' + str(goal_count)
    cv2.putText(trackPosition, goal_count_str, (50, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
    trackPosition_path = getlogPosition_path(
        imageFolderPath, done, last_epsilon)
    cv2.imwrite(trackPosition_path, trackPosition)
    recordVideo.write(trackPosition)

    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)

    if cv2.waitKey(1) & 0xFF == 'q':
        recordVideo.release()
        break

    if i % 1000 == 0:
        with open(os.path.join(folder_path, q_table_filename), "wb") as f:
            pickle.dump(q_table, f)

        with open(os.path.join(folder_path, game.allStates_filename), "wb") as f:
            pickle.dump(game.allStates, f)

        with open(os.path.join(folder_path, game.record_state_change_filename), "wb") as f:
            pickle.dump(game.record_state_change, f)

        with open(os.path.join(folder_path, last_epsilon_filename), "wb") as f:
            pickle.dump(last_epsilon, f)
        game.creat_axes(axes, i, last_epsilon)
        plt.savefig(diagram_path)
        print(f"Diagram saved at eps {i}")

    del trackPos
    del trackPosition
    last_epsilon += 1

# folder_path_done = folder_path + "_DONE"
# os.rename(folder_path, folder_path_done)
# folder_path = folder_path_done

game.creat_axes(axes, i, last_epsilon)
plt.savefig(diagram_path)
print(f"Diagram saved to: {diagram_path}")

with open(os.path.join(folder_path, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)

with open(os.path.join(folder_path, game.allStates_filename), "wb") as f:
    pickle.dump(game.allStates, f)

with open(os.path.join(folder_path, game.record_state_change_filename), "wb") as f:
    pickle.dump(game.record_state_change, f)

with open(os.path.join(folder_path, last_epsilon_filename), "wb") as f:
    pickle.dump(last_epsilon, f)

print('done')
