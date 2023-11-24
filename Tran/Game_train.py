import os
import pickle
from Game_class import *
import cv2


# folder_path = getlogVersion(base_path)
# os.makedirs(folder_path, exist_ok=True)

# videoFolderPath = os.path.join(folder_path, 'VIDEO')
# imageFolderPath = os.path.join(folder_path, 'IMAGE')
# logFolderPath = os.path.join(folder_path, 'LOG')
# os.makedirs(videoFolderPath, exist_ok=True)
# os.makedirs(imageFolderPath, exist_ok=True)
# os.makedirs(logFolderPath, exist_ok=True)
# log_path = getLog_path(logFolderPath)
# os.makedirs(log_path)

videoFile_path = getlogVideo_path(videoFolderPath)
recordVideo = cv2.VideoWriter(videoFile_path,
                              cv2.VideoWriter_fourcc(*'XVID'),
                              10,
                              (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))
Video = 1

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
version = ed

all_States = np.zeros(game.new_observation_shape)

for i in range(n_epsilondes):
    print(f"Epsilon {i}")
    game.reset()

    trackPos = Env.copy()

    log_path = getLog_path(logFolderPath, version)
    with open(log_path, 'a') as file:
        print(
            f'\n******************** Epsilon {i} ***********************', file=file)

    reward, _, trackPosition, done = game.run_episode(
        q_table, Video, trackPos, log_path, all_States)
    if done == PLAYER_SETTING.GOAL:
        goal_count += 1
    reward_records.append(reward)
    goal_count_str = 'goal counter: ' + str(goal_count)
    cv2.putText(trackPosition, goal_count_str, (50, 350),
                cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
    trackPosition_path = getlogPosition_path(imageFolderPath, done, version)
    cv2.imwrite(trackPosition_path, trackPosition)
    recordVideo.write(trackPosition)
    # # TODO update diagram
    # if (i % 10 == 0):
    #     game.creat_axes(axes, i)
    #     # plt.draw()
    #     # plt.pause(1)
    #     plt.savefig(diagram_path)
    #     # print(f"Diagram saved to: {diagram_path}")

    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)
    # print(f"Q-table saved to: {q_table_path}")
    # print("-----------------------------------------------------")
    # for j in range(len(screenRecord)):
    #     recordVideo.write(screenRecord[j])

    if cv2.waitKey(1) & 0xFF == 'q':
        recordVideo.release()
        break
    
    del trackPos
    del trackPosition
    version += 1


game.creat_axes(axes, i)
plt.savefig(diagram_path)
print(f"Diagram saved to: {diagram_path}")

folder_path_done = folder_path + "_DONE"
os.rename(folder_path, folder_path_done)
folder_path = folder_path_done

with open(os.path.join(folder_path, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)
