import os
import pickle
from Game_class import *
import cv2

videoFile_path = getlogVideo_path(getlogVersion(base_path))
recordVideo = cv2.VideoWriter(videoFile_path,
                            cv2.VideoWriter_fourcc(*'XVID'),
                            GAME_SETTING.FPS,
                            (GAME_SETTING.SCREEN_WIDTH, GAME_SETTING.SCREEN_HEIGHT))
Video = 1

folder_path = getlogVersion(base_path)
os.makedirs(folder_path, exist_ok=True)

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
    q_table = np.random.rand(*game.new_observation_shape + (ACTION_SPACE,))
    print("No existing Q-table found. Creating a new Q-table ... ")


print(f"Shape of Q-table: {q_table.shape}")
print(f"Init state:  {game.reset()}")
print("Start training....")

game.fig, axes = plt.subplots()
reward_records = []

pre_reward = 0
for i in range(n_epsilondes):
    print(f"Epsilon {i} :")
    game.reset()
    
    trackPos = Env.copy()
    
    reward, _, trackPosition, done = game.run_episode(q_table, Video, trackPos)
    reward_records.append(reward)
    trackPosition_path = getlogPosition_path(getlogVersion(base_path), done)
    compare_to_precious = reward - pre_reward
    pre_reward = reward
    compare_to_precious = 'reward - precious reward: ' + str(compare_to_precious)
    cv2.putText(trackPosition, compare_to_precious, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, COLOR.WHITE, 1)
    cv2.imwrite(trackPosition_path, trackPosition)
    recordVideo.write(trackPosition)    
    # TODO update diagram
    if (i % 10 == 0):
        game.creat_axes(axes, i)
        # plt.draw()
        # plt.pause(1)
        plt.savefig(diagram_path)
        print(f"Diagram saved to: {diagram_path}")

    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)
    print(f"Q-table saved to: {q_table_path}")
    print("-----------------------------------------------------")
    # for j in range(len(screenRecord)):
    #     recordVideo.write(screenRecord[j])
        
    if cv2.waitKey(1) & 0xFF == 'q':
        recordVideo.release()
        break

folder_path_done = folder_path + "_DONE"
os.rename(folder_path, folder_path_done)
folder_path = folder_path_done

with open(os.path.join(folder_path, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)
