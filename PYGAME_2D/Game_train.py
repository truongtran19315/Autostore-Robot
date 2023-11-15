import os
import pickle
from Game_class import *

folder_path = getlogVersion(base_path)
os.makedirs(folder_path, exist_ok=True)

q_table_filename = "q_table.pkl"
q_table_path = os.path.join(folder_path, q_table_filename)
diagram_filename = "diagram.png"
diagram_path = os.path.join(folder_path, diagram_filename)

screen = np.zeros((720, 1280, 3), dtype=np.uint8)
game = Game(screen)

if os.path.exists(q_table_path):
    with open(q_table_path, "rb") as f:
        q_table = pickle.load(f)
    print("Q-table loaded successfully!")
else:
    q_table = np.random.rand(*game.new_observation_shape + (ACTION_SPACE,))
    print("No existing Q-table found. Creating a new Q-table ... ")


print(f"Shape of Q-table: {q_table.shape}")
# print(q_table)
print(f"Init state:  {game.reset()}")
print("Start training....")

game.fig, axes = plt.subplots()
for i in range(n_epsilondes):
    state_count_change = 0
    print(f"Epsilon {i} :")
    game.reset()
    reward_records = game.run_episode(q_table)

    # TODO update diagram
    game.creat_axes(axes, i)
    plt.draw()
    plt.pause(0.1)
    plt.savefig(diagram_path)
    print(f"Diagram saved to: {diagram_path}")

    with open(q_table_path, "wb") as f:
        pickle.dump(q_table, f)
    print(f"Q-table saved to: {q_table_path}")
    print("---------------------------------")


folder_path_done = folder_path + "_DONE"
os.rename(folder_path, folder_path_done)
folder_path = folder_path_done

with open(os.path.join(folder_path, q_table_filename), "wb") as f:
    pickle.dump(q_table, f)