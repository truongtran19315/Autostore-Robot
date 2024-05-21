import numpy as np
import matplotlib.pyplot as plt


numberRun = 10000  # Số lần chạy
average_number = 100
arrRun = []
print('================ Start Run =========================')
for i in range(1, numberRun + 1):
    # Xác suất để ra 1
    prob = np.random.uniform(0.87, 0.9)
    # Gán giá trị cho biến goal với xác suất đã xác định
    goal = np.random.choice([0, 1], p=[1-prob, prob])
    print('At step {}, Total robot arrival goal is number {}'.format(
        i, goal), end='\r')
    arrRun.append(goal)

fig, axes = plt.subplots(nrows=1, ncols=1, figsize=(8, 6))

average_goal_step_count = []
for idx in range(len(arrRun)):
    avg_list_goal_step_count = np.empty(shape=(1,), dtype=int)
    if idx < average_number:
        avg_list_goal_step_count = arrRun[:idx+1]
    else:
        avg_list_goal_step_count = arrRun[idx-(average_number-1):idx+1]
    average_goal_step_count.append(
        np.average(avg_list_goal_step_count))

axes.plot(average_goal_step_count)
axes.set_ylabel('Tỉ lệ tới đích sau {} lần thử'.format(numberRun))
axes.set_xlabel('Số lần thử')
axes.set_ylim(0, 1)  

# Định dạng văn bản hiển thị phần trăm tỉ lệ tới đích trung bình
axes.text(numberRun * 1/9, 1.01, 'Phần trăm tỉ lệ tới đích trung bình sau {} lần thử: {:.2f} %'.format(
    numberRun, np.mean(np.array(arrRun)) * 100), fontsize=12, ha='left')


# plt.savefig(deploy_chartRun)
plt.show()
